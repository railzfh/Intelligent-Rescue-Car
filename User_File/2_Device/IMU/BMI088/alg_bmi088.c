/**
 * @file alg_bmi088.c
 * @brief BMI088 四元数 EKF、矩阵滤波与七参数转换。
 * 基于 Wang Hongxi 的 QuaternionEKF (V1.2.0)、kalman_filter (V1.2.2)、BMI_IMU (V2.0.0)。
 */
#include "alg_bmi088.h"
#include "arm_math.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define TRUE 1
#define FALSE 0

#define mat arm_matrix_instance_f32
#define Matrix_Init arm_mat_init_f32
#define Matrix_Add arm_mat_add_f32
#define Matrix_Subtract arm_mat_sub_f32
#define Matrix_Multiply arm_mat_mult_f32
#define Matrix_Transpose arm_mat_trans_f32
#define Matrix_Inverse arm_mat_inverse_f32


typedef struct BMI088_Kalman
{
    float *FilteredValue;
    float *MeasuredVector;
    float *ControlVector;

    uint8_t xhatSize;
    uint8_t uSize;
    uint8_t zSize;

    uint8_t UseAutoAdjustment;
    uint8_t MeasurementValidNum;

    uint8_t *MeasurementMap;      // 量测与状态的关系 how measurement relates to the state
    float *MeasurementDegree;     // 测量值对应H矩阵元素值 elements of each measurement in H
    float *MatR_DiagonalElements; // 量测方差 variance for each measurement
    float *StateMinVariance;      // 最小方差 避免方差过度收敛 suppress filter excessive convergence
    uint8_t *temp;

    // 配合用户定义函数使用,作为标志位用于判断是否要跳过标准KF中五个环节中的任意一个
    uint8_t SkipEq1, SkipEq2, SkipEq3, SkipEq4, SkipEq5;

    // definiion of struct mat: rows & cols & pointer to vars
    mat xhat;      // x(k|k)
    mat xhatminus; // x(k|k-1)
    mat u;         // control vector u
    mat z;         // measurement vector z
    mat P;         // covariance matrix P(k|k)
    mat Pminus;    // covariance matrix P(k|k-1)
    mat F, FT;     // state transition matrix F FT
    mat B;         // control matrix B
    mat H, HT;     // measurement matrix H
    mat Q;         // process noise covariance matrix Q
    mat R;         // measurement noise covariance matrix R
    mat K;         // kalman gain  K
    mat S, temp_matrix, temp_matrix1, temp_vector, temp_vector1;

    int8_t MatStatus;

    // 用户定义函数,可以替换或扩展基准KF的功能
    void (*User_Func0_f)(struct BMI088_Kalman *kf);
    void (*User_Func1_f)(struct BMI088_Kalman *kf);
    void (*User_Func2_f)(struct BMI088_Kalman *kf);
    void (*User_Func3_f)(struct BMI088_Kalman *kf);
    void (*User_Func4_f)(struct BMI088_Kalman *kf);
    void (*User_Func5_f)(struct BMI088_Kalman *kf);
    void (*User_Func6_f)(struct BMI088_Kalman *kf);

    // 矩阵存储空间指针
    float *xhat_data, *xhatminus_data;
    float *u_data;
    float *z_data;
    float *P_data, *Pminus_data;
    float *F_data, *FT_data;
    float *B_data;
    float *H_data, *HT_data;
    float *Q_data;
    float *R_data;
    float *K_data;
    float *S_data, *temp_matrix_data, *temp_matrix_data1, *temp_vector_data, *temp_vector_data1;
} BMI088_Kalman_t;


typedef struct
{
    uint8_t Initialized;
    BMI088_Kalman_t filter;
    uint8_t ConvergeFlag;
    uint8_t StableFlag;
    uint64_t ErrorCount;
    uint64_t UpdateCount;

    float q[4];        // 四元数估计值
    float GyroBias[3]; // 陀螺仪零偏估计值

    float Gyro[3];
    float Accel[3];

    float OrientationCosine[3];

    float accLPFcoef;
    float gyro_norm;
    float accl_norm;
    float AdaptiveGainScale;

    float Roll;
    float Pitch;
    float Yaw;

    float YawTotalAngle;

    float Q1; // 四元数更新过程噪声
    float Q2; // 陀螺仪零偏过程噪声
    float R;  // 加速度计量测噪声

    float dt; // 姿态更新周期
    mat ChiSquare;
    float ChiSquare_Data[1];      // 卡方检验检测函数
    float ChiSquareTestThreshold; // 卡方检验阈值
    float lambda;                 // 渐消因子

    int16_t YawRoundCount;

    float YawAngleLast;
} BMI088_EKF_t;

static bool Kalman_Filter_Init(BMI088_Kalman_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize);
static void Kalman_Filter_Measure(BMI088_Kalman_t *kf);
static void Kalman_Filter_xhatMinusUpdate(BMI088_Kalman_t *kf);
static void Kalman_Filter_PminusUpdate(BMI088_Kalman_t *kf);
static void Kalman_Filter_SetK(BMI088_Kalman_t *kf);
static void Kalman_Filter_xhatUpdate(BMI088_Kalman_t *kf);
static void Kalman_Filter_P_Update(BMI088_Kalman_t *kf);
static float *Kalman_Filter_Update(BMI088_Kalman_t *kf);

// 单个 6 状态 EKF 使用固定内存，不依赖堆分配。
static float Filter_Memory[512];
static size_t Filter_Memory_Used;
static void *Filter_Allocate(size_t size)
{
    size = (size + 3U) & ~(size_t)3U;
    if (size > sizeof(Filter_Memory) - Filter_Memory_Used) { return NULL; }
    void *result = (uint8_t *)Filter_Memory + Filter_Memory_Used;
    Filter_Memory_Used += size;
    return result;
}
#undef user_malloc
#define user_malloc Filter_Allocate

static uint16_t sizeof_float, sizeof_double;

static void H_K_R_Adjustment(BMI088_Kalman_t *kf);

/**
 * @brief 初始化矩阵维度信息并为矩阵分配空间
 *
 * @param kf kf类型定义
 * @param xhatSize 状态变量维度
 * @param uSize 控制变量维度
 * @param zSize 观测量维度
 */
static bool Kalman_Filter_Init(BMI088_Kalman_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize)
{
    if (xhatSize != 6U || uSize != 0U || zSize != 3U) { return false; }
    Filter_Memory_Used = 0;
    sizeof_float = sizeof(float);
    sizeof_double = sizeof(double);

    kf->xhatSize = xhatSize;
    kf->uSize = uSize;
    kf->zSize = zSize;

    kf->MeasurementValidNum = 0;

    // measurement flags
    kf->MeasurementMap = (uint8_t *)user_malloc(sizeof(uint8_t) * zSize);
    if (kf->MeasurementMap == NULL) { return false; }
    memset(kf->MeasurementMap, 0, sizeof(uint8_t) * zSize);
    kf->MeasurementDegree = (float *)user_malloc(sizeof_float * zSize);
    if (kf->MeasurementDegree == NULL) { return false; }
    memset(kf->MeasurementDegree, 0, sizeof_float * zSize);
    kf->MatR_DiagonalElements = (float *)user_malloc(sizeof_float * zSize);
    if (kf->MatR_DiagonalElements == NULL) { return false; }
    memset(kf->MatR_DiagonalElements, 0, sizeof_float * zSize);
    kf->StateMinVariance = (float *)user_malloc(sizeof_float * xhatSize);
    if (kf->StateMinVariance == NULL) { return false; }
    memset(kf->StateMinVariance, 0, sizeof_float * xhatSize);
    kf->temp = (uint8_t *)user_malloc(sizeof(uint8_t) * zSize);
    if (kf->temp == NULL) { return false; }
    memset(kf->temp, 0, sizeof(uint8_t) * zSize);

    // filter data
    kf->FilteredValue = (float *)user_malloc(sizeof_float * xhatSize);
    if (kf->FilteredValue == NULL) { return false; }
    memset(kf->FilteredValue, 0, sizeof_float * xhatSize);
    kf->MeasuredVector = (float *)user_malloc(sizeof_float * zSize);
    if (kf->MeasuredVector == NULL) { return false; }
    memset(kf->MeasuredVector, 0, sizeof_float * zSize);
    kf->ControlVector = (float *)user_malloc(sizeof_float * uSize);
    if (kf->ControlVector == NULL) { return false; }
    memset(kf->ControlVector, 0, sizeof_float * uSize);

    // xhat x(k|k)
    kf->xhat_data = (float *)user_malloc(sizeof_float * xhatSize);
    if (kf->xhat_data == NULL) { return false; }
    memset(kf->xhat_data, 0, sizeof_float * xhatSize);
    Matrix_Init(&kf->xhat, kf->xhatSize, 1, (float *)kf->xhat_data);

    // xhatminus x(k|k-1)
    kf->xhatminus_data = (float *)user_malloc(sizeof_float * xhatSize);
    if (kf->xhatminus_data == NULL) { return false; }
    memset(kf->xhatminus_data, 0, sizeof_float * xhatSize);
    Matrix_Init(&kf->xhatminus, kf->xhatSize, 1, (float *)kf->xhatminus_data);

    if (uSize != 0)
    {
        // control vector u
        kf->u_data = (float *)user_malloc(sizeof_float * uSize);
        if (kf->u_data == NULL) { return false; }
        memset(kf->u_data, 0, sizeof_float * uSize);
        Matrix_Init(&kf->u, kf->uSize, 1, (float *)kf->u_data);
    }

    // measurement vector z
    kf->z_data = (float *)user_malloc(sizeof_float * zSize);
    if (kf->z_data == NULL) { return false; }
    memset(kf->z_data, 0, sizeof_float * zSize);
    Matrix_Init(&kf->z, kf->zSize, 1, (float *)kf->z_data);

    // covariance matrix P(k|k)
    kf->P_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    if (kf->P_data == NULL) { return false; }
    memset(kf->P_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->P, kf->xhatSize, kf->xhatSize, (float *)kf->P_data);

    // create covariance matrix P(k|k-1)
    kf->Pminus_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    if (kf->Pminus_data == NULL) { return false; }
    memset(kf->Pminus_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Pminus, kf->xhatSize, kf->xhatSize, (float *)kf->Pminus_data);

    // state transition matrix F FT
    kf->F_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    if (kf->F_data == NULL) { return false; }
    kf->FT_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    if (kf->FT_data == NULL) { return false; }
    memset(kf->F_data, 0, sizeof_float * xhatSize * xhatSize);
    memset(kf->FT_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->F, kf->xhatSize, kf->xhatSize, (float *)kf->F_data);
    Matrix_Init(&kf->FT, kf->xhatSize, kf->xhatSize, (float *)kf->FT_data);

    if (uSize != 0)
    {
        // control matrix B
        kf->B_data = (float *)user_malloc(sizeof_float * xhatSize * uSize);
        if (kf->B_data == NULL) { return false; }
        memset(kf->B_data, 0, sizeof_float * xhatSize * uSize);
        Matrix_Init(&kf->B, kf->xhatSize, kf->uSize, (float *)kf->B_data);
    }

    // measurement matrix H
    kf->H_data = (float *)user_malloc(sizeof_float * zSize * xhatSize);
    if (kf->H_data == NULL) { return false; }
    kf->HT_data = (float *)user_malloc(sizeof_float * xhatSize * zSize);
    if (kf->HT_data == NULL) { return false; }
    memset(kf->H_data, 0, sizeof_float * zSize * xhatSize);
    memset(kf->HT_data, 0, sizeof_float * xhatSize * zSize);
    Matrix_Init(&kf->H, kf->zSize, kf->xhatSize, (float *)kf->H_data);
    Matrix_Init(&kf->HT, kf->xhatSize, kf->zSize, (float *)kf->HT_data);

    // process noise covariance matrix Q
    kf->Q_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    if (kf->Q_data == NULL) { return false; }
    memset(kf->Q_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Q, kf->xhatSize, kf->xhatSize, (float *)kf->Q_data);

    // measurement noise covariance matrix R
    kf->R_data = (float *)user_malloc(sizeof_float * zSize * zSize);
    if (kf->R_data == NULL) { return false; }
    memset(kf->R_data, 0, sizeof_float * zSize * zSize);
    Matrix_Init(&kf->R, kf->zSize, kf->zSize, (float *)kf->R_data);

    // kalman gain K
    kf->K_data = (float *)user_malloc(sizeof_float * xhatSize * zSize);
    if (kf->K_data == NULL) { return false; }
    memset(kf->K_data, 0, sizeof_float * xhatSize * zSize);
    Matrix_Init(&kf->K, kf->xhatSize, kf->zSize, (float *)kf->K_data);

    kf->S_data = (float *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
    if (kf->S_data == NULL) { return false; }
    kf->temp_matrix_data = (float *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
    if (kf->temp_matrix_data == NULL) { return false; }
    kf->temp_matrix_data1 = (float *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
    if (kf->temp_matrix_data1 == NULL) { return false; }
    kf->temp_vector_data = (float *)user_malloc(sizeof_float * kf->xhatSize);
    if (kf->temp_vector_data == NULL) { return false; }
    kf->temp_vector_data1 = (float *)user_malloc(sizeof_float * kf->xhatSize);
    if (kf->temp_vector_data1 == NULL) { return false; }
    Matrix_Init(&kf->S, kf->xhatSize, kf->xhatSize, (float *)kf->S_data);
    Matrix_Init(&kf->temp_matrix, kf->xhatSize, kf->xhatSize, (float *)kf->temp_matrix_data);
    Matrix_Init(&kf->temp_matrix1, kf->xhatSize, kf->xhatSize, (float *)kf->temp_matrix_data1);
    Matrix_Init(&kf->temp_vector, kf->xhatSize, 1, (float *)kf->temp_vector_data);
    Matrix_Init(&kf->temp_vector1, kf->xhatSize, 1, (float *)kf->temp_vector_data1);

    kf->SkipEq1 = 0;
    kf->SkipEq2 = 0;
    kf->SkipEq3 = 0;
    kf->SkipEq4 = 0;
    kf->SkipEq5 = 0;
    return true;
}

static void Kalman_Filter_Measure(BMI088_Kalman_t *kf)
{
    // 矩阵H K R根据量测情况自动调整
    // matrix H K R auto adjustment
    if (kf->UseAutoAdjustment != 0)
        H_K_R_Adjustment(kf);
    else
    {
        memcpy(kf->z_data, kf->MeasuredVector, sizeof_float * kf->zSize);
        memset(kf->MeasuredVector, 0, sizeof_float * kf->zSize);
    }

    if (kf->uSize != 0U) { memcpy(kf->u_data, kf->ControlVector, sizeof_float * kf->uSize); }
}

static void Kalman_Filter_xhatMinusUpdate(BMI088_Kalman_t *kf)
{
    if (!kf->SkipEq1)
    {
        if (kf->uSize > 0)
        {
            kf->temp_vector.numRows = kf->xhatSize;
            kf->temp_vector.numCols = 1;
            kf->MatStatus = Matrix_Multiply(&kf->F, &kf->xhat, &kf->temp_vector);
            kf->temp_vector1.numRows = kf->xhatSize;
            kf->temp_vector1.numCols = 1;
            kf->MatStatus = Matrix_Multiply(&kf->B, &kf->u, &kf->temp_vector1);
            kf->MatStatus = Matrix_Add(&kf->temp_vector, &kf->temp_vector1, &kf->xhatminus);
        }
        else
        {
            kf->MatStatus = Matrix_Multiply(&kf->F, &kf->xhat, &kf->xhatminus);
        }
    }
}

static void Kalman_Filter_PminusUpdate(BMI088_Kalman_t *kf)
{
    if (!kf->SkipEq2)
    {
        kf->MatStatus = Matrix_Transpose(&kf->F, &kf->FT);
        kf->MatStatus = Matrix_Multiply(&kf->F, &kf->P, &kf->Pminus);
        kf->temp_matrix.numRows = kf->Pminus.numRows;
        kf->temp_matrix.numCols = kf->FT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->FT, &kf->temp_matrix); // temp_matrix = F P(k-1) FT
        kf->MatStatus = Matrix_Add(&kf->temp_matrix, &kf->Q, &kf->Pminus);
    }
}
static void Kalman_Filter_SetK(BMI088_Kalman_t *kf)
{
    if (!kf->SkipEq3)
    {
        kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT); // z|x => x|z
        kf->temp_matrix.numRows = kf->H.numRows;
        kf->temp_matrix.numCols = kf->Pminus.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix); // temp_matrix = H·P'(k)
        kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
        kf->temp_matrix1.numCols = kf->HT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1); // temp_matrix1 = H·P'(k)·HT
        kf->S.numRows = kf->R.numRows;
        kf->S.numCols = kf->R.numCols;
        kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S); // S = H P'(k) HT + R
        kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);     // temp_matrix1 = inv(H·P'(k)·HT + R)
        kf->temp_matrix.numRows = kf->Pminus.numRows;
        kf->temp_matrix.numCols = kf->HT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix); // temp_matrix = P'(k)·HT
        kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);
    }
}
static void Kalman_Filter_xhatUpdate(BMI088_Kalman_t *kf)
{
    if (!kf->SkipEq4)
    {
        kf->temp_vector.numRows = kf->H.numRows;
        kf->temp_vector.numCols = 1;
        kf->MatStatus = Matrix_Multiply(&kf->H, &kf->xhatminus, &kf->temp_vector); // temp_vector = H xhat'(k)
        kf->temp_vector1.numRows = kf->z.numRows;
        kf->temp_vector1.numCols = 1;
        kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1); // temp_vector1 = z(k) - H·xhat'(k)
        kf->temp_vector.numRows = kf->K.numRows;
        kf->temp_vector.numCols = 1;
        kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector); // temp_vector = K(k)·(z(k) - H·xhat'(k))
        kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
    }
}
static void Kalman_Filter_P_Update(BMI088_Kalman_t *kf)
{
    if (!kf->SkipEq5)
    {
        kf->temp_matrix.numRows = kf->K.numRows;
        kf->temp_matrix.numCols = kf->H.numCols;
        kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
        kf->temp_matrix1.numCols = kf->Pminus.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->K, &kf->H, &kf->temp_matrix);                 // temp_matrix = K(k)·H
        kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->Pminus, &kf->temp_matrix1); // temp_matrix1 = K(k)·H·P'(k)
        kf->MatStatus = Matrix_Subtract(&kf->Pminus, &kf->temp_matrix1, &kf->P);
    }
}

/**
 * @brief 执行卡尔曼滤波黄金五式,提供了用户定义函数,可以替代五个中的任意一个环节,方便自行扩展为EKF/UKF/ESKF/AUKF等
 *
 * @param kf kf类型定义
 * @return float* 返回滤波值
 */
static float *Kalman_Filter_Update(BMI088_Kalman_t *kf)
{
    // 0. 获取量测信息
    Kalman_Filter_Measure(kf);
    if (kf->User_Func0_f != NULL)
        kf->User_Func0_f(kf);

    // 先验估计
    // 1. xhat'(k)= A·xhat(k-1) + B·u
    Kalman_Filter_xhatMinusUpdate(kf);
    if (kf->User_Func1_f != NULL)
        kf->User_Func1_f(kf);

    // 预测更新
    // 2. P'(k) = A·P(k-1)·AT + Q
    Kalman_Filter_PminusUpdate(kf);
    if (kf->User_Func2_f != NULL)
        kf->User_Func2_f(kf);

    if (kf->MeasurementValidNum != 0 || kf->UseAutoAdjustment == 0)
    {
        // 量测更新
        // 3. K(k) = P'(k)·HT / (H·P'(k)·HT + R)
        Kalman_Filter_SetK(kf);

        if (kf->User_Func3_f != NULL)
            kf->User_Func3_f(kf);

        // 融合
        // 4. xhat(k) = xhat'(k) + K(k)·(z(k) - H·xhat'(k))
        Kalman_Filter_xhatUpdate(kf);

        if (kf->User_Func4_f != NULL)
            kf->User_Func4_f(kf);

        // 修正方差
        // 5. P(k) = (1-K(k)·H)·P'(k) ==> P(k) = P'(k)-K(k)·H·P'(k)
        Kalman_Filter_P_Update(kf);
    }
    else
    {
        // 无有效量测,仅预测
        // xhat(k) = xhat'(k)
        // P(k) = P'(k)
        memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
        memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
    }

    // 自定义函数,可以提供后处理等
    if (kf->User_Func5_f != NULL)
        kf->User_Func5_f(kf);

    // 避免滤波器过度收敛
    // suppress filter excessive convergence
    for (uint8_t i = 0; i < kf->xhatSize; i++)
    {
        if (kf->P_data[i * kf->xhatSize + i] < kf->StateMinVariance[i])
            kf->P_data[i * kf->xhatSize + i] = kf->StateMinVariance[i];
    }

    memcpy(kf->FilteredValue, kf->xhat_data, sizeof_float * kf->xhatSize);

    if (kf->User_Func6_f != NULL)
        kf->User_Func6_f(kf);

    return kf->FilteredValue;
}

static void H_K_R_Adjustment(BMI088_Kalman_t *kf)
{
    kf->MeasurementValidNum = 0;

    memcpy(kf->z_data, kf->MeasuredVector, sizeof_float * kf->zSize);
    memset(kf->MeasuredVector, 0, sizeof_float * kf->zSize);

    // 识别量测数据有效性并调整矩阵H R K
    // recognize measurement validity and adjust matrices H R K
    memset(kf->R_data, 0, sizeof_float * kf->zSize * kf->zSize);
    memset(kf->H_data, 0, sizeof_float * kf->xhatSize * kf->zSize);
    for (uint8_t i = 0; i < kf->zSize; i++)
    {
        if (kf->z_data[i] != 0)
        {
            // 重构向量z
            // rebuild vector z
            kf->z_data[kf->MeasurementValidNum] = kf->z_data[i];
            kf->temp[kf->MeasurementValidNum] = i;
            // 重构矩阵H
            // rebuild matrix H
            kf->H_data[kf->xhatSize * kf->MeasurementValidNum + kf->MeasurementMap[i] - 1] = kf->MeasurementDegree[i];
            kf->MeasurementValidNum++;
        }
    }
    for (uint8_t i = 0; i < kf->MeasurementValidNum; i++)
    {
        // 重构矩阵R
        // rebuild matrix R
        kf->R_data[i * kf->MeasurementValidNum + i] = kf->MatR_DiagonalElements[kf->temp[i]];
    }

    // 调整矩阵维数
    // adjust the dimensions of system matrices
    kf->H.numRows = kf->MeasurementValidNum;
    kf->H.numCols = kf->xhatSize;
    kf->HT.numRows = kf->xhatSize;
    kf->HT.numCols = kf->MeasurementValidNum;
    kf->R.numRows = kf->MeasurementValidNum;
    kf->R.numCols = kf->MeasurementValidNum;
    kf->K.numRows = kf->xhatSize;
    kf->K.numCols = kf->MeasurementValidNum;
    kf->z.numRows = kf->MeasurementValidNum;
}

static BMI088_EKF_t BMI088_EKF;

static const float BMI088_EKF_Transition[36] = {1, 0, 0, 0, 0, 0,
                                       0, 1, 0, 0, 0, 0,
                                       0, 0, 1, 0, 0, 0,
                                       0, 0, 0, 1, 0, 0,
                                       0, 0, 0, 0, 1, 0,
                                       0, 0, 0, 0, 0, 1};
static float BMI088_EKF_Covariance[36] = {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 100, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 0.1, 100};
static float BMI088_EKF_Gain[18];
static float BMI088_EKF_Observation[18];

static float invSqrt(float x);
static void BMI088_EKF_Observe(BMI088_Kalman_t *kf);
static void BMI088_EKF_Linearize(BMI088_Kalman_t *kf);
static void BMI088_EKF_SetObservationMatrix(BMI088_Kalman_t *kf);
static void BMI088_EKF_CorrectState(BMI088_Kalman_t *kf);

/**
 * @brief Quaternion EKF initialization and some reference value
 * @param[in] process_noise1 quaternion process noise    10
 * @param[in] process_noise2 gyro bias process noise     0.001
 * @param[in] measure_noise  accel measure noise         1000000
 * @param[in] lambda         fading coefficient          0.9996
 * @param[in] lpf            lowpass filter coefficient  0
 */
static void BMI088_EKF_Init(float process_noise1, float process_noise2, float measure_noise, float lambda, float lpf)
{
    BMI088_EKF.Initialized = 1;
    BMI088_EKF.Q1 = process_noise1;
    BMI088_EKF.Q2 = process_noise2;
    BMI088_EKF.R = measure_noise;
    BMI088_EKF.ChiSquareTestThreshold = 1e-8;
    BMI088_EKF.ConvergeFlag = 0;
    BMI088_EKF.ErrorCount = 0;
    BMI088_EKF.UpdateCount = 0;
    if (lambda > 1)
    {
        lambda = 1;
    }
    BMI088_EKF.lambda = lambda;
    BMI088_EKF.accLPFcoef = lpf;

    // 初始化矩阵维度信息
    if (!Kalman_Filter_Init(&BMI088_EKF.filter, 6, 0, 3))
    {
        BMI088_EKF.Initialized = 0;
        return;
    }
    Matrix_Init(&BMI088_EKF.ChiSquare, 1, 1, (float *)BMI088_EKF.ChiSquare_Data);

    // 姿态初始化
    BMI088_EKF.filter.xhat_data[0] = 1;
    BMI088_EKF.filter.xhat_data[1] = 0;
    BMI088_EKF.filter.xhat_data[2] = 0;
    BMI088_EKF.filter.xhat_data[3] = 0;

    // 自定义函数初始化,用于扩展或增加kf的基础功能
    BMI088_EKF.filter.User_Func0_f = BMI088_EKF_Observe;
    BMI088_EKF.filter.User_Func1_f = BMI088_EKF_Linearize;
    BMI088_EKF.filter.User_Func2_f = BMI088_EKF_SetObservationMatrix;
    BMI088_EKF.filter.User_Func3_f = BMI088_EKF_CorrectState;

    // 设定标志位,用自定函数替换kf标准步骤中的SetK(计算增益)以及xhatupdate(后验估计/融合)
    BMI088_EKF.filter.SkipEq3 = TRUE;
    BMI088_EKF.filter.SkipEq4 = TRUE;

    memcpy(BMI088_EKF.filter.F_data, BMI088_EKF_Transition, sizeof(BMI088_EKF_Transition));
    memcpy(BMI088_EKF.filter.P_data, BMI088_EKF_Covariance, sizeof(BMI088_EKF_Covariance));
}

/**
 * @brief Quaternion EKF update
 * @param[in]       gyro x y z in rad/s
 * @param[in]       accel x y z in m/s²
 * @param[in]       update period in s
 */
static void BMI088_EKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    // 0.5(Ohm-Ohm^bias)*deltaT,用于更新工作点处的状态转移F矩阵
    static float halfgxdt, halfgydt, halfgzdt;
    static float accelInvNorm;
    if (!BMI088_EKF.Initialized)
    {
        BMI088_EKF_Init(10, 0.001, 1000000 * 10, 0.9996 * 0 + 1, 0);
    }

    /*   F, number with * represent vals to be set
     0      1*     2*     3*     4     5
     6*     7      8*     9*    10    11
    12*    13*    14     15*    16    17
    18*    19*    20*    21     22    23
    24     25     26     27     28    29
    30     31     32     33     34    35
    */
    BMI088_EKF.dt = dt;

    BMI088_EKF.Gyro[0] = gx - BMI088_EKF.GyroBias[0];
    BMI088_EKF.Gyro[1] = gy - BMI088_EKF.GyroBias[1];
    BMI088_EKF.Gyro[2] = gz - BMI088_EKF.GyroBias[2];

    // set F
    halfgxdt = 0.5f * BMI088_EKF.Gyro[0] * dt;
    halfgydt = 0.5f * BMI088_EKF.Gyro[1] * dt;
    halfgzdt = 0.5f * BMI088_EKF.Gyro[2] * dt;

    // 此部分设定状态转移矩阵F的左上角部分 4x4子矩阵,即0.5(Ohm-Ohm^bias)*deltaT,右下角有一个2x2单位阵已经初始化好了
    // 注意在predict步F的右上角是4x2的零矩阵,因此每次predict的时候都会调用memcpy用单位阵覆盖前一轮线性化后的矩阵
    memcpy(BMI088_EKF.filter.F_data, BMI088_EKF_Transition, sizeof(BMI088_EKF_Transition));

    BMI088_EKF.filter.F_data[1] = -halfgxdt;
    BMI088_EKF.filter.F_data[2] = -halfgydt;
    BMI088_EKF.filter.F_data[3] = -halfgzdt;

    BMI088_EKF.filter.F_data[6] = halfgxdt;
    BMI088_EKF.filter.F_data[8] = halfgzdt;
    BMI088_EKF.filter.F_data[9] = -halfgydt;

    BMI088_EKF.filter.F_data[12] = halfgydt;
    BMI088_EKF.filter.F_data[13] = -halfgzdt;
    BMI088_EKF.filter.F_data[15] = halfgxdt;

    BMI088_EKF.filter.F_data[18] = halfgzdt;
    BMI088_EKF.filter.F_data[19] = halfgydt;
    BMI088_EKF.filter.F_data[20] = -halfgxdt;

    // accel low pass filter,加速度过一下低通滤波平滑数据,降低撞击和异常的影响
    if (BMI088_EKF.UpdateCount == 0) // 如果是第一次进入,需要初始化低通滤波
    {
        BMI088_EKF.Accel[0] = ax;
        BMI088_EKF.Accel[1] = ay;
        BMI088_EKF.Accel[2] = az;
    }
    BMI088_EKF.Accel[0] = BMI088_EKF.Accel[0] * BMI088_EKF.accLPFcoef / (BMI088_EKF.dt + BMI088_EKF.accLPFcoef) + ax * BMI088_EKF.dt / (BMI088_EKF.dt + BMI088_EKF.accLPFcoef);
    BMI088_EKF.Accel[1] = BMI088_EKF.Accel[1] * BMI088_EKF.accLPFcoef / (BMI088_EKF.dt + BMI088_EKF.accLPFcoef) + ay * BMI088_EKF.dt / (BMI088_EKF.dt + BMI088_EKF.accLPFcoef);
    BMI088_EKF.Accel[2] = BMI088_EKF.Accel[2] * BMI088_EKF.accLPFcoef / (BMI088_EKF.dt + BMI088_EKF.accLPFcoef) + az * BMI088_EKF.dt / (BMI088_EKF.dt + BMI088_EKF.accLPFcoef);

    // set z,单位化重力加速度向量
    accelInvNorm = invSqrt(BMI088_EKF.Accel[0] * BMI088_EKF.Accel[0] + BMI088_EKF.Accel[1] * BMI088_EKF.Accel[1] + BMI088_EKF.Accel[2] * BMI088_EKF.Accel[2]);
    for (uint8_t i = 0; i < 3; i++)
    {
        BMI088_EKF.filter.MeasuredVector[i] = BMI088_EKF.Accel[i] * accelInvNorm; // 用加速度向量更新量测值
    }

    // get body state
    BMI088_EKF.gyro_norm = 1.0f / invSqrt(BMI088_EKF.Gyro[0] * BMI088_EKF.Gyro[0] +
                                        BMI088_EKF.Gyro[1] * BMI088_EKF.Gyro[1] +
                                        BMI088_EKF.Gyro[2] * BMI088_EKF.Gyro[2]);
    BMI088_EKF.accl_norm = 1.0f / accelInvNorm;

    // 如果角速度小于阈值且加速度处于设定范围内,认为运动稳定,加速度可以用于修正角速度
    // 稍后在最后的姿态更新部分会利用StableFlag来确定
    if (BMI088_EKF.gyro_norm < 0.3f && BMI088_EKF.accl_norm > 9.8f - 0.5f && BMI088_EKF.accl_norm < 9.8f + 0.5f)
    {
        BMI088_EKF.StableFlag = 1;
    }
    else
    {
        BMI088_EKF.StableFlag = 0;
    }

    // set Q R,过程噪声和观测噪声矩阵
    BMI088_EKF.filter.Q_data[0] = BMI088_EKF.Q1 * BMI088_EKF.dt;
    BMI088_EKF.filter.Q_data[7] = BMI088_EKF.Q1 * BMI088_EKF.dt;
    BMI088_EKF.filter.Q_data[14] = BMI088_EKF.Q1 * BMI088_EKF.dt;
    BMI088_EKF.filter.Q_data[21] = BMI088_EKF.Q1 * BMI088_EKF.dt;
    BMI088_EKF.filter.Q_data[28] = BMI088_EKF.Q2 * BMI088_EKF.dt;
    BMI088_EKF.filter.Q_data[35] = BMI088_EKF.Q2 * BMI088_EKF.dt;
    BMI088_EKF.filter.R_data[0] = BMI088_EKF.R;
    BMI088_EKF.filter.R_data[4] = BMI088_EKF.R;
    BMI088_EKF.filter.R_data[8] = BMI088_EKF.R;

    // 更新滤波矩阵，保留自定义观测与校正步骤。
    Kalman_Filter_Update(&BMI088_EKF.filter);

    // 获取融合后的数据,包括四元数和xy零飘值
    BMI088_EKF.q[0] = BMI088_EKF.filter.FilteredValue[0];
    BMI088_EKF.q[1] = BMI088_EKF.filter.FilteredValue[1];
    BMI088_EKF.q[2] = BMI088_EKF.filter.FilteredValue[2];
    BMI088_EKF.q[3] = BMI088_EKF.filter.FilteredValue[3];
    BMI088_EKF.GyroBias[0] = BMI088_EKF.filter.FilteredValue[4];
    BMI088_EKF.GyroBias[1] = BMI088_EKF.filter.FilteredValue[5];
    BMI088_EKF.GyroBias[2] = 0; // 大部分时候z轴通天,无法观测yaw的漂移

    // 利用四元数反解欧拉角
    BMI088_EKF.Yaw = atan2f(2.0f * (BMI088_EKF.q[0] * BMI088_EKF.q[3] + BMI088_EKF.q[1] * BMI088_EKF.q[2]), 2.0f * (BMI088_EKF.q[0] * BMI088_EKF.q[0] + BMI088_EKF.q[1] * BMI088_EKF.q[1]) - 1.0f) * 57.295779513f;
    BMI088_EKF.Pitch = atan2f(2.0f * (BMI088_EKF.q[0] * BMI088_EKF.q[1] + BMI088_EKF.q[2] * BMI088_EKF.q[3]), 2.0f * (BMI088_EKF.q[0] * BMI088_EKF.q[0] + BMI088_EKF.q[3] * BMI088_EKF.q[3]) - 1.0f) * 57.295779513f;
    BMI088_EKF.Roll = asinf(-2.0f * (BMI088_EKF.q[1] * BMI088_EKF.q[3] - BMI088_EKF.q[0] * BMI088_EKF.q[2])) * 57.295779513f;

    // get Yaw total, yaw数据可能会超过360,处理一下方便其他功能使用(如小陀螺)
    if (BMI088_EKF.Yaw - BMI088_EKF.YawAngleLast > 180.0f)
    {
        BMI088_EKF.YawRoundCount--;
    }
    else if (BMI088_EKF.Yaw - BMI088_EKF.YawAngleLast < -180.0f)
    {
        BMI088_EKF.YawRoundCount++;
    }
    BMI088_EKF.YawTotalAngle = 360.0f * BMI088_EKF.YawRoundCount + BMI088_EKF.Yaw;
    BMI088_EKF.YawAngleLast = BMI088_EKF.Yaw;
    BMI088_EKF.UpdateCount++; // 初始化低通滤波用,计数测试用
}

/**
 * @brief 用于更新线性化后的状态转移矩阵F右上角的一个4x2分块矩阵,稍后用于协方差矩阵P的更新;
 *        并对零漂的方差进行限制,防止过度收敛并限幅防止发散
 *
 * @param kf
 */
static void BMI088_EKF_Linearize(BMI088_Kalman_t *kf)
{
    static float q0, q1, q2, q3;
    static float qInvNorm;

    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    // quaternion normalize
    qInvNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    for (uint8_t i = 0; i < 4; i++)
    {
        kf->xhatminus_data[i] *= qInvNorm;
    }
    /*  F, number with * represent vals to be set
     0     1     2     3     4*     5*
     6     7     8     9    10*    11*
    12    13    14    15    16*    17*
    18    19    20    21    22*    23*
    24    25    26    27    28     29
    30    31    32    33    34     35
    */
    // set F
    kf->F_data[4] = q1 * BMI088_EKF.dt / 2;
    kf->F_data[5] = q2 * BMI088_EKF.dt / 2;

    kf->F_data[10] = -q0 * BMI088_EKF.dt / 2;
    kf->F_data[11] = q3 * BMI088_EKF.dt / 2;

    kf->F_data[16] = -q3 * BMI088_EKF.dt / 2;
    kf->F_data[17] = -q0 * BMI088_EKF.dt / 2;

    kf->F_data[22] = q2 * BMI088_EKF.dt / 2;
    kf->F_data[23] = -q1 * BMI088_EKF.dt / 2;

    // fading filter,防止零飘参数过度收敛
    kf->P_data[28] /= BMI088_EKF.lambda;
    kf->P_data[35] /= BMI088_EKF.lambda;

    // 限幅,防止发散
    if (kf->P_data[28] > 10000)
    {
        kf->P_data[28] = 10000;
    }
    if (kf->P_data[35] > 10000)
    {
        kf->P_data[35] = 10000;
    }
}

/**
 * @brief 在工作点处计算观测函数h(x)的Jacobi矩阵H
 *
 * @param kf
 */
static void BMI088_EKF_SetObservationMatrix(BMI088_Kalman_t *kf)
{
    static float doubleq0, doubleq1, doubleq2, doubleq3;
    /* H
     0     1     2     3     4     5
     6     7     8     9    10    11
    12    13    14    15    16    17
    last two cols are zero
    */
    // set H
    doubleq0 = 2 * kf->xhatminus_data[0];
    doubleq1 = 2 * kf->xhatminus_data[1];
    doubleq2 = 2 * kf->xhatminus_data[2];
    doubleq3 = 2 * kf->xhatminus_data[3];

    memset(kf->H_data, 0, sizeof_float * kf->zSize * kf->xhatSize);

    kf->H_data[0] = -doubleq2;
    kf->H_data[1] = doubleq3;
    kf->H_data[2] = -doubleq0;
    kf->H_data[3] = doubleq1;

    kf->H_data[6] = doubleq1;
    kf->H_data[7] = doubleq0;
    kf->H_data[8] = doubleq3;
    kf->H_data[9] = doubleq2;

    kf->H_data[12] = doubleq0;
    kf->H_data[13] = -doubleq1;
    kf->H_data[14] = -doubleq2;
    kf->H_data[15] = doubleq3;
}

/**
 * @brief 利用观测值和先验估计得到最优的后验估计
 *        加入了卡方检验以判断融合加速度的条件是否满足
 *        同时引入发散保护保证恶劣工况下的必要量测更新
 *
 * @param kf
 */
static void BMI088_EKF_CorrectState(BMI088_Kalman_t *kf)
{
    static float q0, q1, q2, q3;

    kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT); // z|x => x|z
    kf->temp_matrix.numRows = kf->H.numRows;
    kf->temp_matrix.numCols = kf->Pminus.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix); // temp_matrix = H·P'(k)
    kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
    kf->temp_matrix1.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1); // temp_matrix1 = H·P'(k)·HT
    kf->S.numRows = kf->R.numRows;
    kf->S.numCols = kf->R.numCols;
    kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S); // S = H P'(k) HT + R
    kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);     // temp_matrix1 = inv(H·P'(k)·HT + R)

    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    kf->temp_vector.numRows = kf->H.numRows;
    kf->temp_vector.numCols = 1;
    // 计算预测得到的重力加速度方向(通过姿态获取的)
    kf->temp_vector_data[0] = 2 * (q1 * q3 - q0 * q2);
    kf->temp_vector_data[1] = 2 * (q0 * q1 + q2 * q3);
    kf->temp_vector_data[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3; // temp_vector = h(xhat'(k))

    // 计算预测值和各个轴的方向余弦
    for (uint8_t i = 0; i < 3; i++)
    {
        BMI088_EKF.OrientationCosine[i] = acosf(fabsf(kf->temp_vector_data[i]));
    }

    // 利用加速度计数据修正
    kf->temp_vector1.numRows = kf->z.numRows;
    kf->temp_vector1.numCols = 1;
    kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1); // temp_vector1 = z(k) - h(xhat'(k))

    // chi-square test,卡方检验
    kf->temp_matrix.numRows = kf->temp_vector1.numRows;
    kf->temp_matrix.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix1, &kf->temp_vector1, &kf->temp_matrix); // temp_matrix = inv(H·P'(k)·HT + R)·(z(k) - h(xhat'(k)))
    kf->temp_vector.numRows = 1;
    kf->temp_vector.numCols = kf->temp_vector1.numRows;
    kf->MatStatus = Matrix_Transpose(&kf->temp_vector1, &kf->temp_vector); // temp_vector = z(k) - h(xhat'(k))'
    kf->MatStatus = Matrix_Multiply(&kf->temp_vector, &kf->temp_matrix, &BMI088_EKF.ChiSquare);
    // rk is small,filter converged/converging
    if (BMI088_EKF.ChiSquare_Data[0] < 0.5f * BMI088_EKF.ChiSquareTestThreshold)
    {
        BMI088_EKF.ConvergeFlag = 1;
    }
    // rk is bigger than thre but once converged
    if (BMI088_EKF.ChiSquare_Data[0] > BMI088_EKF.ChiSquareTestThreshold && BMI088_EKF.ConvergeFlag)
    {
        if (BMI088_EKF.StableFlag)
        {
            BMI088_EKF.ErrorCount++; // 载体静止时仍无法通过卡方检验
        }
        else
        {
            BMI088_EKF.ErrorCount = 0;
        }

        if (BMI088_EKF.ErrorCount > 50)
        {
            // 滤波器发散
            BMI088_EKF.ConvergeFlag = 0;
            kf->SkipEq5 = FALSE; // step-5 is cov mat P updating
        }
        else
        {
            //  残差未通过卡方检验 仅预测
            //  xhat(k) = xhat'(k)
            //  P(k) = P'(k)
            memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
            memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
            kf->SkipEq5 = TRUE; // part5 is P updating
            return;
        }
    }
    else // if divergent or rk is not that big/acceptable,use adaptive gain
    {
        // scale adaptive,rk越小则增益越大,否则更相信预测值
        if (BMI088_EKF.ChiSquare_Data[0] > 0.1f * BMI088_EKF.ChiSquareTestThreshold && BMI088_EKF.ConvergeFlag)
        {
            BMI088_EKF.AdaptiveGainScale = (BMI088_EKF.ChiSquareTestThreshold - BMI088_EKF.ChiSquare_Data[0]) / (0.9f * BMI088_EKF.ChiSquareTestThreshold);
        }
        else
        {
            BMI088_EKF.AdaptiveGainScale = 1;
        }
        BMI088_EKF.ErrorCount = 0;
        kf->SkipEq5 = FALSE;
    }

    // cal kf-gain K
    kf->temp_matrix.numRows = kf->Pminus.numRows;
    kf->temp_matrix.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix); // temp_matrix = P'(k)·HT
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);

    // implement adaptive
    for (uint8_t i = 0; i < kf->K.numRows * kf->K.numCols; i++)
    {
        kf->K_data[i] *= BMI088_EKF.AdaptiveGainScale;
    }
    for (uint8_t i = 4; i < 6; i++)
    {
        for (uint8_t j = 0; j < 3; j++)
        {
            kf->K_data[i * 3 + j] *= BMI088_EKF.OrientationCosine[i - 4] / 1.5707963f; // 1 rad
        }
    }

    kf->temp_vector.numRows = kf->K.numRows;
    kf->temp_vector.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector); // temp_vector = K(k)·(z(k) - H·xhat'(k))

    // 零漂修正限幅,一般不会有过大的漂移
    if (BMI088_EKF.ConvergeFlag)
    {
        for (uint8_t i = 4; i < 6; i++)
        {
            if (kf->temp_vector.pData[i] > 1e-2f * BMI088_EKF.dt)
            {
                kf->temp_vector.pData[i] = 1e-2f * BMI088_EKF.dt;
            }
            if (kf->temp_vector.pData[i] < -1e-2f * BMI088_EKF.dt)
            {
                kf->temp_vector.pData[i] = -1e-2f * BMI088_EKF.dt;
            }
        }
    }

    // 不修正yaw轴数据
    kf->temp_vector.pData[3] = 0;
    kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
}

/**
 * @brief EKF观测环节,其实就是把数据复制一下
 *
 * @param kf kf类型定义
 */
static void BMI088_EKF_Observe(BMI088_Kalman_t *kf)
{
    memcpy(BMI088_EKF_Covariance, kf->P_data, sizeof(BMI088_EKF_Covariance));
    memcpy(BMI088_EKF_Gain, kf->K_data, sizeof(BMI088_EKF_Gain));
    memcpy(BMI088_EKF_Observation, kf->H_data, sizeof(BMI088_EKF_Observation));
}

/**
 * @brief 自定义1/sqrt(x),速度更快
 *
 * @param x x
 * @return float
 */
static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    uint32_t i;
    memcpy(&i, &y, sizeof(i));
    i = 0x5f375a86 - (i >> 1);
    memcpy(&y, &i, sizeof(y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/**
 * 姿态适配，基于 Wang Hongxi 的 BMI_IMU.c (V2.0.0, 2022/2/23)。
 */

static bool BMI088_IsFinite(float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7f800000U) != 0x7f800000U;
}

bool BMI088_Attitude_Init(void)
{
    BMI088_EKF_Init(10, 0.001, 10000000, 1, 0);
    return BMI088_EKF.Initialized != 0;
}

bool BMI088_Attitude_Update(const BMI088_Sample_t *sample, float dt, BMI088_Attitude_t *attitude)
{
    if (!sample || !attitude || !(dt > 0.0f && dt <= 0.1f)) { return false; }
    BMI088_EKF_Update(sample->gyro_radps[0], sample->gyro_radps[1], sample->gyro_radps[2],
                      sample->accel_mps2[0], sample->accel_mps2[1], sample->accel_mps2[2], dt);
    // 原 EKF 的 Roll 绕 Y 轴、Pitch 绕 X 轴；对外统一为 pitch/roll。
    const BMI088_Attitude_t next = {
        BMI088_EKF.Yaw, BMI088_EKF.YawTotalAngle, BMI088_EKF.Roll, BMI088_EKF.Pitch,
        BMI088_EKF.Gyro[2], BMI088_EKF.Gyro[1], BMI088_EKF.Gyro[0]};
    if (!BMI088_IsFinite(next.yawAngle) || !BMI088_IsFinite(next.yawTotalAngle) ||
        !BMI088_IsFinite(next.pitchAngle) || !BMI088_IsFinite(next.rollAngle) ||
        !BMI088_IsFinite(next.yawSpeed) || !BMI088_IsFinite(next.pitchSpeed) || !BMI088_IsFinite(next.rollSpeed))
    {
        return false;
    }
    *attitude = next;
    return true;
}
