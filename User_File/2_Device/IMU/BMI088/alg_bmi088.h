#ifndef ALG_BMI088_H
#define ALG_BMI088_H

#include "dvc_bmi088.h"

// 模块内部姿态接口，由 dvc_bmi088.c 调用。
bool BMI088_Attitude_Init(void);
bool BMI088_Attitude_Update(const BMI088_Sample_t *sample, float dt, BMI088_Attitude_t *attitude);

#endif
