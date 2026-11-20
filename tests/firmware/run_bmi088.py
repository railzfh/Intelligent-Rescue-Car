"""Run the actual BMI088 driver/EKF on a host with mocked HAL and DWT."""
import argparse
import os
from pathlib import Path
import subprocess

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--fast-math', action='store_true', help='Check Release optimization')
parser.add_argument('--cc', default='cc', help='Native C compiler (Windows: x64 clang)')
parser.add_argument('--link', help='Windows lld-link executable')
parser.add_argument('--c-include', type=Path, help='Optional C standard library headers')
parser.add_argument('--dsp-source', type=Path, required=True, help='CMSIS-DSP Source/MatrixFunctions')
args = parser.parse_args()
if os.name == 'nt' and not args.link:
    parser.error('--link is required on Windows')
repo = Path(__file__).resolve().parents[2]
root = repo / 'build/bmi088-native-tests'
port = repo / 'User_File/2_Device/IMU/BMI088'

(root/'dsp').mkdir(parents=True, exist_ok=True)
(root/'main.h').write_text('''#ifndef MOCK_MAIN_H
#define MOCK_MAIN_H
#include <stdint.h>
typedef struct { int unused; } SPI_HandleTypeDef;
typedef struct { int unused; } GPIO_TypeDef;
typedef struct { uint32_t CYCCNT, CTRL, LAR; } Mock_DWT_Type;
typedef struct { uint32_t DEMCR; } Mock_CoreDebug_Type;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;
typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 } HAL_StatusTypeDef;
extern GPIO_TypeDef mock_gpio;
extern Mock_DWT_Type mock_dwt;
extern Mock_CoreDebug_Type mock_core_debug;
extern uint32_t SystemCoreClock;
#define DWT (&mock_dwt)
#define CoreDebug (&mock_core_debug)
#define CoreDebug_DEMCR_TRCENA_Msk (1U << 24)
#define DWT_CTRL_CYCCNTENA_Msk 1U
#define BMI088_ACCEL__SPI_CS_GPIO_Port (&mock_gpio)
#define BMI088_GYRO__SPI_CS_GPIO_Port (&mock_gpio)
#define BMI088_ACCEL__SPI_CS_Pin 1U
#define BMI088_GYRO__SPI_CS_Pin 8U
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, GPIO_PinState);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t, uint32_t);
#endif
''', encoding='ascii')
(root/'spi.h').write_text('#include "main.h"\nextern SPI_HandleTypeDef hspi2;\n', encoding='ascii')
(root/'arm_math.h').write_text('''#ifndef MOCK_ARM_MATH_H
#define MOCK_ARM_MATH_H
#include <stdint.h>
#include <stddef.h>
#define PI 3.14159265358979323846f
typedef float float32_t;
typedef enum { ARM_MATH_SUCCESS = 0, ARM_MATH_ARGUMENT_ERROR = -1, ARM_MATH_LENGTH_ERROR = -2,
               ARM_MATH_SIZE_MISMATCH = -3, ARM_MATH_NANINF = -4, ARM_MATH_SINGULAR = -5 } arm_status;
typedef struct { uint16_t numRows, numCols; float *pData; } arm_matrix_instance_f32;
void arm_mat_init_f32(arm_matrix_instance_f32 *, uint16_t, uint16_t, float *);
arm_status arm_mat_add_f32(const arm_matrix_instance_f32 *, const arm_matrix_instance_f32 *, arm_matrix_instance_f32 *);
arm_status arm_mat_sub_f32(const arm_matrix_instance_f32 *, const arm_matrix_instance_f32 *, arm_matrix_instance_f32 *);
arm_status arm_mat_mult_f32(const arm_matrix_instance_f32 *, const arm_matrix_instance_f32 *, arm_matrix_instance_f32 *);
arm_status arm_mat_trans_f32(const arm_matrix_instance_f32 *, arm_matrix_instance_f32 *);
arm_status arm_mat_inverse_f32(const arm_matrix_instance_f32 *, arm_matrix_instance_f32 *);
float arm_sin_f32(float);
float arm_cos_f32(float);
#endif
''', encoding='ascii')
(root/'dsp/matrix_functions.h').write_text('#include "arm_math.h"\n', encoding='ascii')
(root/'ucrt.def').write_text('''LIBRARY ucrtbase.dll
EXPORTS
memcpy
memset
memcmp
sqrtf
sinf
cosf
atan2f
asinf
acosf
''', encoding='ascii')
(root/'support.c').write_text('int _fltused;\nfloat fabsf(float x) { return x < 0 ? -x : x; }\n', encoding='ascii')

sources = [
    repo/'tests/firmware/test_bmi088.c',
    port/'dvc_bmi088.c', port/'alg_bmi088.c',
]
sources += [args.dsp_source/name for name in [
    'arm_mat_init_f32.c', 'arm_mat_add_f32.c', 'arm_mat_sub_f32.c',
    'arm_mat_mult_f32.c', 'arm_mat_trans_f32.c', 'arm_mat_inverse_f32.c']]
if os.name == 'nt':
    sources.append(root/'support.c')
includes = [root, port]
if args.c_include:
    includes.append(args.c_include)
objects = []
for i, source in enumerate(sources):
    obj = root/f'{i}_{source.stem}.obj'
    cmd = [args.cc, '-std=c11', '-Ofast' if args.fast_math else '-O1', '-ffreestanding', '-fno-builtin',
           '-DARM_MATH_MATRIX_CHECK', '-Wall', '-Wextra']
    for inc in includes:
        cmd += ['-I', str(inc)]
    subprocess.run(cmd + ['-c', str(source), '-o', str(obj)], check=True)
    objects.append(str(obj))
exe = root/('test_bmi088.exe' if os.name == 'nt' else 'test_bmi088')
if os.name == 'nt':
    subprocess.run([args.link, '/lib', '/machine:x64', '/def:'+str(root/'ucrt.def'),
                    '/out:'+str(root/'ucrt.lib')], check=True)
    subprocess.run([args.link, '/entry:main', '/subsystem:console', '/nodefaultlib',
                    '/out:'+str(exe), *objects, str(root/'ucrt.lib')], check=True)
else:
    subprocess.run([args.cc, *objects, '-lm', '-o', str(exe)], check=True)
result = subprocess.run([str(exe)])
print('BMI088 native test return code:', result.returncode)
if result.returncode:
    print('Failure corresponds to a line in tests/firmware/test_bmi088.c')
raise SystemExit(result.returncode)
