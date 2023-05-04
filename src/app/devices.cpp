#include "defs.hpp"

dev_entry_t imu_devices[] = {
    {DEV_NONE, "no device set"},
    {DEV_FXOS8700_FXAS21002C, "FXOS8700/FXAS21002C"},
    {DEV_MPU6886, "MPU6886"},
    {DEV_MPU9250_DMP, "MPU9250"},
    {DEV_MPU6050, "MPU6050"},
    {DEV_BMI270_BMM150, "BMI270/BMM150"},
    {DEV_ICM20948, "ICM20948"},
    {DEV_BNO08x, "BNO08x"},
    // {DEV_SH200Q, "SH200Q"},
    {DEV_ENDMARK, ""},
};
