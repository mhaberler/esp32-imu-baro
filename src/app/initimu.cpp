#include "defs.hpp"
#include "probe.hpp"

#ifdef M5UNIFIED

const char *m5unified_imu_type(void) {
    const char *name = "none detected";

    switch (M5.Imu.getType()) {
        case m5::imu_t::imu_mpu6050:
            name = "MPU6050";
            break;
        case m5::imu_t::imu_mpu6886:
            name = "MPU6886";
            break;
        case m5::imu_t::imu_mpu9250:
            name = "MPU9250";
            break;
        case m5::imu_t::imu_sh200q:
            name = "SH200Q";
            break;
        case m5::imu_t::imu_bmi270:
            name = "BMI270/BMM150";
            break;
        default:
            name = "none";
            break;
    }
    return name;
}
#endif

bool initIMU(options_t &options, config_t &config) {
    const i2c_probe_t **dev;
    bool initialized   = false;
    esp_log_level_t ll = esp_log_level_get("*");
    esp_log_level_set("*", ESP_LOG_ERROR);

    switch (options.selected_imu) {
        case DEV_MPU9250_DMP:
            dev  = &config.dev[I2C_MPU9250];
            *dev = probe_dev("MPU9250", mpu9250_devs);
            if (*dev) {
                config.accel_type        = ACCEL_MPU9250_DMP;
                config.accel_name        = "MPU9250_DMP";
                config.gyro_type         = GYRO_MPU9250_DMP;
                config.gyro_name         = "MPU9250_DMP";
                config.magnetometer_type = MAG_MPU9250_DMP;
                config.magnetometer_name = "MPU9250_DMP";

                // Use setSensors to turn on or off MPU-9250 sensors.
                // Any of the following defines can be combined:
                // INV_XYZ_GYRO, INV_XYZ_ACCEL, INV_XYZ_COMPASS,
                // INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
                // Enable all sensors:
                mpu9250_dmp->setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL |
                                        INV_XYZ_COMPASS);

                // Use setGyroFSR() and setAccelFSR() to configure the
                // gyroscope and accelerometer full scale ranges.
                // Gyro options are +/- 250, 500, 1000, or 2000 dps
                mpu9250_dmp->setGyroFSR(250);  // Set gyro to 2000 dps
                // Accel options are +/- 2, 4, 8, or 16 g
                mpu9250_dmp->setAccelFSR(2);  // Set accel to +/-2g
                // Note: the MPU-9250's magnetometer FSR is set at
                // +/- 4912 uT (micro-tesla's)

                // setLPF() can be used to set the digital low-pass filter
                // of the accelerometer and gyroscope.
                // Can be any of the following: 188, 98, 42, 20, 10, 5
                // (values are in Hz).
                mpu9250_dmp->setLPF(188);  // Set LPF corner frequency to 5Hz

                // The sample rate of the accel/gyro can be set using
                // setSampleRate. Acceptable values range from 4Hz to 1kHz
                mpu9250_dmp->setSampleRate(100);  // Set sample rate to 10Hz

                // Likewise, the compass (magnetometer) sample rate can be
                // set using the setCompassSampleRate() function.
                // This value can range between: 1-100Hz
                mpu9250_dmp->setCompassSampleRate(100);  // Set mag rate to 10Hz

                initialized = true;
            }
            break;
        case DEV_MPU6050:
            mpu6050 = new Adafruit_MPU6050();
            dev     = &config.dev[I2C_MPU6050];
            *dev    = probe_dev("MPU6050", mpu6050_devs);
            if (*dev) {
                accelerometer = mpu6050->getAccelerometerSensor();
                gyroscope     = mpu6050->getGyroSensor();
                accelerometer->printSensorDetails();
                gyroscope->printSensorDetails();
                config.accel_type = ACCEL_MPU6050;
                config.accel_name = "MPU6050";
                config.gyro_type  = GYRO_MPU6050;
                config.gyro_name  = "MPU6050";
                initialized       = true;
            }
            break;

        case DEV_MPU6886:
            mpu6886 = new Adafruit_MPU6886();
            dev     = &config.dev[I2C_MPU6886];
            *dev    = probe_dev("MPU6886", mpu6886_devs);
            if (*dev) {
                accelerometer = mpu6886->getAccelerometerSensor();
                gyroscope     = mpu6886->getGyroSensor();
                // accelerometer->printSensorDetails();
                // gyroscope->printSensorDetails();
                config.accel_type = ACCEL_MPU6886;
                config.accel_name = "MPU6886";
                config.gyro_type  = GYRO_MPU6886;
                config.gyro_name  = "MPU6886";
                initialized       = true;
            }
            break;

        case DEV_BNO08x:
#if defined(DRV_BNO08x)
            bno08x = new Adafruit_BNO08x();
            dev    = &config.dev[I2C_BNO08X];
            *dev   = probe_dev("BNO08x", bno08x_devs);
            if (*dev) {
                for (int n = 0; n < bno08x->prodIds.numEntries; n++) {
                    LOGD("Part: {} Version: {}.{}.{} Build: {}",
                         bno08x->prodIds.entry[n].swPartNumber,
                         bno08x->prodIds.entry[n].swVersionMajor,
                         bno08x->prodIds.entry[n].swVersionMinor,
                         bno08x->prodIds.entry[n].swVersionPatch,
                         bno08x->prodIds.entry[n].swBuildNumber);
                }
                if (!bno08x->enableReport(SH2_ACCELEROMETER)) {
                    LOGD("Could not enable accelerometer");
                }
                if (!bno08x->enableReport(SH2_GYROSCOPE_CALIBRATED)) {
                    LOGD("Could not enable gyroscope");
                }
                if (!bno08x->enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
                    LOGD("Could not enable magnetic field calibrated");
                }
                // FIXME setReports(reportType, reportIntervalUs);
                // FIXME       initialized = true; //MISSING
            }
#endif
            break;

        case DEV_ICM20948:
            icm20948 = new Adafruit_ICM20948();
            dev      = &config.dev[I2C_ICM20949];
            *dev     = probe_dev("ICM20948", icm20948_devs);
            if (*dev) {
                config.accel_type        = ACCEL_ICM20948;
                config.accel_name        = "ICM20948";
                config.gyro_type         = GYRO_ICM20948;
                config.gyro_name         = "ICM20948";
                config.magnetometer_type = MAG_ICM20948;
                config.magnetometer_name = "ICM20948";

                accelerometer = icm20948->getAccelerometerSensor();
                magnetometer  = icm20948->getMagnetometerSensor();
                gyroscope     = icm20948->getGyroSensor();

                accelerometer->printSensorDetails();
                gyroscope->printSensorDetails();
                magnetometer->printSensorDetails();
                initialized = true;
            }
            break;

        case DEV_BMI270_BMM150:
            bmi270_bmm150 = new BoschSensorClass();

            config.dev[I2C_BMI270] = probe_dev("bmi270", bmi270_devs);
            if (config.dev[I2C_BMI270]) {
                config.accel_type = ACCEL_BMI270;
                config.accel_name = "BMI270";
                config.gyro_type  = GYRO_BMI270;
                config.gyro_name  = "BMI270";
            }

            config.dev[I2C_BMM150] = probe_dev("bmm150", bmm150_devs);
            if (config.dev[I2C_BMM150]) {
                config.magnetometer_type = MAG_BMM150;
                config.magnetometer_name = "BMM150";
            }

            if (!config.dev[I2C_BMM150] && !config.dev[I2C_BMI270]) {
                delete bmi270_bmm150;
            } else {
                initialized = true;
            }
            break;

        case DEV_FXOS8700_FXAS21002C:
            fxos = new Adafruit_FXOS8700(0x8700A, 0x8700B);
            dev  = &config.dev[I2C_FXOS];
            *dev = probe_dev("fxos8700", fxos_devs);
            if (*dev) {
                /* Set accelerometer range (optional, default is 2G) */
                // fxos.setAccelRange(ACCEL_RANGE_2G);

                /* Set the sensor mode (optional, default is hybrid mode) */
                // fxos.setSensorMode(ACCEL_ONLY_MODE);

                /* Set the magnetometer's oversampling ratio (optional, default
                 * is 7)
                 */
                // fxos.setMagOversamplingRatio(MAG_OSR_7);

                /* Set the output data rate (optional, default is 100Hz) */
                // fxos.setOutputDataRate(ODR_100HZ);
            }

            fxas = new Adafruit_FXAS21002C(0x0021002C);
            dev  = &config.dev[I2C_FXAS];
            *dev = probe_dev("fxas21002c", fxas_devs);
            if (*dev) {
                // FIXME sensor setup
                // Set gyro range.(optional,
                // default is 250 dps)
                // fxas.setRange(GYRO_RANGE_2000DPS);
            }

            // try these first as the NXP sensors are autodetected by
            // M5.Imu.begin() as type MPU9250 which is wrong...
            if (config.dev[I2C_FXOS] && config.dev[I2C_FXAS]) {
                config.accel_type        = ACCEL_FXOS8700;
                config.accel_name        = "FXOS8700";
                config.gyro_type         = GYRO_FXAS21002C;
                config.gyro_name         = "FXAS21002C";
                config.magnetometer_type = MAG_FXOS8700;
                config.magnetometer_name = "FXOS8700";

                accelerometer = fxos->getAccelerometerSensor();
                magnetometer  = fxos->getMagnetometerSensor();
                gyroscope     = fxas;

                accelerometer->printSensorDetails();
                gyroscope->printSensorDetails();
                magnetometer->printSensorDetails();
                initialized = true;
            }
            break;

#ifdef M5UNIFIED
        case DEV_M5UNIFIED:
            config.m5_imu_avail = M5.Imu.begin();
            if (config.m5_imu_avail) {
                //  M5.Imu.setRotation(options.rotation);

                LOGD("M5 IMU driver initialized, type={} rotation={}",
                     m5unified_imu_type(), (int)options.rotation);
                config.accel_type = ACCEL_M5IMU;
                config.accel_name = "M5IMU";
                config.gyro_type  = GYRO_M5IMU;
                config.gyro_name  = "M5IMU";
                config.magnetometer_type = MAG_M5IMU;
                config.magnetometer_name = "M5IMU";
                initialized = true;
            } else {
                LOGD("M5 IMU driver init failed");
            }
            break;
#endif
        case DEV_NONE:
        case DEV_ENDMARK:
            break;
    }
    esp_log_level_set("*", ll);
    return initialized;
}
