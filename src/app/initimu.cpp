#include "defs.hpp"

void initIMU(options_t &options, config_t &config) {

  switch (options.selected_imu) {
  case DEV_MPU9250_DMP:
#ifdef HAVE_WIRE1
    __mpwire = &Wire1;
    mpu9250_dmp = new MPU9250_DMP();
    config.mpu9250_dmp_avail = (mpu9250_dmp->begin() == INV_SUCCESS);
#endif
    if (!config.mpu9250_dmp_avail) {
      if (!mpu9250_dmp) {
        mpu9250_dmp = new MPU9250_DMP();
      }
      __mpwire = &Wire;
      config.mpu9250_dmp_avail = (mpu9250_dmp->begin() == INV_SUCCESS);
    }
    if (config.mpu9250_dmp_avail) {
      config.accel_type = ACCEL_MPU9250_DMP;
      config.accel_name = "MPU9250_DMP";
      config.gyro_type = GYRO_MPU9250_DMP;
      config.gyro_name = "MPU9250_DMP";
      config.magnetometer_type = MAG_MPU9250_DMP;
      config.magnetometer_name = "MPU9250_DMP";

      // Use setSensors to turn on or off MPU-9250 sensors.
      // Any of the following defines can be combined:
      // INV_XYZ_GYRO, INV_XYZ_ACCEL, INV_XYZ_COMPASS,
      // INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
      // Enable all sensors:
      mpu9250_dmp->setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

      // Use setGyroFSR() and setAccelFSR() to configure the
      // gyroscope and accelerometer full scale ranges.
      // Gyro options are +/- 250, 500, 1000, or 2000 dps
      mpu9250_dmp->setGyroFSR(250); // Set gyro to 2000 dps
      // Accel options are +/- 2, 4, 8, or 16 g
      mpu9250_dmp->setAccelFSR(2); // Set accel to +/-2g
      // Note: the MPU-9250's magnetometer FSR is set at
      // +/- 4912 uT (micro-tesla's)

      // setLPF() can be used to set the digital low-pass filter
      // of the accelerometer and gyroscope.
      // Can be any of the following: 188, 98, 42, 20, 10, 5
      // (values are in Hz).
      mpu9250_dmp->setLPF(188); // Set LPF corner frequency to 5Hz

      // The sample rate of the accel/gyro can be set using
      // setSampleRate. Acceptable values range from 4Hz to 1kHz
      mpu9250_dmp->setSampleRate(100); // Set sample rate to 10Hz

      // Likewise, the compass (magnetometer) sample rate can be
      // set using the setCompassSampleRate() function.
      // This value can range between: 1-100Hz
      mpu9250_dmp->setCompassSampleRate(100); // Set mag rate to 10Hz
    }
    break;
  case DEV_MPU6050:
#ifdef HAVE_WIRE1
    mpu6050 = new Adafruit_MPU6050();
    config.mpu6050_avail = mpu6050->begin(MPU6050_I2CADDR_DEFAULT, &Wire1);
#endif
    if (!config.mpu6050_avail) {
      if (!mpu6050) {
        mpu6050 = new Adafruit_MPU6050();
      }
      config.mpu6050_avail = mpu6050->begin(MPU6050_I2CADDR_DEFAULT, &Wire);
    }
    if (config.mpu6050_avail) {
      accelerometer = mpu6050->getAccelerometerSensor();
      gyroscope = mpu6050->getGyroSensor();
      accelerometer->printSensorDetails();
      gyroscope->printSensorDetails();
      config.accel_type = ACCEL_MPU6050;
      config.accel_name = "MPU6050";
      config.gyro_type = GYRO_MPU6050;
      config.gyro_name = "MPU6050";
    }
    break;
  case DEV_MPU6886:
#ifdef HAVE_WIRE1
    mpu6886 = new Adafruit_MPU6886();
    config.mpu6886_avail = mpu6886->begin(MPU6886_I2CADDR_DEFAULT, &Wire1);
#endif
    if (!config.mpu6886_avail) {
      if (!mpu6886) {
        mpu6886 = new Adafruit_MPU6886();
      }
      config.mpu6886_avail = mpu6886->begin(MPU6886_I2CADDR_DEFAULT, &Wire);
    }
    if (config.mpu6886_avail) {
      accelerometer = mpu6886->getAccelerometerSensor();
      gyroscope = mpu6886->getGyroSensor();
      // accelerometer->printSensorDetails();
      // gyroscope->printSensorDetails();
      config.accel_type = ACCEL_MPU6886;
      config.accel_name = "MPU6886";
      config.gyro_type = GYRO_MPU6886;
      config.gyro_name = "MPU6886";
    }
    break;
  case DEV_BNO08x:
#ifdef HAVE_WIRE1
    bno08x = new Adafruit_BNO08x();
    config.bno08x_avail = bno08x->begin_I2C(BNO08x_I2CADDR_DEFAULT, &Wire1);
#endif
    if (!config.bno08x_avail) {
      if (!bno08x) {
        bno08x = new Adafruit_BNO08x();
      }
      config.bno08x_avail = bno08x->begin_I2C(BNO08x_I2CADDR_DEFAULT, &Wire);
    }
    if (config.bno08x_avail) {
      LOGD("BNO08x Found!");

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
    }
    break;
  case DEV_ICM20948:
#ifdef HAVE_WIRE1
    icm20948 = new Adafruit_ICM20948();
    config.icm20948_avail =
        icm20948->begin_I2C(ICM20948_I2CADDR_DEFAULT, &Wire1);
#endif
    if (!config.icm20948_avail) {
      if (!icm20948) {
        icm20948 = new Adafruit_ICM20948();
      }
      config.icm20948_avail =
          icm20948->begin_I2C(ICM20948_I2CADDR_DEFAULT, &Wire);
    }
    if (config.icm20948_avail) {
      config.accel_type = ACCEL_ICM20948;
      config.accel_name = "ICM20948";
      config.gyro_type = GYRO_ICM20948;
      config.gyro_name = "ICM20948";
      config.magnetometer_type = MAG_ICM20948;
      config.magnetometer_name = "ICM20948";

      accelerometer = icm20948->getAccelerometerSensor();
      magnetometer = icm20948->getMagnetometerSensor();
      gyroscope = icm20948->getGyroSensor();

      accelerometer->printSensorDetails();
      gyroscope->printSensorDetails();
      magnetometer->printSensorDetails();
    }
    break;
  case DEV_BMI270_BMM150:
    bmi270_bmm150 = new BoschSensorClass();
#ifdef HAVE_WIRE1
    // bmi270_bmm150->debug(Serial);
    config.bmm150_avail =
        bmi270_bmm150->begin_bmm150(BMM150_DEFAULT_I2C_ADDRESS, &Wire1);
    if (config.bmm150_avail) {
      config.magnetometer_type = MAG_BMM150;
      config.magnetometer_name = "BMM150";
    }
    config.bmi270_avail =
        bmi270_bmm150->begin_bmi270(BMI2_I2C_PRIM_ADDR, &Wire1);
    if (config.bmi270_avail) {
      config.accel_type = ACCEL_BMI270;
      config.accel_name = "BMI270";
      config.gyro_type = GYRO_BMI270;
      config.gyro_name = "BMI270";
      break;
    }
#endif
    // bmi270_bmm150->debug(Serial);
    if (!config.bmm150_avail) {
      config.bmm150_avail =
          bmi270_bmm150->begin_bmm150(BMM150_DEFAULT_I2C_ADDRESS, &Wire);
      if (config.bmm150_avail) {
        config.magnetometer_type = MAG_BMM150;
        config.magnetometer_name = "BMM150";
      }
    }
    if (!config.bmi270_avail) {
      config.bmi270_avail =
          bmi270_bmm150->begin_bmi270(BMI2_I2C_PRIM_ADDR, &Wire);
      if (config.bmi270_avail) {
        config.accel_type = ACCEL_BMI270;
        config.accel_name = "BMI270";
        config.gyro_type = GYRO_BMI270;
        config.gyro_name = "BMI270";
      }
    }
    break;
  case DEV_FXOS8700_FXAS21002C:
#ifdef HAVE_WIRE1
    config.fxos8700_avail = fxos.begin(0x1F, &Wire1);
    if (!config.fxos8700_avail) {
      config.fxos8700_avail = fxos.begin(0x1F, &Wire);
    }
#else
    config.fxos8700_avail = fxos.begin(0x1F, &Wire);
#endif

    if (config.fxos8700_avail) {
      /* Set accelerometer range (optional, default is 2G) */
      // fxos.setAccelRange(ACCEL_RANGE_2G);

      /* Set the sensor mode (optional, default is hybrid mode) */
      // fxos.setSensorMode(ACCEL_ONLY_MODE);

      /* Set the magnetometer's oversampling ratio (optional, default is 7)
       */
      // fxos.setMagOversamplingRatio(MAG_OSR_7);

      /* Set the output data rate (optional, default is 100Hz) */
      // fxos.setOutputDataRate(ODR_100HZ);
    }
#ifdef HAVE_WIRE1

    config.fxas21002_avail = fxas.begin(0x21, &Wire1);
    if (!config.fxas21002_avail) {
      config.fxas21002_avail = fxas.begin(0x21, &Wire);
    }
#else
    config.fxas21002_avail = fxas.begin(0x21, &Wire);
#endif

    if (config.fxas21002_avail) {
      // FIXME sensor setup
      // Set gyro range.(optional,
      // default is 250 dps)
      // fxas.setRange(GYRO_RANGE_2000DPS);
    }
    // try these first as the NXP sensors are autodetected by M5.Imu.begin()
    // as type MPU9250 which is wrong...
    if (config.fxos8700_avail && config.fxas21002_avail) {
      config.accel_type = ACCEL_FXOS8700;
      config.accel_name = "FXOS8700";
      config.gyro_type = GYRO_FXAS21002C;
      config.gyro_name = "FXAS21002C";
      config.magnetometer_type = MAG_FXOS8700;
      config.magnetometer_name = "FXOS8700";

      accelerometer = fxos.getAccelerometerSensor();
      magnetometer = fxos.getMagnetometerSensor();
      gyroscope = &fxas;

      accelerometer->printSensorDetails();
      gyroscope->printSensorDetails();
      magnetometer->printSensorDetails();
    }
    break;
  case DEV_NONE:
  case DEV_ENDMARK:
    break;
  }
}
