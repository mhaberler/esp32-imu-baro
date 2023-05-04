
#include "FlowSensor.hpp"
#include "TimerStats.h"
#include "custom.hpp"
#include "defs.hpp"

static sensor_state_t initTB = {
    .final_accel = {.type = SENSOR_TYPE_ACCELEROMETER},
    .final_gyro = {.type = SENSOR_TYPE_GYROSCOPE},
    .final_mag = {.type = SENSOR_TYPE_MAGNETIC_FIELD}};
TripleBuffer<sensor_state_t> triple_buffer(initTB);

TimerStats imuStats, customImuStats, slowSensorStats;
static bool run_lowrate_sensors;
static Ticker sensor_ticker;
static bool run_sensors;

void initGyroCalibration(float gx, float gy, float gz, config_t &config) {
  // prime the min/max ranges from last gyro reading
  // calibration requires rad/s
  config.max_x = config.min_x = gx / SENSORS_RADS_TO_DPS;
  config.max_y = config.min_y = gy / SENSORS_RADS_TO_DPS;
  config.max_z = config.min_z = gz / SENSORS_RADS_TO_DPS;
}

#ifdef FLOWSENSOR_PIN
extern FlowSensor flow_sensor;
#endif

void handleSensors(config_t &config, const options_t &options) {
#ifdef IMU_PIN
  digitalWrite(IMU_PIN, HIGH);
#endif
  sensor_state_t &state = triple_buffer.getWriteRef();

  imuStats.Start();

  uint32_t start = micros();
  bool mpu9250_ready = false;

  switch (config.accel_type) {
  case ACCEL_MPU9250_DMP:
    mpu9250_ready = mpu9250_dmp->dataReady();
    break;
  case ACCEL_MPU6886:
    mpu6886->getEvent(&state.accel, &state.gyro, &state.temp);
    break;
  default:;
  }

  switch (config.accel_type) {
  case ACCEL_NONE:
    break;
  case ACCEL_FXOS8700:
  case ACCEL_ICM20948:
  case ACCEL_MPU6050:
    accelerometer->getEvent(&state.accel);
    break;
  case ACCEL_BMI270:
    if (bmi270_bmm150->accelerationAvailable()) {
      bmi270_bmm150->readAcceleration(state.accel.acceleration.x,
                                      state.accel.acceleration.y,
                                      state.accel.acceleration.z);
      state.accel.type = SENSOR_TYPE_ACCELEROMETER;
    }
    break;
  case ACCEL_MPU9250_DMP:
    if (mpu9250_ready && (mpu9250_dmp->updateAccel() == INV_SUCCESS)) {
      state.accel.acceleration.x = mpu9250_dmp->calcAccel(mpu9250_dmp->ax);
      state.accel.acceleration.y = mpu9250_dmp->calcAccel(mpu9250_dmp->ay);
      state.accel.acceleration.z = mpu9250_dmp->calcAccel(mpu9250_dmp->az);
      state.accel.type = SENSOR_TYPE_ACCELEROMETER;
    }
    break;
  case ACCEL_MPU6886:
    break;

  default:
    break;
  }
  switch (config.gyro_type) {
  case GYRO_NONE:
    break;

  case GYRO_BMI270:
    if (bmi270_bmm150->gyroscopeAvailable()) {
      // already in DPS
      bmi270_bmm150->readGyroscope(state.gyro.gyro.x, state.gyro.gyro.y,
                                   state.gyro.gyro.z);
      state.gyro.type = SENSOR_TYPE_GYROSCOPE;
    }
    break;

  case GYRO_MPU9250_DMP:
    if (mpu9250_ready && (mpu9250_dmp->updateGyro() == INV_SUCCESS)) {
      // already in DPS
      state.gyro.gyro.x = mpu9250_dmp->calcGyro(mpu9250_dmp->gx);
      state.gyro.gyro.y = mpu9250_dmp->calcGyro(mpu9250_dmp->gy);
      state.gyro.gyro.z = mpu9250_dmp->calcGyro(mpu9250_dmp->gz);
      state.gyro.type = SENSOR_TYPE_GYROSCOPE;
    }
    break;

  case GYRO_FXAS21002C:
  case GYRO_ICM20948:
  case GYRO_MPU6050:
  case GYRO_MPU6886:
    // Adafruit Unified Sensors report rad/s
    gyroscope->getEvent(&state.gyro);
    // Gyroscope needs to be converted from Rad/s to Degree/s
    // the rest are not unit-important
    state.gyro.gyro.x *= SENSORS_RADS_TO_DPS;
    state.gyro.gyro.y *= SENSORS_RADS_TO_DPS;
    state.gyro.gyro.z *= SENSORS_RADS_TO_DPS;
    break;
  }

  switch (config.magnetometer_type) {
  case MAG_NONE:
    break;

  case MAG_FXOS8700:
  case MAG_ICM20948:
    magnetometer->getEvent(&state.mag);
    break;
  case MAG_MPU9250_DMP:
    if (mpu9250_ready && (mpu9250_dmp->updateCompass() == INV_SUCCESS)) {
      // already in uT
      state.mag.magnetic.x = mpu9250_dmp->calcMag(mpu9250_dmp->gx);
      state.mag.magnetic.y = mpu9250_dmp->calcMag(mpu9250_dmp->gy);
      state.mag.magnetic.z = mpu9250_dmp->calcMag(mpu9250_dmp->gz);
      state.mag.type = SENSOR_TYPE_MAGNETIC_FIELD;
    }
    break;
  case MAG_BMM150:
    if (bmi270_bmm150->magneticFieldAvailable()) {
      // already uTesla
      bmi270_bmm150->readMagneticField(
          state.mag.magnetic.x, state.mag.magnetic.y, state.mag.magnetic.z);
      state.mag.type = SENSOR_TYPE_MAGNETIC_FIELD;
    }
    break;
  }

  // #ifdef IMU_PIN
  //   TOGGLE(IMU_PIN);
  // #endif

  if (motion_cal) {
    MotionCal(state, options, config);
    return;
  }

  if (config.gcal_samples == 0) {
    // true once GYRO_SAMPLES have been taken
    // so we're done
    config.gcal_samples = -1; // once only

    Console.fmt("gyro: zero rate offset: ({:.4f}, {:.4f}, {:.4f}) °/s\n",
                config.mid_x, config.mid_y, config.mid_z);
    Console.fmt("gyro: rad/s noise: ({:.4f}, {:.4f}, {:.4f})\n",
                config.max_x - config.min_x, config.max_y - config.min_y,
                config.max_z - config.min_z);

    config.cal.gyro_zerorate[0] = config.mid_x;
    config.cal.gyro_zerorate[1] = config.mid_y;
    config.cal.gyro_zerorate[2] = config.mid_z;
    if (!config.cal.saveCalibration()) {
      Console.fmt("**WARNING** Couldn't save calibration\n");
    } else {
      Console.fmt("Wrote gyro calibration\n");
    }
  }
  if (config.gcal_samples > 0) {

    float x, y, z;

    // calibration requires rad/s
    x = state.gyro.gyro.x / SENSORS_RADS_TO_DPS;
    y = state.gyro.gyro.y / SENSORS_RADS_TO_DPS;
    z = state.gyro.gyro.z / SENSORS_RADS_TO_DPS;

    config.min_x = min(config.min_x, x);
    config.min_y = min(config.min_y, y);
    config.min_z = min(config.min_z, z);

    config.max_x = max(config.max_x, x);
    config.max_y = max(config.max_y, y);
    config.max_z = max(config.max_z, z);

    config.mid_x = (config.max_x + config.min_x) / 2;
    config.mid_y = (config.max_y + config.min_y) / 2;
    config.mid_z = (config.max_z + config.min_z) / 2;

    config.gcal_samples -= 1;
  }

  // apply calibration before NED step:
  if (options.apply_cal) {
    config.cal.calibrate(state.accel);
    config.cal.calibrate(state.gyro);
    config.cal.calibrate(state.mag);
  };

  // all result values are in final_*
  state.final_accel = state.accel;
  state.final_gyro = state.gyro;
  state.final_mag = state.mag;
  // see
  // https://github.com/har-in-air/ESP32_IMU_BARO_GPS_VARIO/blob/dca3b7298c764fcdb2100c89898cae44a715a2eb/src/main.cpp#L312-L314
  // translate from sensor axes to AHRS NED (north-east-down) right handed
  // coordinate frame
  if (options.ned) {
    state.final_accel.acceleration.x = -state.accel.acceleration.y;
    state.final_accel.acceleration.y = -state.accel.acceleration.x;
    state.final_accel.acceleration.z = state.accel.acceleration.z;

    state.final_gyro.gyro.x = state.gyro.gyro.y;
    state.final_gyro.gyro.y = state.gyro.gyro.x;
    state.final_gyro.gyro.z = -state.gyro.gyro.z;

    state.final_mag = state.mag;
  }
  if (options.run_filter) {
    // AHRS step
    if (config.magnetometer_type != MAG_NONE) {
      config.filter->update(
          state.final_gyro.gyro.x, state.final_gyro.gyro.y,
          state.final_gyro.gyro.z, state.final_accel.acceleration.x,
          state.final_accel.acceleration.y, state.final_accel.acceleration.z,
          state.final_mag.magnetic.x, state.final_mag.magnetic.y,
          state.final_mag.magnetic.z);
    } else {
      config.filter->updateIMU(
          state.final_gyro.gyro.x, state.final_gyro.gyro.y,
          state.final_gyro.gyro.z, state.final_accel.acceleration.x,
          state.final_accel.acceleration.y, state.final_accel.acceleration.z);
    }
  }
  // #ifdef IMU_PIN
  //   TOGGLE(IMU_PIN);
  // #endif
  state.roll = config.filter->getRoll();
  state.pitch = config.filter->getPitch();
  state.heading = config.filter->getYaw();
  config.filter->getQuaternion(&state.qw, &state.qx, &state.qy, &state.qz);

  config.filter->getGravityVector(&state.gravx, &state.gravy, &state.gravz);

  state.last_imu_update = micros();
  state.runtime = state.last_imu_update - start;

  slowSensorStats.Start();

  if (run_lowrate_sensors) {
#ifdef EXTRA_PIN
    TOGGLE(EXTRA_PIN);
#endif

    if (config.bmp390_avail) {
      if (bmp.performReading()) {
        baro_report_t &bp = state.baro_values[USE_BARO_BMP3XX];
        bp.hpa = bmp.pressure / 100.0;
        bp.alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
        bp.timestamp = millis();
      }
    }
    if (config.lps22_avail) {
      sensors_event_t event;
      lps2x_pressure->getEvent(&event);
      baro_report_t &bp = state.baro_values[USE_BARO_LPS22];
      bp.hpa = event.pressure;
      bp.alt = hPa2meters(bp.hpa, SEALEVELPRESSURE_HPA);
      bp.timestamp = event.timestamp;
    }
    if (config.dps3xx_avail) {
      sensors_event_t event;
      dps3xx_pressure->getEvent(&event);
      baro_report_t &bp = state.baro_values[USE_BARO_DPS3XX];
      bp.hpa = event.pressure;
      bp.alt = hPa2meters(bp.hpa, SEALEVELPRESSURE_HPA);
      bp.timestamp = event.timestamp;
    }
#ifdef FLOWSENSOR_PIN
    if (config.flowsensor_avail) {
      flow_sensor.getReport(state.flowsensor_values);
    }
#endif

#ifdef EXTRA_PIN
    TOGGLE(EXTRA_PIN);
#endif
    run_lowrate_sensors = false;
  }
  slowSensorStats.Stop();

  customImuStats.Start();
  customIMUrateCode(state, options);
  customImuStats.Stop();

  triple_buffer.flipWriter();

  imuStats.Stop();

#ifdef IMU_PIN
  digitalWrite(IMU_PIN, LOW);
#endif
}

#ifdef MULTICORE

void sensorTask(void *p) {
  while (true) {
    if (run_sensors) {
      handleSensors(config, options);
      run_sensors = false;
    }
    check_serial_gps();
    watchDogRefresh();
    delay(1); // FIXME
  }
}

void triggerSlowSensorUpdate(void) { run_lowrate_sensors = true; }

void setSensorRate(const float hz) { setRate(sensor_ticker, hz, &run_sensors); }

TaskHandle_t sensorTaskHandle = NULL;
const char *sensorTaskName = "sensorTask";

bool initSensorTask(void) {
  xTaskCreateUniversal(sensorTask, sensorTaskName, SENSORTASK_STACKSIZE, NULL,
                       SENSORTASK_PRIORITY, &sensorTaskHandle, SENSORTASK_CORE);
  return sensorTaskHandle != NULL;
}
#endif