#include "FlowSensor.hpp"
#include "TimerStats.h"
#include "custom.hpp"
#include "defs.hpp"

static sensor_state_t initTB = {
    .final_accel = {.type = SENSOR_TYPE_ACCELEROMETER},
    .final_gyro  = {.type = SENSOR_TYPE_GYROSCOPE},
    .final_mag   = {.type = SENSOR_TYPE_MAGNETIC_FIELD}};

TripleBuffer<sensor_state_t> triple_buffer(initTB);
slowSensorReport_t selected_baro;

TimerStats imuStats, customImuStats, slowSensorStats;
Queue slowSensors(100, sizeof(slowSensorReport_t));
extern FlowSensor flow_sensor;
static int8_t sequence = -1;
extern options_t options;
extern config_t config;

void initGyroCalibration(float gx, float gy, float gz, config_t &config) {
    // prime the min/max ranges from last gyro reading
    // calibration requires rad/s
    config.max_x = config.min_x = gx / SENSORS_RADS_TO_DPS;
    config.max_y = config.min_y = gy / SENSORS_RADS_TO_DPS;
    config.max_z = config.min_z = gz / SENSORS_RADS_TO_DPS;
}

int64_t abs_timestamp(const config_t &config) {
    return config.millis_offset + millis();
}

void handleSensors(void) {
#ifdef IMU_PIN
    digitalWrite(IMU_PIN, HIGH);
#endif
    sensor_state_t &state = triple_buffer.getWriteRef();

    imuStats.Start();

    uint32_t start     = micros();
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
            if (config.dev[I2C_BMI270]) {
                if (bmi270_bmm150->accelerationAvailable()) {
                    bmi270_bmm150->readAcceleration(state.accel.acceleration.x,
                                                    state.accel.acceleration.y,
                                                    state.accel.acceleration.z);
                    state.accel.type = SENSOR_TYPE_ACCELEROMETER;
                }
            }
            break;
        case ACCEL_MPU9250_DMP:
            if (mpu9250_ready && (mpu9250_dmp->updateAccel() == INV_SUCCESS)) {
                state.accel.acceleration.x =
                    mpu9250_dmp->calcAccel(mpu9250_dmp->ax);
                state.accel.acceleration.y =
                    mpu9250_dmp->calcAccel(mpu9250_dmp->ay);
                state.accel.acceleration.z =
                    mpu9250_dmp->calcAccel(mpu9250_dmp->az);
                state.accel.type = SENSOR_TYPE_ACCELEROMETER;
            }
            break;

#ifdef M5UNIFIED
        case ACCEL_M5IMU:

            M5.Imu.getAccel(&state.accel.acceleration.x,
                            &state.accel.acceleration.y,
                            &state.accel.acceleration.z);
            state.accel.type = SENSOR_TYPE_ACCELEROMETER;
            break;
#endif

        case ACCEL_MPU6886:
            break;

        default:
            break;
    }
    switch (config.gyro_type) {
        case GYRO_NONE:
            break;

        case GYRO_BMI270:
            if (config.dev[I2C_BMI270]) {
                if (bmi270_bmm150->gyroscopeAvailable()) {
                    // already in DPS
                    bmi270_bmm150->readGyroscope(state.gyro.gyro.x,
                                                 state.gyro.gyro.y,
                                                 state.gyro.gyro.z);
                    state.gyro.type = SENSOR_TYPE_GYROSCOPE;
                }
            }
            break;

        case GYRO_MPU9250_DMP:
            if (mpu9250_ready && (mpu9250_dmp->updateGyro() == INV_SUCCESS)) {
                // already in DPS
                state.gyro.gyro.x = mpu9250_dmp->calcGyro(mpu9250_dmp->gx);
                state.gyro.gyro.y = mpu9250_dmp->calcGyro(mpu9250_dmp->gy);
                state.gyro.gyro.z = mpu9250_dmp->calcGyro(mpu9250_dmp->gz);
                state.gyro.type   = SENSOR_TYPE_GYROSCOPE;
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

#ifdef M5UNIFIED
        case GYRO_M5IMU:

            // already in DPS
            M5.Imu.getGyro(&state.gyro.gyro.x, &state.gyro.gyro.y,
                           &state.gyro.gyro.z);
            break;
#endif

        default:
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
            if (mpu9250_ready &&
                (mpu9250_dmp->updateCompass() == INV_SUCCESS)) {
                // already in uT
                state.mag.magnetic.x = mpu9250_dmp->calcMag(mpu9250_dmp->gx);
                state.mag.magnetic.y = mpu9250_dmp->calcMag(mpu9250_dmp->gy);
                state.mag.magnetic.z = mpu9250_dmp->calcMag(mpu9250_dmp->gz);
                state.mag.type       = SENSOR_TYPE_MAGNETIC_FIELD;
            }
            break;
        case MAG_BMM150:
            if (config.dev[I2C_BMM150]) {
                if (bmi270_bmm150->magneticFieldAvailable()) {
                    // already uTesla
                    bmi270_bmm150->readMagneticField(state.mag.magnetic.x,
                                                     state.mag.magnetic.y,
                                                     state.mag.magnetic.z);
                    state.mag.type = SENSOR_TYPE_MAGNETIC_FIELD;
                }
            }
            break;

        case MAG_M5IMU:
#ifdef M5UNIFIED
            // already uTesla
            M5.Imu.getMag(&state.mag.magnetic.x, &state.mag.magnetic.y,
                          &state.mag.magnetic.z);
            state.mag.type = SENSOR_TYPE_MAGNETIC_FIELD;
#endif
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
        config.gcal_samples = -1;  // once only

        LOGD("gyro: zero rate offset: ({:.4f}, {:.4f}, {:.4f}) °/s",
             config.mid_x, config.mid_y, config.mid_z);
        LOGD("gyro: rad/s noise: ({:.4f}, {:.4f}, {:.4f})",
             config.max_x - config.min_x, config.max_y - config.min_y,
             config.max_z - config.min_z);

        config.cal.gyro_zerorate[0] = config.mid_x;
        config.cal.gyro_zerorate[1] = config.mid_y;
        config.cal.gyro_zerorate[2] = config.mid_z;
        if (!config.cal.saveCalibration()) {
            LOGD("**WARNING** Couldn't save calibration");
        } else {
            config.cal_loaded = true;
            LOGD("Wrote gyro calibration");
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
    if (options.apply_cal && config.cal_loaded) {
        config.cal.calibrate(state.accel);
        config.cal.calibrate(state.gyro);
        config.cal.calibrate(state.mag);
    };

    // all result values are in final_*
    state.final_accel = state.accel;
    state.final_gyro  = state.gyro;
    state.final_mag   = state.mag;
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
    if (options.run_filter && (config.filter != NULL)) {
        // AHRS step
        if (config.magnetometer_type != MAG_NONE) {
            config.filter->update(
                state.final_gyro.gyro.x, state.final_gyro.gyro.y,
                state.final_gyro.gyro.z, state.final_accel.acceleration.x,
                state.final_accel.acceleration.y,
                state.final_accel.acceleration.z, state.final_mag.magnetic.x,
                state.final_mag.magnetic.y, state.final_mag.magnetic.z);
        } else {
            config.filter->updateIMU(
                state.final_gyro.gyro.x, state.final_gyro.gyro.y,
                state.final_gyro.gyro.z, state.final_accel.acceleration.x,
                state.final_accel.acceleration.y,
                state.final_accel.acceleration.z);
        }
    }
    // #ifdef IMU_PIN
    //   TOGGLE(IMU_PIN);
    // #endif
    if (config.filter != NULL) {
        state.roll    = config.filter->getRoll();
        state.pitch   = config.filter->getPitch();
        state.heading = config.filter->getYaw();
        config.filter->getQuaternion(&state.qw, &state.qx, &state.qy,
                                     &state.qz);

        config.filter->getGravityVector(&state.gravx, &state.gravy,
                                        &state.gravz);
    }

    state.last_imu_update = micros();
    state.runtime         = state.last_imu_update - start;

    slowSensorStats.Start();

#ifdef EXTRA_PIN
    TOGGLE(EXTRA_PIN);
#endif
    if (run_sensors) {
        sequence++;
        switch (sequence) {
            case 0:
                if (config.dev[I2C_BMP390]) {
                    if (bmp->performReading()) {
                        slowSensorReport_t slow;
                        slow.typus     = TYPE_BMP3XX;
                        slow.timestamp = abs_timestamp(config);
                        slow.baro.hpa  = bmp->pressure / 100.0;
                        slow.baro.alt = bmp->readAltitude(SEALEVELPRESSURE_HPA);

                        slowSensors.Enqueue(&slow);
                        if (options.which_baro == I2C_BMP390) {
                            memcpy(&selected_baro, &slow,
                                   sizeof(selected_baro));
                        }
                    }
                }
                break;

            case 1:
                if (config.dev[I2C_LPS22]) {
                    sensors_event_t event;
                    slowSensorReport_t slow;

                    lps2x_pressure->getEvent(&event);
                    slow.typus     = TYPE_LPS22;
                    slow.timestamp = abs_timestamp(config);
                    slow.baro.hpa  = event.pressure;
                    slow.baro.alt =
                        hPa2meters(event.pressure, SEALEVELPRESSURE_HPA);

                    slowSensors.Enqueue(&slow);
                    if (options.which_baro == I2C_LPS22) {
                        memcpy(&selected_baro, &slow, sizeof(selected_baro));
                    }
                }
                break;

            case 2:
                if (config.dev[I2C_DPS3XX]) {
                    sensors_event_t event;
                    slowSensorReport_t slow;

                    dps3xx_pressure->getEvent(&event);
                    slow.typus     = TYPE_DPS3XX;
                    slow.timestamp = abs_timestamp(config);
                    slow.baro.hpa  = event.pressure;
                    slow.baro.alt =
                        hPa2meters(event.pressure, SEALEVELPRESSURE_HPA);

                    slowSensors.Enqueue(&slow);
                    if (options.which_baro == I2C_DPS3XX) {
                        memcpy(&selected_baro, &slow, sizeof(selected_baro));
                    }
                }
                break;

            case 3:
                if (config.dev[I2C_INA219]) {
                    slowSensorReport_t slow;

                    slow.ina219.busvoltage   = ina219->getBusVoltage_V();
                    slow.ina219.shuntvoltage = ina219->getShuntVoltage_mV();
                    slow.ina219.current_mA   = ina219->getCurrent_mA();
                    slow.ina219.loadvoltage  = ina219->getBusVoltage_V();
                    slow.ina219.power_mW     = ina219->getPower_mW();
                    slow.typus               = TYPE_INA219;
                    slow.timestamp           = abs_timestamp(config);

                    slowSensors.Enqueue(&slow);
                }
                break;

            case 4:
                if (config.dev[I2C_TMP117]) {
                    sensors_event_t event;
                    slowSensorReport_t slow;

                    tmp117->getEvent(&event);
                    slow.typus              = TYPE_TMP117;
                    slow.timestamp          = abs_timestamp(config);
                    slow.tmp117.temperature = event.temperature;
                    slowSensors.Enqueue(&slow);
                }
                break;

            case 5:
                if (config.dev[I2C_UBLOXI2C]) {
#ifdef UBLOX_GPS

#ifdef UBLOX_PIN
                    TOGGLE(UBLOX_PIN);
#endif
                    ublox_loopcheck();
#ifdef UBLOX_PIN
                    TOGGLE(UBLOX_PIN);
#endif
#endif
                }
                break;
            default:
                run_sensors = false;
                sequence    = -1;  // reset FSM
                break;
        }
    }
#ifdef EXTRA_PIN
    TOGGLE(EXTRA_PIN);
#endif

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
