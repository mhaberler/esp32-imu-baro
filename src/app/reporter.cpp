#include "TimerStats.h"
#include "../custom-example/custom.hpp"
#include "defs.hpp"
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

extern TripleBuffer<sensor_state_t> triple_buffer;
extern TimerStats customImuStats, imuStats, slowSensorStats;
extern Queue slowSensors;
extern Queue bleSensors;

TimerStats reporterStats, customReporterStats;

static uint32_t last_report_us;  // micros() of last report
static int64_t last_report_abs;  //  abs_timestamp(config) of last report

static UBX_NAV_PVT_data_t ub_nav_pvt;
static int64_t ub_nav_pvt_abs;

// DynamicJsonDocument doc(JSON_DOCUMENT_SIZE);
// static SpiRamJsonDocument(SPIRAM_JSON_DOCUMENT_SIZE) doc;

typedef StaticJsonDocument<JSON_DOCUMENT_SIZE> jdoc_t;
static jdoc_t doc;

static void emitJson(const config_t &config, options_t &opt, jdoc_t &doc);
static void emitStats(config_t &config, options_t &opt, jdoc_t &doc);

void ublox_nav_pvt(UBX_NAV_PVT_data_t *ub) {
    extern config_t config;
    ub_nav_pvt_abs = abs_timestamp(config);
    memcpy(&ub_nav_pvt, ub, sizeof(ub_nav_pvt));
}

void reporter(config_t &config, options_t &opt) {
#ifdef REPORT_PIN
    digitalWrite(REPORT_PIN, HIGH);
#endif

//   if (!triple_buffer.newSnap()) { // anything to do?
// #ifdef REPORT_PIN
//     digitalWrite(REPORT_PIN, LOW);
// #endif
//     return;
//   }
#ifdef MOTIONCAL

    if (motion_cal) {
        return;
    }
#endif
    bool newSnap             = triple_buffer.newSnap();
    const sensor_state_t &ss = triple_buffer.getReadRef();

    reporterStats.Start();

    int64_t now = millis() + config.millis_offset;
    // uint32_t now = micros();
    // uint32_t now_ms = millis();

    // for IMU and derived values, use timestamp recorded in sensor_state_t
    uint32_t its = ss.last_imu_update / 1000;

    if (newSnap && (ss.last_imu_update > last_report_us)) {
        if (options.teleplot_viewer) {
            if (options.report_raw) {
                // https://github.com/nesnes/teleplot/blob/main/clients/MPU6050_HID_tests/MPU6050_HID_tests.ino
#ifdef TELEPLOT
                teleplot.update_ms("a.x", its, ss.final_accel.acceleration.x,
                                   ms2, TELEPLOT_FLAG_NOPLOT);
                teleplot.update_ms("a.y", its, ss.final_accel.acceleration.y,
                                   ms2, TELEPLOT_FLAG_NOPLOT);
                teleplot.update_ms("a.z", its, ss.final_accel.acceleration.z,
                                   ms2, TELEPLOT_FLAG_NOPLOT);

                teleplot.update_ms("g.x", its, ss.final_gyro.gyro.x, dps,
                                   TELEPLOT_FLAG_NOPLOT);
                teleplot.update_ms("g.y", its, ss.final_gyro.gyro.y, dps,
                                   TELEPLOT_FLAG_NOPLOT);
                teleplot.update_ms("g.z", its, ss.final_gyro.gyro.z, dps,
                                   TELEPLOT_FLAG_NOPLOT);

                if (config.magnetometer_type != MAG_NONE) {
                    teleplot.update_ms("m.x", its, ss.final_mag.magnetic.x, uT,
                                       TELEPLOT_FLAG_NOPLOT);
                    teleplot.update_ms("m.y", its, ss.final_mag.magnetic.y, uT,
                                       TELEPLOT_FLAG_NOPLOT);
                    teleplot.update_ms("m.z", its, ss.final_mag.magnetic.z, uT,
                                       TELEPLOT_FLAG_NOPLOT);
                }
#endif
            }
            if (options.report_hpr) {
#ifdef TELEPLOT

                teleplot.update_ms("pose.roll", its, ss.roll, deg,
                                   TELEPLOT_FLAG_NOPLOT);
                teleplot.update_ms("pose.pitch", its, ss.pitch, deg,
                                   TELEPLOT_FLAG_NOPLOT);
                teleplot.update_ms("pose.yaw", its, ss.heading, deg,
                                   TELEPLOT_FLAG_NOPLOT);
                // 3D box

                teleplot.update3D(
                    ShapeTeleplot(options.selected_imu_name, "cube", "#ffff00")
                        .setPos(1, 1, 1)
                        .setCubeProperties(2.5, 1.5, 0.5)
                        .setRot(ss.qx, ss.qy, ss.qz, ss.qw),
                    TELEPLOT_FLAG_NOPLOT);
#endif
            }
            if (options.report_grav) {
#ifdef TELEPLOT

                teleplot.update_ms("grav.x", its, ss.gravx, deg,
                                   TELEPLOT_FLAG_NOPLOT);
                teleplot.update_ms("grav.y", its, ss.gravy, deg,
                                   TELEPLOT_FLAG_NOPLOT);
                teleplot.update_ms("grav.z", its, ss.gravz, deg,
                                   TELEPLOT_FLAG_NOPLOT);
#endif
            }
        }
        if (options.ndjson) {
            if (options.report_raw) {
                JsonObject a = doc.createNestedObject("accel");
                a["t"]       = now;
                a["x"]       = ss.final_accel.acceleration.x;
                a["y"]       = ss.final_accel.acceleration.y;
                a["z"]       = ss.final_accel.acceleration.z;
                emitJson(config, options, doc);

                JsonObject j = doc.createNestedObject("gyro");
                j["t"]       = now;
                j["x"]       = ss.final_gyro.gyro.x;
                j["y"]       = ss.final_gyro.gyro.y;
                j["z"]       = ss.final_gyro.gyro.z;
                emitJson(config, options, doc);

                if (config.magnetometer_type != MAG_NONE) {
                    JsonObject m = doc.createNestedObject("mag");
                    m["t"]       = now;
                    m["x"]       = ss.final_mag.magnetic.x;
                    m["y"]       = ss.final_mag.magnetic.y;
                    m["z"]       = ss.final_mag.magnetic.z;
                }
                emitJson(config, options, doc);
            }
            if (options.report_hpr) {
                JsonObject m = doc.createNestedObject("hpr");
                m["t"]       = now;
                m["heading"] = ss.heading;
                m["pitch"]   = ss.pitch;
                m["roll"]    = ss.roll;
                emitJson(config, options, doc);
            }
            if (options.report_quat) {
                JsonObject m = doc.createNestedObject("quat");
                m["t"]       = now;
                m["qw"]      = ss.qw;
                m["qx"]      = ss.qx;
                m["qy"]      = ss.qy;
                m["qz"]      = ss.qz;
                emitJson(config, options, doc);
            }
            if (options.report_grav) {
                JsonObject m = doc.createNestedObject("grav");
                m["t"]       = now;
                m["x"]       = ss.gravx;
                m["y"]       = ss.gravy;
                m["z"]       = ss.gravz;
                emitJson(config, options, doc);
            }
        }
    }

    if (config.flowsensor_avail) {
        flowsensor_report_t report;
        flow_sensor.getReport(report);

        static uint32_t track_flowcount = 0xFFFFFFFF;
        static bool track_flowing       = false;
        if ((track_flowcount != report.count) ||
            (track_flowing != report.flowing)) {
            if (options.teleplot_viewer) {
#ifdef TELEPLOT

                if (track_flowing != report.flowing) {
                    // insert fake report for nice edge
                    teleplot.update_ms("flowDetected", report.timestamp - 1,
                                       (int)(!report.flowing), bool_,
                                       TELEPLOT_FLAG_NOPLOT);
                }
                teleplot.update_ms("flowDetected", report.timestamp,
                                   (int)report.flowing, bool_,
                                   TELEPLOT_FLAG_NOPLOT);
                teleplot.update_ms("flowCounter", report.timestamp,
                                   report.count, counter_,
                                   TELEPLOT_FLAG_NOPLOT);
                teleplot.update_ms("flowBounces", report.timestamp,
                                   report.bounces, counter_,
                                   TELEPLOT_FLAG_NOPLOT);
#endif
            }
            if (options.ndjson) {
                JsonObject j = doc.createNestedObject("flow");
                j["t"]       = report.timestamp;
                j["count"]   = report.count;
                j["bounces"] = report.bounces;
                j["flowing"] = report.flowing;
                emitJson(config, options, doc);
            }
            track_flowcount = report.count;
            track_flowing   = report.flowing;
        }
    }

    if (config.quad_sensor_avail) {
        qsensor_report_t report;
        quad_sensor.getReport(report);
        static uint32_t track_flowcount = 0xFFFFFFFF;
        static bool track_flowing       = false;
        if ((track_flowcount != report.count) ||
            (track_flowing != report.changed)) {
#ifdef TELEPLOT

            if (options.teleplot_viewer) {
                if (track_flowing != report.changed) {
                    // insert fake report for nice edge
                    teleplot.update_ms(
                        "flowDetected", report.last_change / 1000 - 1,
                        (int)(!report.changed), bool_, TELEPLOT_FLAG_NOPLOT);
                }
                teleplot.update_ms("flowDetected", report.last_change / 1000,
                                   (int)report.changed, bool_,
                                   TELEPLOT_FLAG_NOPLOT);
                teleplot.update_ms("flowCounter", report.last_change / 1000,
                                   report.count, counter_,
                                   TELEPLOT_FLAG_NOPLOT);
            }
#endif
            if (options.ndjson) {
                JsonObject j = doc.createNestedObject("flow");
                j["t"]       = report.last_change / 1000;
                j["count"]   = report.count;
                j["changed"] = report.changed;
                emitJson(config, options, doc);
            }
            track_flowcount = report.count;
            track_flowing   = report.changed;
        }
    }

    if (options.report_baro) {
        slowSensorReport_t bp;
        while (slowSensors.Dequeue(&bp, 0)) {
            std::string dev = sensor_types[bp.typus];
            if (options.teleplot_viewer) {
                switch (bp.typus) {
                    case TYPE_LPS22:
                    case TYPE_DPS3XX:
                    case TYPE_BMP3XX:

#ifdef TELEPLOT

                        teleplot.update_ms(dev + ".hpa", bp.timestamp,
                                           bp.baro.hpa, hpascal,
                                           TELEPLOT_FLAG_NOPLOT);
                        teleplot.update_ms(dev + ".alt", bp.timestamp,
                                           bp.baro.alt, meter,
                                           TELEPLOT_FLAG_NOPLOT);
#endif
                        break;
                    case TYPE_INA219:
#ifdef TELEPLOT

                        teleplot.update_ms(dev + ".current", bp.timestamp,
                                           bp.ina219.current_mA, mA,
                                           TELEPLOT_FLAG_NOPLOT);
#endif
                        break;
                    case TYPE_INA226:
#ifdef TELEPLOT

                        teleplot.update_ms(dev + ".current", bp.timestamp,
                                           bp.ina226.current_mA, mA,
                                           TELEPLOT_FLAG_NOPLOT);
#endif
                        break;
                    case TYPE_TMP117:
#ifdef TELEPLOT

                        teleplot.update_ms(dev + ".temperature", bp.timestamp,
                                           bp.tmp117.temperature, deg,
                                           TELEPLOT_FLAG_NOPLOT);
#endif
                        break;
                    case TYPE_TSL2591:
#ifdef TELEPLOT

                        teleplot.update_ms(dev + ".infrared", bp.timestamp,
                                           bp.tsl2591.ir, deg,
                                           TELEPLOT_FLAG_NOPLOT);
                        teleplot.update_ms(dev + ".full", bp.timestamp,
                                           bp.tsl2591.full, deg,
                                           TELEPLOT_FLAG_NOPLOT);
#endif
                    default:
                        break;
                }

                if (options.ndjson) {
                    JsonObject j = doc.createNestedObject(dev);
                    j["t"]       = bp.timestamp;

                    switch (bp.typus) {
                        case TYPE_LPS22:
                        case TYPE_DPS3XX:
                        case TYPE_BMP3XX:
                            j["hpa"] = bp.baro.hpa;
                            j["alt"] = bp.baro.alt;
                            break;
                        case TYPE_INA219:
                            j["shuntvoltage"] = bp.ina219.shuntvoltage;
                            j["busvoltage"]   = bp.ina219.busvoltage;
                            j["current_mA"]   = bp.ina219.current_mA;
                            j["power_mW"]     = bp.ina219.power_mW;
                            break;
                        case TYPE_INA226:
                            j["shuntvoltage"] = bp.ina226.shuntvoltage;
                            j["busvoltage"]   = bp.ina226.busvoltage;
                            j["current_mA"]   = bp.ina226.current_mA;
                            j["power_mW"]     = bp.ina226.power_mW;
                            break;
                        case TYPE_TMP117:
                            j["temperature"] = bp.tmp117.temperature;
                            break;
                        case TYPE_TSL2591:
                            j["infrared"] = bp.tsl2591.ir;
                            j["full"]     = bp.tsl2591.full;
                            break;
                        default:
                            break;
                    }
                    emitJson(config, options, doc);
                }
            }
            // blesensor_report_t ble;
            // while (bleSensors.Dequeue(&ble, 0)) {
            //   if (opt.trace & INFO_BLE_DEQUEUED) {
            //     print_ble_report(ble);
            //   }
            // }
#ifdef SERIAL_GPS
            if (true) {
                if (GPSFIX3D(serialGps) && serialGps.altitude.isUpdated()) {
                    if (options.teleplot_viewer) {
#ifdef TELEPLOT

                        teleplot.update_ms("serialGps.alt", its,
                                           serialGps.altitude.meters(), meter,
                                           TELEPLOT_FLAG_NOPLOT);
#endif
                    }
                    if (options.ndjson) {
                        JsonObject j = doc.createNestedObject("serialGps");
                        j["t"]       = its;  // FIXME wrong
                        j["alt"]     = serialGps.altitude.meters();
                        emitJson(config, options, doc);
                    }
                }
            }
#endif
#ifdef UBLOX_GPS
            if (ub_nav_pvt_abs > last_report_abs) {
                if (options.teleplot_viewer) {
                    teleplot.update_ms("ubloxGps.alt", millis(),
                                       ub_nav_pvt.hMSL / 1000.0, meter,
                                       TELEPLOT_FLAG_NOPLOT);
                }
                if (options.ndjson) {
                    JsonObject j = doc.createNestedObject("ubloxGps");
                    j["t"]       = ub_nav_pvt_abs;
                    j["fix"]     = ub_nav_pvt.fixType;
                    switch (ub_nav_pvt.fixType) {
                        case 4:
                        case 3:
                            j["hMSL"]    = ub_nav_pvt.hMSL / 1000.0;
                            j["hAE"]     = ub_nav_pvt.height / 1000.0;
                            j["velD"]    = ub_nav_pvt.height / 1000.0;
                            j["gSpeed"]  = ub_nav_pvt.gSpeed / 1000.0;
                            j["headMot"] = ub_nav_pvt.headMot * 1e-5;
                            j["pDOP"]    = ub_nav_pvt.pDOP * 0.01;
                        case 2:
                            j["lat"] = ub_nav_pvt.lat * 1e-7;
                            j["lon"] = ub_nav_pvt.lon * 1e-7;
                            break;
                    }
                    emitJson(config, options, doc);
                }
            }
#endif
            customReporterStats.Start();
            customReportingRateCode(ss, options);
            customReporterStats.Stop();

            if (run_stats) {
                emitStats(config, options, doc);
                run_stats = false;
            }

            last_report_us  = micros();
            last_report_abs = abs_timestamp(config);

            reporterStats.Stop();

#ifdef REPORT_PIN
            digitalWrite(REPORT_PIN, LOW);
#endif
        }
    }
}
#ifdef TIMING_STATS
#endif
#ifdef MEMORY_STATS
#endif

static void emitJson(const config_t &config, options_t &opt, jdoc_t &doc) {
    if (!doc.isNull()) {
        if (!options.ndjson) {
            if (config.log_open && bufferedLogger) {
                serializeJsonPretty(doc, *bufferedLogger);
                bufferedLogger->print("\n");
            }
            serializeJsonPretty(doc, Console);
            Console.print("\n");

        } else {
            if (config.log_open&& bufferedLogger) {
                serializeJson(doc, *bufferedLogger);
                bufferedLogger->print("\n");
            }
            serializeJson(doc, Console);
            Console.print("\n");
        }
        doc.clear();
    }
}

static void emitStats(config_t &config, options_t &opt, jdoc_t &doc) {
    if (opt.timing_stats) {
        if (opt.teleplot_viewer) {
#ifdef TELEPLOT

            teleplot.update_ms("handleImu.mean", millis(), imuStats.Mean(), uS,
                               TELEPLOT_FLAG_NOPLOT);
            teleplot.update_ms("reporter.mean", millis(), reporterStats.Mean(),
                               uS, TELEPLOT_FLAG_NOPLOT);
            teleplot.update_ms("customImu.mean", millis(),
                               customImuStats.Mean(), uS, TELEPLOT_FLAG_NOPLOT);
            teleplot.update_ms("customReporter.mean", millis(),
                               customReporterStats.Mean(), uS,
                               TELEPLOT_FLAG_NOPLOT);
            teleplot.update_ms("slowSensor.mean", millis(),
                               slowSensorStats.Mean(), uS,
                               TELEPLOT_FLAG_NOPLOT);
#endif
        }
        if (options.ndjson) {
            JsonObject m             = doc.createNestedObject("timing");
            m["t"]                   = millis();
            m["handleImu.mean"]      = imuStats.Mean();
            m["reporter.mean"]       = reporterStats.Mean();
            m["customImu.mean"]      = customImuStats.Mean();
            m["customReporter.mean"] = customReporterStats.Mean();
            m["slowSensor.mean"]     = slowSensorStats.Mean();
            emitJson(config, options, doc);
        }
    }

    if (opt.memory_usage) {
        config.reporter_stack_util =
            (int)(100.0 *
                  (opt.reporter_stack -
                   uxTaskGetStackHighWaterMark(reporterTask->GetHandle())) /
                  opt.reporter_stack);
        if (sensorTask) {
            config.sensor_stack_util =
                (int)(100.0 *
                      (opt.sensor_stack -
                       uxTaskGetStackHighWaterMark(sensorTask->GetHandle())) /
                      opt.sensor_stack);
        }

        uint32_t freeHeap = ESP.getFreeHeap();

        if (opt.teleplot_viewer) {
#ifdef TELEPLOT

            teleplot.update_ms("reporterStack", millis(),
                               config.reporter_stack_util, percent_,
                               TELEPLOT_FLAG_NOPLOT);
            teleplot.update_ms("sensorStack", millis(),
                               config.sensor_stack_util, percent_,
                               TELEPLOT_FLAG_NOPLOT);
            teleplot.update_ms("freeHeap", millis(), freeHeap, bytes_,
                               TELEPLOT_FLAG_NOPLOT);
#endif
        }
        if (options.ndjson) {
            JsonObject m       = doc.createNestedObject("memory");
            m["t"]             = millis();
            m["reporterStack"] = config.reporter_stack_util;
            m["sensorStack"]   = config.sensor_stack_util;
            m["freeHeap"]      = freeHeap;
            emitJson(config, options, doc);
        }
    }
}
