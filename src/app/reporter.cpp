#include "TimerStats.h"
#include "custom.hpp"
#include "defs.hpp"
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

extern TripleBuffer<sensor_state_t> triple_buffer;
extern TimerStats customImuStats, imuStats, slowSensorStats;

TimerStats reporterStats, customReporterStats;

static uint32_t last_report_us; // micros() of last report
static uint32_t last_report_ms; // millis() of last report

static UBX_NAV_PVT_data_t ub_nav_pvt;
static uint32_t ub_nav_pvt_ms;

void ublox_nav_pvt(UBX_NAV_PVT_data_t *ub) {
  ub_nav_pvt_ms = millis();
  memcpy(&ub_nav_pvt, ub, sizeof(ub_nav_pvt));
}

static void emitStats(config_t &config, options_t &opt,
                      DynamicJsonDocument &doc);

void reporter(config_t &config, options_t &opt) {
#ifdef REPORT_PIN
  digitalWrite(REPORT_PIN, HIGH);
#endif

  if (!triple_buffer.newSnap()) { // anything to do?
#ifdef REPORT_PIN
    digitalWrite(REPORT_PIN, LOW);
#endif
    return;
  }
  if (motion_cal) {
    return;
  }

  const sensor_state_t &ss = triple_buffer.getReadRef();

  DynamicJsonDocument doc(JSON_SIZE);

  reporterStats.Start();

  uint32_t now = micros();
  uint32_t now_ms = millis();

  // for IMU and derived values, use timestamp recorded in sensor_state_t
  uint32_t its = ss.last_imu_update / 1000;

  if (ss.last_imu_update > last_report_us) {
    if (options.teleplot_viewer) {
      if (options.report_raw) {
        // https://github.com/nesnes/teleplot/blob/main/clients/MPU6050_HID_tests/MPU6050_HID_tests.ino

        teleplot.update_ms("a.x", its, ss.final_accel.acceleration.x, ms2,
                           TELEPLOT_FLAG_NOPLOT);
        teleplot.update_ms("a.y", its, ss.final_accel.acceleration.y, ms2,
                           TELEPLOT_FLAG_NOPLOT);
        teleplot.update_ms("a.z", its, ss.final_accel.acceleration.z, ms2,
                           TELEPLOT_FLAG_NOPLOT);

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
      }
      if (options.report_hpr) {
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
      }
      if (options.report_grav) {
        teleplot.update_ms("grav.x", its, ss.gravx, deg, TELEPLOT_FLAG_NOPLOT);
        teleplot.update_ms("grav.y", its, ss.gravy, deg, TELEPLOT_FLAG_NOPLOT);
        teleplot.update_ms("grav.z", its, ss.gravz, deg, TELEPLOT_FLAG_NOPLOT);
      }
    } else {
      if (options.report_raw) {
        JsonObject a = doc.createNestedObject("accel");
        a["t"] = now;
        a["x"] = ss.final_accel.acceleration.x;
        a["y"] = ss.final_accel.acceleration.y;
        a["z"] = ss.final_accel.acceleration.z;

        JsonObject j = doc.createNestedObject("gyro");
        j["t"] = now;
        j["x"] = ss.final_gyro.gyro.x;
        j["y"] = ss.final_gyro.gyro.y;
        j["z"] = ss.final_gyro.gyro.z;

        if (config.magnetometer_type != MAG_NONE) {
          JsonObject m = doc.createNestedObject("mag");
          m["t"] = now;
          m["x"] = ss.final_mag.magnetic.x;
          m["y"] = ss.final_mag.magnetic.y;
          m["z"] = ss.final_mag.magnetic.z;
        }
      }
      if (options.report_hpr) {
        JsonObject m = doc.createNestedObject("hpr");
        m["t"] = now;
        m["heading"] = ss.heading;
        m["pitch"] = ss.pitch;
        m["roll"] = ss.roll;
      }
      if (options.report_quat) {
        JsonObject m = doc.createNestedObject("quat");
        m["t"] = now;
        m["qw"] = ss.qw;
        m["qx"] = ss.qx;
        m["qy"] = ss.qy;
        m["qz"] = ss.qz;
      }
      if (options.report_grav) {
        JsonObject m = doc.createNestedObject("grav");
        m["t"] = now;
        m["x"] = ss.gravx;
        m["y"] = ss.gravy;
        m["z"] = ss.gravz;
      }
    }
  }
  if (options.report_baro) {
    if (config.bmp390_avail && options.report_baro) {
      const baro_report_t &bp = ss.baro_values[USE_BARO_BMP3XX];
      static uint32_t last;
      if (bp.timestamp > last) {

        if (options.teleplot_viewer) {
          teleplot.update_ms("bmp390.hpa", bp.timestamp, bp.hpa, hpascal,
                             TELEPLOT_FLAG_NOPLOT);
          teleplot.update_ms("bmp390.alt", bp.timestamp, bp.alt, meter,
                             TELEPLOT_FLAG_NOPLOT);
        } else {
          JsonObject j = doc.createNestedObject("bmp3xx");
          j["t"] = bp.timestamp;
          j["hpa"] = bp.hpa;
          j["alt"] = bp.alt;
        }
        last = bp.timestamp;
      }
    }

    if (config.lps22_avail) {
      const baro_report_t &bp = ss.baro_values[USE_BARO_LPS22];
      static uint32_t last;
      if (bp.timestamp > last) {
        if (options.teleplot_viewer) {
          teleplot.update_ms("lps2x.hpa", bp.timestamp, bp.hpa, hpascal,
                             TELEPLOT_FLAG_NOPLOT);
          teleplot.update_ms("lps2x.alt", bp.timestamp, bp.alt, meter,
                             TELEPLOT_FLAG_NOPLOT);
        } else {
          JsonObject j = doc.createNestedObject("lps2x");
          j["t"] = bp.timestamp;
          j["hpa"] = bp.hpa;
          j["alt"] = bp.alt;
        }
        last = bp.timestamp;
      }
    }

    if (config.dps3xx_avail) {
      const baro_report_t &bp = ss.baro_values[USE_BARO_DPS3XX];
      static uint32_t last;
      if (bp.timestamp > last) {
        if (options.teleplot_viewer) {
          teleplot.update_ms("dps3xx.hpa", bp.timestamp, bp.hpa, hpascal,
                             TELEPLOT_FLAG_NOPLOT);
          teleplot.update_ms("dps3xx.alt", bp.timestamp, bp.alt, meter,
                             TELEPLOT_FLAG_NOPLOT);
        } else {
          JsonObject j = doc.createNestedObject("dps3xx");
          j["t"] = bp.timestamp;
          j["hpa"] = bp.hpa;
          j["alt"] = bp.alt;
        }
        last = bp.timestamp;
      }
    }

    if (config.flowsensor_avail) {
      const flowsensor_report_t &report = ss.flowsensor_values;
      static uint32_t irqs = 999999;
      static bool flowing = true;
      if (true) {
        // if ((report.irqs ^ irqs) || (report.flowing ^ flowing)) {
        irqs = report.irqs;
        flowing = report.flowing;
        if (options.teleplot_viewer) {
          teleplot.update_ms("flowDetected", report.timestamp,
                             (int)report.flowing, bool_, TELEPLOT_FLAG_NOPLOT);
          teleplot.update_ms("flowCounter", report.timestamp, report.count,
                             counter_, TELEPLOT_FLAG_NOPLOT);
          teleplot.update_ms("flowBounces", report.timestamp, report.bounces,
                             counter_, TELEPLOT_FLAG_NOPLOT);
        } else {
          JsonObject j = doc.createNestedObject("flow");
          j["t"] = report.timestamp;
          j["count"] = report.count;
          j["bounces"] = report.bounces;
          j["flowing"] = report.flowing;
        }
      }
    }
  }

  if (true) {
    if (GPSFIX3D(serialGps) && serialGps.altitude.isUpdated()) {
      if (options.teleplot_viewer) {
        teleplot.update_ms("serialGps.alt", its,
                           serialGps.altitude.meters(), meter,
                           TELEPLOT_FLAG_NOPLOT);
      } else {
        JsonObject j = doc.createNestedObject("serialGps");
        j["t"] = its; // FIXME wrong
        j["alt"] = serialGps.altitude.meters();
      }
    }
  }
  if (ub_nav_pvt_ms > last_report_ms) {
    if (options.teleplot_viewer) {
      teleplot.update_ms("ubloxGps.alt", millis(), ub_nav_pvt.hMSL / 1000.0,
                         meter, TELEPLOT_FLAG_NOPLOT);
    } else {
      JsonObject j = doc.createNestedObject("ubloxGps");
      j["t"] = millis(); // FIXME wrong
      j["alt"] = ub_nav_pvt.hMSL / 1000.0;
    }
  }

  customReporterStats.Start();
  customReportingRateCode(ss, options);
  customReporterStats.Stop();

  if (run_stats) {
    emitStats(config, options, doc);
    run_stats = false;
  }

  if (!doc.isNull()) {
    if (!options.ndjson) {
      serializeJsonPretty(doc, Console);
      Console.print("\n");
    } else {
      serializeJson(doc, Console);
      Console.print("\n");
    }
  }
  last_report_us = micros();
  last_report_ms = millis();

  reporterStats.Stop();

#ifdef REPORT_PIN
  digitalWrite(REPORT_PIN, LOW);
#endif
}
#ifdef TIMING_STATS
#endif
#ifdef MEMORY_STATS
#endif
static void emitStats(config_t &config, options_t &opt,
                      DynamicJsonDocument &doc) {

  if (opt.timing_stats) {
    if (opt.teleplot_viewer) {
      teleplot.update_ms("handleImu.mean", millis(), imuStats.Mean(), uS,
                         TELEPLOT_FLAG_NOPLOT);
      teleplot.update_ms("reporter.mean", millis(), reporterStats.Mean(), uS,
                         TELEPLOT_FLAG_NOPLOT);
      teleplot.update_ms("customImu.mean", millis(), customImuStats.Mean(), uS,
                         TELEPLOT_FLAG_NOPLOT);
      teleplot.update_ms("customReporter.mean", millis(),
                         customReporterStats.Mean(), uS, TELEPLOT_FLAG_NOPLOT);
      teleplot.update_ms("slowSensor.mean", millis(), slowSensorStats.Mean(),
                         uS, TELEPLOT_FLAG_NOPLOT);

    } else {
      JsonObject m = doc.createNestedObject("timing");
      m["t"] = millis();
      m["handleImu.mean"] = imuStats.Mean();
      m["reporter.mean"] = reporterStats.Mean();
      m["customImu.mean"] = customImuStats.Mean();
      m["customReporter.mean"] = customReporterStats.Mean();
      m["slowSensor.mean"] = slowSensorStats.Mean();
    }
  }

  if (opt.memory_usage) {
    uint8_t ssp =
        (int)(100.0 *
              (REPORTERTASK_STACKSIZE -
               uxTaskGetStackHighWaterMark(reporterTask->GetHandle())) /
              REPORTERTASK_STACKSIZE);

    uint8_t rsp = (int)(100.0 *
                        (SENSORTASK_STACKSIZE -
                         uxTaskGetStackHighWaterMark(sensorTask->GetHandle())) /
                        SENSORTASK_STACKSIZE);
    uint8_t bsp =
        (int)(100.0 *
              (BACKGROUNDTASK_STACKSIZE -
               uxTaskGetStackHighWaterMark(backgroundTask->GetHandle())) /
              BACKGROUNDTASK_STACKSIZE);

    uint32_t freeHeap = ESP.getFreeHeap();

    if (opt.teleplot_viewer) {
      teleplot.update_ms("reporterStack", millis(), ssp, percent_,
                         TELEPLOT_FLAG_NOPLOT);
      teleplot.update_ms("sensorStack", millis(), rsp, percent_,
                         TELEPLOT_FLAG_NOPLOT);
      teleplot.update_ms("backgroundStack", millis(), bsp, percent_,
                         TELEPLOT_FLAG_NOPLOT);
      teleplot.update_ms("freeHeap", millis(), freeHeap, bytes_,
                         TELEPLOT_FLAG_NOPLOT);
    } else {
      JsonObject m = doc.createNestedObject("memory");
      m["t"] = millis();
      m["reporterStack"] = rsp;
      m["sensorStack"] = ssp;
      m["backgroundStack"] = bsp;
      m["freeHeap"] = freeHeap;
    }
  }
}
