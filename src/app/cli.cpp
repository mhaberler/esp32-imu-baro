
#include "custom.hpp"
#include "defs.hpp"
#include <ESPmDNS.h>
#include <WiFi.h>
#include <NimBLEDevice.h>

#include <FlowSensor.hpp>
#include <QuadratureDecoder.hpp>

extern FlowSensor flow_sensor;
extern QuadratureDecoder quad_sensor;

CmdCallback<NUM_COMMANDS> cmdCallback;
CmdBuffer<CMD_BUFSIZE> buffer;
CmdParser shell;
extern options_t options;
extern config_t config;

// remain in sync with slow_sensor_type_t
const char *sensor_types[] = {"none",   "lps22",  "dps3xx", "bmp3xx",
                              "ina219", "tmp117", "geiger", "<max>"};

static int num_services = -1;
extern MDNSResponder MDNS;
extern WiFiClass WiFi;

void printDevs(void) {
    dev_entry_t *d = imu_devices;

    while (d->dev != DEV_ENDMARK) {
        Console.fmtln("\t{} .. {}", (int)d->dev, d->name);
        d++;
    }
}

BaseType_t coreAffinity(const int arg) {
    switch (arg) {
        case 1:
        case 0:
            return arg;
        default:
            return tskNO_AFFINITY;
    }
}

void help(options_t &options, config_t &config, CmdParser *cp) {
    Console.fmtln("Type your commands. Supported: ");
    Console.fmtln("  help");
    Console.fmtln("  wire [bus sda scl khz] query or set I2C bus setting");
    Console.fmtln(
        "  spi [bus miso mosi sck kHz] query or set SPI bus "
        "pin/frequency setting");
    Console.fmtln("  sd [csPin cdPin kHz] query or set SD card settings");
    Console.fmtln("  wdt [seconds] query or set the wtachdog timer");
    Console.fmtln(
        "  ble [rate] | [rate core prio stack] query or set BLE scanner"
        "(rate < 0 disables BLE)");

    Console.fmtln(
        "  bs [<index> name macaddress type usage]"
        " list all, or configure a particular BLE sensor");

    Console.fmtln(
        "  ssid <ssid1> [ssid2] ...# set WiFi SSID's to connect to, "
        "currently {}:",
        options.num_ssid);
    for (auto i = 0; i < options.num_ssid; i++) {
        Console.fmtln("\t{} {}", options.ssids[i], options.passwords[i]);
    }
    Console.fmtln(
        "  pass <pass1> [pass2] ...# set corresponding WiFi passwords");

    Console.fmtln(
        "  tpdest <IP address> <port number> # teleplot UDP (use \"tpdest '' "
        "0\" to clear)"
        "destination: {}:{}",
        options.tpHost, options.tpPort);
    Console.fmtln(
        "  sd [service [proto]] # mDNS discovery, "
        "defaullt: teleplot udp");
    Console.fmtln(
        "  ss <index> # set teleplot destination to mDNS scan result by index");
    Console.fmtln("  i2c  # run i2c scan");
    Console.fmtln("  fs [pin] # report or set the flowsensor pin: {}",
                  options.flowsensor_pin);
    Console.fmtln(
        "  qs [pinA pinB] # report or set the quadrature sensor pins: {} "
        "{}",
        options.qs_pinA, options.qs_pinB);
    Console.fmtln("  dev <device> # set IMU device: {} ({})",
                  (int)options.selected_imu, options.selected_imu_name);
    Console.fmtln(
        "  use <int> # select baro sensor as KF input"
        " (0=none, 1=lps22, 2=dps3xx, 3=bmp3xx)?) :{} .. {}",
        (int)options.which_baro, sensor_types[options.which_baro]);
    Console.fmtln("  report <freq> # set reporting rate (HZ): {:.1f}",
                  options.report_rate);
    Console.fmtln("  poll  [<freq>] # get/set slow sensor poll rate");
    Console.fmtln("  report  [<freq> [prio [core]]]");
    Console.fmtln("       <freq> # set reporter rate (HZ): {:.1f}",
                  options.report_rate);
    Console.fmtln("       prio # set reporter priority: {}",
                  options.reporter_prio);
    Console.fmtln("       core # set reporter core: {} (reboot!)",
                  options.reporter_core);

    Console.fmtln("  imu  [<freq> [prio [core]]]");
    Console.fmtln("       <freq> # set imu sampling rate (HZ): {:.1f}",
                  options.imu_rate);
    Console.fmtln("       prio # set sensor priority: {}", options.sensor_prio);
    Console.fmtln("       core # set sensor core: {} (reboot!)",
                  options.sensor_core);

    Console.fmtln("  baro # toggle 'report barometer sensors' flag: {}",
                  B2S(options.report_baro));

    Console.fmtln("  hpr # toggle 'report heading/pitch/roll' flag: {}",
                  B2S(options.report_hpr));
    Console.fmtln("  quat # toggle 'report quaternions' flag: {}",
                  B2S(options.report_quat));
    Console.fmtln("  raw # toggle 'report raw sensor values' flag: {}",
                  B2S(options.report_raw));
    Console.fmtln("  filter # toggle AHRS filter execution: {}",
                  B2S(options.run_filter));

    Console.fmtln("  grav # toggle 'report gravity vector' flag: {}",
                  B2S(options.report_grav));
    Console.fmtln("  ned # toggle NED translation before AHRS' flag: {}",
                  B2S(options.ned));

    Console.fmtln("  ts # toggle timing statistics flag: {}",
                  B2S(options.timing_stats));

    Console.fmtln("  mu # toggle memory usage flag: {}",
                  B2S(options.memory_usage));
    Console.fmtln("  task [name] # show FreeRTOS task status");
    Console.fmtln("  heap # show heap status");
    Console.fmtln("  psram # show psram status");
    Console.fmtln("  plat # show platform report");
    Console.fmtln("  build # show build info");

    Console.fmtln("  apply # toggle 'apply calibration' flag: {}",
                  B2S(options.apply_cal));
    Console.fmtln(
        "  tele # toggle 'generate output for teleplot' "
        "flag: {}",
        B2S(options.teleplot_viewer));

    Console.fmtln("  mcal # enter MotionCal compass calibration");
    Console.fmtln("  gcal # run gyro calibration and store offsets");
    Console.fmtln("  par # show key params");

    Console.fmtln("  sg # show serial GPS status");
    Console.fmtln(
        "  sg rx tx speed uart # set serial GPS pins, speed, uart#- save "
        "and reboot");

    Console.fmtln("  ahrs <integer> # select AHRS algorithm: {}",
                  (int)options.ahrs_algo);
    Console.fmtln("     0 # NXP Fusion");
    Console.fmtln("     1 # Madgewick");
    Console.fmtln("     2 # Mahoney");
    // Console.fmtln("     4 # har-in-air Mahoney");

    Console.fmtln("  log  [integer] # show or set log level: {}",
                  options.log_level);
    Console.fmtln("  trace  [integer] # show or set trace mask: {:X}",
                  options.trace);
    Console.fmtln("  root   <path> # webserver root directory: '{}'",
                  options.littlefs_static_path);
    Console.fmtln("  save");
    Console.fmtln(
        "  wipe  # erase preferences and set to default (factory reset)");
    Console.fmtln("  ndjson # toggle NDJSON printing: {}", B2S(options.ndjson));

    Console.fmtln("  reboot");
    customHelpText(config, options);
    Console.fmtln("currently active sensors: ");
    printSensorsDetected(options, config);
    Console.fmtln("supported devices:");
    printDevs();
    printCurrentCalibration(options, config);
}

const char *algoName(ahrs_algo_t algo) {
    switch (algo) {
        case ALGO_NXP_FUSION:
            return "NXP Fusion";
        case ALGO_MADGEWICK:
            return "Madgewick";
        case ALGO_MAHONEY:
            return "Mahoney";
        case ALGO_HAR_IN_AIR:
            return "har-in-air";
        default:
            return "invalid";
    }
}

void initShell(void) {
    shell.setOptIgnoreQuote(false);
    shell.setOptKeyValue(true);

    cmdCallback.addCmd("HELP",
                       [](CmdParser *cp) { printHelp(options, config); });
    cmdCallback.addCmd("WIRE", [](CmdParser *cp) {
        if (cp->getParamCount() < 2) {
            for (auto i = 0; i < NUM_I2C; i++) {
                i2c_cfg_t *ip = &options.i2c_cfg[i];
                Console.fmtln("WIRE{}: sda={} scl={} kHz={}", i, ip->sda,
                              ip->scl, ip->kHz);
            }
            return;
        }
        if (cp->getParamCount() == 5) {
            int8_t bus    = (atoi(cp->getCmdParam(1)) & (NUM_I2C - 1));
            i2c_cfg_t *ip = &options.i2c_cfg[bus];

            ip->sda = atoi(cp->getCmdParam(2));
            ip->scl = atoi(cp->getCmdParam(3));
            ip->kHz = static_cast<uint16_t>(atoi(cp->getCmdParam(4)));
            bool result;
            switch (bus) {
                case 0:
                    Wire.end();
                    result = Wire.begin(ip->sda, ip->scl, ip->kHz * 1000);
                    break;
                case 1:
                    Wire1.end();
                    result = Wire1.begin(ip->sda, ip->scl, ip->kHz * 1000);
                    break;
                default:
                    Console.fmtln("no such I2C bus: {}", bus);
                    return;
            }
            Console.fmtln("Wire{}.begin(sda={}, scl={}, clock={}kHz) = {}", bus,
                          ip->sda, ip->scl, ip->kHz, T2OK(result));
        } else {
            Console.fmtln("WIRE command requires 4 params: busnum sda scl kHz");
        }
    });

    cmdCallback.addCmd("SPI", [](CmdParser *cp) {
        if (cp->getParamCount() < 2) {
            for (auto i = 0; i < NUM_SPI; i++) {
                spi_cfg_t *sc = &options.spi_cfg[i];
                Console.fmtln("SPI{}: miso={} mosi={} sck={}", i, sc->miso,
                              sc->mosi, sc->sck);
            }
            return;
        }
        if (cp->getParamCount() == 5) {
            int8_t bus    = (atoi(cp->getCmdParam(1)) & (NUM_SPI - 1));
            spi_cfg_t *sc = &options.spi_cfg[bus];

            sc->miso = atoi(cp->getCmdParam(2));
            sc->mosi = atoi(cp->getCmdParam(3));
            sc->sck  = atoi(cp->getCmdParam(4));
            // sc->kHz = static_cast<uint16_t>(atoi(cp->getCmdParam(5)));
            Console.fmtln("SPI{} set to: miso={} mosi={} sck={}", bus, sc->miso,
                          sc->mosi, sc->sck);
        }
    });
    // Console.fmtln("  sd [csPin cdPin kHz] query or set SD card settings");
    cmdCallback.addCmd("SD", [](CmdParser *cp) {
        if (cp->getParamCount() < 2) {
            Console.fmtln("SD: csPin={} cdPin={} kHz={}", options.sd_cs_pin,
                          options.sd_card_detect_pin, options.sd_freq_kHz);
            return;
        }

        if (cp->getParamCount() == 4) {
            options.sd_cs_pin          = atoi(cp->getCmdParam(1));
            options.sd_card_detect_pin = atoi(cp->getCmdParam(2));
            options.sd_freq_kHz =
                static_cast<uint32_t>(atoi(cp->getCmdParam(3)));
            Console.fmtln("SD set to: csPin={} cdPin={} kHz={}",
                          options.sd_cs_pin, options.sd_card_detect_pin,
                          options.sd_freq_kHz);
        }
    });
    cmdCallback.addCmd("WDT", [](CmdParser *cp) {
        if (cp->getParamCount() < 2) {
            Console.fmtln("watchdog: seconds={}", options.watchdog);
            return;
        }

        if (cp->getParamCount() == 2) {
            options.watchdog = atof(cp->getCmdParam(1));
            Console.fmtln("watchdog: set to seconds={}", options.watchdog);
        }
    });
    cmdCallback.addCmd("BLE", [](CmdParser *cp) {
        if (cp->getParamCount() < 2) {
            Console.fmtln("BLE: rate={} core={} prio={} stack={}",
                          options.ble_rate, options.ble_core, options.ble_prio,
                          options.ble_stack);
            return;
        }
        if (cp->getParamCount() == 2) {
            options.ble_rate = atof(cp->getCmdParam(1));
            Console.fmtln("BLE rate set to: rate={}", options.ble_rate);
        }
        if (cp->getParamCount() == 5) {
            options.ble_rate  = atof(cp->getCmdParam(1));
            options.ble_core  = atoi(cp->getCmdParam(2));
            options.ble_prio  = atoi(cp->getCmdParam(3));
            options.ble_stack = atoi(cp->getCmdParam(4));
            Console.fmtln("BLE set to: rate={} core={} prio={} stack={}",
                          options.ble_rate, options.ble_core, options.ble_prio,
                          options.ble_stack);
        }
    });

    // "  bs [<index> name macaddress type usage]"
    // "list all, or configure a particular BLE sensor");

    cmdCallback.addCmd("BS", [](CmdParser *cp) {
        if (cp->getParamCount() < 2) {
            int n = 0;
            for (auto i = 0; i < NUM_BLESENSORS; i++) {
                BLEsensor_t *bp = &options.blesensors[i];
                switch (bp->type) {
                    case SENSOR_NONE:
                        continue;
                    default:
                        n++;
                        Console.fmtln(
                            "sensor {}: name={} type={} usage={} mac={}", i,
                            bp->name, (uint32_t)bp->type, (uint32_t)bp->usage,
                            bp->blemac);
                }
            }
            Console.fmtln("ble sensors configured: {}", n);
            return;
        }
        if (strcasecmp(cp->getCmdParam(1), "clear") == 0) {
            memset(options.blesensors, 0, sizeof(options.blesensors));
            Console.fmtln("bs clear: wiped ble sensors table");
            return;
        }
        if (cp->getParamCount() != 6) {
            Console.fmtln("usage: bs [<index> name macaddress type usage]");
            return;
        }
        uint32_t i = atoi(cp->getCmdParam(1));
        if (i > NUM_BLESENSORS - 1) {
            Console.fmtln("index {} out of range, must be <  {}", i,
                          NUM_BLESENSORS);
            return;
        }
        BLEsensor_t *bp           = &options.blesensors[i];
        NimBLEAddress const &zero = NimBLEAddress((uint64_t)0, (uint8_t)0);
        NimBLEAddress const &adr =
            NimBLEAddress(std::string(cp->getCmdParam(3)));

        if (adr.equals(zero)) {
            Console.fmtln("could not parse '{}' into a MAC address",
                          cp->getCmdParam(3));
            return;
        }
        // remember the converted-back version of what was given
        strncpy(bp->blemac, std::string(adr).c_str(), sizeof(bp->blemac));
        // remember in parsed form
        // config.nimble_adr[i] = adr;

        strncpy(bp->name, cp->getCmdParam(2), sizeof(bp->name));
        bp->type  = (sensor_type_t)atoi(cp->getCmdParam(4));
        bp->usage = (ble_sensor_usage_t)atoi(cp->getCmdParam(5));
        Console.fmtln("sensor {} set to: name={} type={} usage={} mac={}", i,
                      bp->name, (uint32_t)bp->type, (uint32_t)bp->usage,
                      bp->blemac);
    });

    cmdCallback.addCmd("SSID", [](CmdParser *cp) {
        if (cp->getParamCount() < 2) {
            Console.fmtln("ssid: at least one SSID needed");
            return;
        }
        Console.fmtln("{} ssid's present:", cp->getParamCount());
        for (auto i = 1; i < cp->getParamCount(); i++) {
            strcpy(options.ssids[i - 1], cp->getCmdParam(i));
            Console.fmtln("adding SSID '{}'", options.ssids[i - 1]);
        }
        options.num_ssid = cp->getParamCount() - 1;
    });
    cmdCallback.addCmd("PASS", [](CmdParser *cp) {
        if (cp->getParamCount() != (options.num_ssid + 1)) {
            Console.fmtln(
                "pass: same number as SSID's must be given (found {})",
                cp->getParamCount());
            return;
        }
        Console.fmtln("{} passwords present:", cp->getParamCount());
        for (auto i = 1; i < cp->getParamCount(); i++) {
            strcpy(options.passwords[i - 1], cp->getCmdParam(i));
            Console.fmtln("adding password '{}' for '{}'",
                          options.passwords[i - 1], options.ssids[i - 1]);
        }
    });
    cmdCallback.addCmd("TPDEST", [](CmdParser *cp) {
        if (cp->getParamCount() != 3) {
            Console.fmtln("format:  tpdest <IP address> <port number>");
            return;
        }
        const char *h = cp->getCmdParam(1);
        if (strcmp(h, "''") == 0) {
            memset(options.tpHost, 0, sizeof(options.tpHost));
            options.tpPort = 0;
            Console.fmtln("teleplot UDP destination cleared");
            return;
        } else {
            strcpy(options.tpHost, h);
        }
        options.tpPort = atoi(cp->getCmdParam(2));
        Console.fmtln("teleplot UDP destination set to: {}:{}", options.tpHost,
                      options.tpPort);
    });
    cmdCallback.addCmd("TASK", [](CmdParser *cp) {
        if (cp->getParamCount() == 2) {
            Console.fmtln("this will be fixed");
            return;
        }
        task_report(NULL, 0);
    });

    cmdCallback.addCmd("HEAP", [](CmdParser *cp) { heap_report(NULL, 0); });
    cmdCallback.addCmd("PSRAM", [](CmdParser *cp) { psram_report(NULL, 0); });
    cmdCallback.addCmd("PLAT", [](CmdParser *cp) { platform_report(); });
    cmdCallback.addCmd("BUILD", [](CmdParser *cp) { build_setup_report(); });
    cmdCallback.addCmd("SD", [](CmdParser *cp) {
        const char *service = "teleplot";
        const char *proto   = "udp";
        switch (cp->getParamCount()) {
            case 1:
                break;
            case 2:
                service = cp->getCmdParam(1);
                break;
            case 3:
                service = cp->getCmdParam(1);
                proto   = cp->getCmdParam(2);
                break;
            default:
                Console.fmtln("format:  sd [<service> [<proto>]]");
                return;
        }
        num_services = browseService(service, proto);
    });

    cmdCallback.addCmd("SS", [](CmdParser *cp) {
        if (cp->getParamCount() != 2) {
            Console.fmtln("format:  ss <index>");
            return;
        }
        if (num_services < 0) {
            Console.fmtln("ss: run 'sd <service> <proto>' first");
            return;
        }
        int index = atoi(cp->getCmdParam(1));
        if (index < 0) {
            // erase
            memset(options.tpHost, 0, sizeof(options.tpHost));
            options.tpPort = -1;
            return;
        }
        if (index > num_services) {
            Console.fmtln("ss: index out of bounds: {}", cp->getCmdParam(1));
            return;
        }
        memcpy(options.tpHost, MDNS.IP(index).toString().c_str(),
               sizeof(options.tpHost));
        ;
        options.tpPort = MDNS.port(index);
        Console.fmtln("ss: teleplot destination {}:{}", options.tpHost,
                      options.tpPort);
    });
    cmdCallback.addCmd("I2C", [](CmdParser *cp) {
        // reporterTask->suspend();
        // if (sensorTask) sensorTask->suspend();
        flushBuffers();

        LOGD("start");
        i2cScan();
        LOGD("end");
        flushBuffers();

        // reporterTask->resume();
        // if (sensorTask) sensorTask->resume();
    });
    cmdCallback.addCmd("DEV", [](CmdParser *cp) {
        if (cp->getParamCount() < 2) {
            Console.fmtln("current IMU dev {} {}", (int)options.selected_imu,
                          options.selected_imu_name);
            return;
        }
        if (cp->getParamCount() < 4) {
            options.selected_imu      = (device_type_t)atoi(cp->getCmdParam(1));
            options.selected_imu_name = imu_devices[options.selected_imu].name;

            Console.fmtln("selected dev {} {}", (int)options.selected_imu,
                          options.selected_imu_name);

            if (options.selected_imu != DEV_NONE) {
                // bool was_running = (sensorTask && sensorTask->isRunning());
                // if (was_running) {
                //     sensorTask->suspend(true);
                // }
                if (initIMU(options, config)) {
                    selectAHRS(options, config);
                    Console.fmtln("selected dev {} {} - save and reboot",
                                  (int)options.selected_imu,
                                  options.selected_imu_name);
                }
                // if (was_running) {
                //     sensorTask->resume();
                // }
                flushBuffers();
            }
        }
        if (cp->getParamCount() == 3) {
            options.rotation = static_cast<uint8_t>(atoi(cp->getCmdParam(2)));
            Console.fmtln("rotation set to: {}", (int)options.rotation);
        }
    });
    cmdCallback.addCmd("USE", [](CmdParser *cp) {
        if (cp->getParamCount() != 2) {
            Console.fmtln("using baro {} .. {}", (int)options.which_baro,
                          sensor_types[options.which_baro]);
            return;
        }
        options.which_baro = (i2c_dev_t)atoi(cp->getCmdParam(1));
        Console.fmtln("using baro {} .. {}", (int)options.which_baro,
                      sensor_types[options.which_baro]);
    });
    cmdCallback.addCmd("POLL", [](CmdParser *cp) {
        if (cp->getParamCount() != 2) {
            Console.fmtln("current poll frequency: {}", options.sensor_rate);
            return;
        }
        options.sensor_rate = atof(cp->getCmdParam(1));
        Console.fmtln("using poll frequency: {}", options.sensor_rate);
    });

    cmdCallback.addCmd("REPORT", [](CmdParser *cp) {
        if (cp->getParamCount() == 1) {
            Console.fmtln(
                "current report rate: {:.1f} Hz, prio {}, core {}, stack {}%",
                options.report_rate, reporterTask->GetPriority(),
                options.reporter_core, config.reporter_stack_util);
            return;
        }
        if (cp->getParamCount() > 1) {
            options.report_rate = atof(cp->getCmdParam(1));
            reporterTask->setRate(1000.0 / options.report_rate);
            Console.fmtln("report rate set to {:.1f} Hz", options.report_rate);
        }
        if (cp->getParamCount() > 2) {
            options.reporter_prio = atoi(cp->getCmdParam(2));
            reporterTask->SetPriority(options.reporter_prio);
            Console.fmtln("reporter prio set to  {}", options.reporter_prio);
        }
        if (cp->getParamCount() > 3) {
            options.reporter_core = coreAffinity(atoi(cp->getCmdParam(3)));
            Console.fmtln(
                "--- new reporter core assignment {} valid after reboot!",
                options.reporter_core);
        }
    });
    cmdCallback.addCmd("IMU", [](CmdParser *cp) {
        if (cp->getParamCount() == 1) {
            Console.fmtln(
                "current IMU rate: {:.1f} Hz, prio {}, core {}, stack {}%",
                options.imu_rate, options.sensor_prio, options.sensor_core,
                config.sensor_stack_util);
            return;
        }
        options.imu_rate = atof(cp->getCmdParam(1));

        if (cp->getParamCount() > 1) {
            options.imu_rate = atof(cp->getCmdParam(1));
            if (sensorTask) sensorTask->setRate(1000.0 / options.imu_rate);
            Console.fmtln("IMU update rate set to {:.1f} Hz", options.imu_rate);
        }
        if (cp->getParamCount() > 2) {
            options.sensor_prio = atof(cp->getCmdParam(2));
            if (sensorTask) sensorTask->SetPriority(options.sensor_prio);
            Console.fmtln("sensor prio set to  {}", options.sensor_prio);
        }
        if (cp->getParamCount() > 3) {
            options.sensor_core = coreAffinity(atoi(cp->getCmdParam(3)));
            Console.fmtln(
                "--- new sensor core assignment {} valid after reboot!",
                options.sensor_core);
        }
    });

    cmdCallback.addCmd("HPR", [](CmdParser *cp) {
        options.report_hpr = !options.report_hpr;
        Console.fmtln("heading/ptch/roll reporting: {}",
                      B2S(options.report_hpr));
    });
    cmdCallback.addCmd("QUAT", [](CmdParser *cp) {
        options.report_quat = !options.report_quat;
        Console.fmtln("quaternion reporting: {}", B2S(options.report_quat));
    });
    cmdCallback.addCmd("RAW", [](CmdParser *cp) {
        options.report_raw = !options.report_raw;
        Console.fmtln("raw accel/gyro/mag reporting: {}",
                      B2S(options.report_raw));
    });
    cmdCallback.addCmd("GRAV", [](CmdParser *cp) {
        options.report_grav = !options.report_grav;
        Console.fmtln("gravity vector reporting: {}", B2S(options.report_grav));
    });
    cmdCallback.addCmd("BARO", [](CmdParser *cp) {
        options.report_baro = !options.report_baro;
        Console.fmtln("baro sensor reporting: {}", B2S(options.report_baro));
    });
    cmdCallback.addCmd("FILTER", [](CmdParser *cp) {
        options.run_filter = !options.run_filter;
        Console.fmtln("AHRS filter execution: {}", B2S(options.run_filter));
    });
    cmdCallback.addCmd("TELE", [](CmdParser *cp) {
        options.teleplot_viewer = !options.teleplot_viewer;
        Console.fmtln("teleplot mode reporting: {}",
                      B2S(options.teleplot_viewer));
    });
    cmdCallback.addCmd("APPLY", [](CmdParser *cp) {
        options.apply_cal = !options.apply_cal;
        Console.fmtln("{}", options.apply_cal
                                ? "applying calibration"
                                : "reporting uncalibrated values");
    });
    cmdCallback.addCmd("MCAL", [](CmdParser *cp) {
        motion_cal = true;
        Console.fmtln("Starting MotionCal mode");
    });
    cmdCallback.addCmd("GCAL", [](CmdParser *cp) {
        config.gcal_samples = GYRO_SAMPLES;
        Console.fmtln("starting gyro calibration, {} samples..",
                      config.gcal_samples);
    });

    cmdCallback.addCmd("AHRS", [](CmdParser *cp) {
        if (cp->getParamCount() != 2) {
            Console.fmtln("need an int parameter");
            return;
        }
        options.ahrs_algo = (ahrs_algo_t)atoi(cp->getCmdParam(1));
        selectAHRS(options, config);
        Console.fmtln("AHRS algorithm set to: {}", algoName(options.ahrs_algo));
    });
    cmdCallback.addCmd("LOG", [](CmdParser *cp) {
        if (cp->getParamCount() != 2) {
            Console.fmtln("log level: {}", options.log_level);
            return;
        }
        options.log_level = atoi(cp->getCmdParam(1));
        set_syslog_loglevel(options.log_level);
    });
    cmdCallback.addCmd("TRACE", [](CmdParser *cp) {
        if (cp->getParamCount() != 2) {
            Console.fmtln("trace mask: {:X}", options.trace);
            return;
        }
        options.trace = atoi(cp->getCmdParam(1));
        Console.fmtln("trace mask set to: {:X}", options.trace);
    });
    cmdCallback.addCmd("NED", [](CmdParser *cp) {
        options.ned = !options.ned;
        Console.fmtln("North-East-Down adjustment: {}", B2S(options.ned));
    });
    cmdCallback.addCmd("TS", [](CmdParser *cp) {
        options.timing_stats = !options.timing_stats;
        Console.fmtln("timing statistics: {}", B2S(options.timing_stats));
    });
    cmdCallback.addCmd("MU", [](CmdParser *cp) {
        options.memory_usage = !options.memory_usage;
        Console.fmtln("memory usage: {}", B2S(options.memory_usage));
    });
    cmdCallback.addCmd("ROOT", [](CmdParser *cp) {
        if (cp->getParamCount() != 2) {
            Console.fmtln("root: level: {}", options.littlefs_static_path);
            return;
        }
        strcpy(options.littlefs_static_path, cp->getCmdParam(1));
        Console.fmtln("root: set to '{}'", options.littlefs_static_path);
    });
    cmdCallback.addCmd("PAR", [](CmdParser *cp) {
        Console.fmtln("imu={} baro={} ned={} cal={} kf={}",
                      options.selected_imu_name,
                      sensor_types[options.which_baro], B2S(options.ned),
                      B2S(options.apply_cal), (int)options.which_kfmask);
        Console.fmtln(
            "irate={:.1f} rrate={:.1f} srate={:.1f} blerate={:.1f} filter={}",
            options.imu_rate, options.report_rate, options.sensor_rate,
            options.ble_rate, (options.run_filter));
        Console.fmtln("current tpdest={}:{} ({})",
                      config.currentTpHost.toString().c_str(),
                      config.currentTpPort,
                      strlen(options.tpHost) ? "manually set" : "via mDNS");
        Console.fmtln("IP={} GW={} netmask={} rssi={}",
                      WiFi.localIP().toString().c_str(),
                      WiFi.gatewayIP().toString().c_str(),
                      WiFi.subnetMask().toString().c_str(), WiFi.RSSI());
        Console.fmtln("flowsensor pin={}/{} IRQs={} enabled={}",
                      flow_sensor.pinNum(),
                      T2OK(digitalPinIsValid(flow_sensor.pinNum())),
                      flow_sensor.numIrqs(), flow_sensor.enabled());
        Console.fmtln(
            "quadrature sensor pinA={}/{} pinB={}/{} IRQs={} enabled={}",
            quad_sensor.pinA(), T2OK(digitalPinIsValid(quad_sensor.pinA())),
            quad_sensor.pinB(), T2OK(digitalPinIsValid(quad_sensor.pinB())),
            quad_sensor.numIrqs(), quad_sensor.enabled());
        Console.fmtln("BLE rate={:.1f} ads={}", options.ble_rate,
                      config.ble_ads);
        Console.fmtln("Power button: wasClicked={} getClickCount={}",
                      M5.BtnPWR.wasClicked(), M5.BtnPWR.getClickCount());
    });
    cmdCallback.addCmd("ND", [](CmdParser *cp) {
        options.ndjson = !options.ndjson;

        Console.fmtln("ndjson: {}", B2S(options.ndjson));
    });
    cmdCallback.addCmd("SAVE", [](CmdParser *cp) {
        Console.fmtln("Saving preferences...");
        savePrefs(options);
        Console.fmtln("done.");
    });
    cmdCallback.addCmd("WIPE", [](CmdParser *cp) {
        Console.fmtln("factory reset...");
        wipePrefs();
        ESP.restart();
    });
    cmdCallback.addCmd("REBOOT", [](CmdParser *cp) {
        Console.fmtln("Rebooting...");
        ESP.restart();
    });
#ifdef SERIAL_GPS
    cmdCallback.addCmd("SG", [](CmdParser *cp) {
        if (cp->getParamCount() == 1) {
            Console.fmtln("serial GPS status:");
            Console.fmtln(
                "configured for: rx={} tx={} speed={} "
                "UART{} TinyGPSPlus={}",
                options.gps_rx_pin, options.gps_tx_pin, options.gps_speed,
                GPS_UART, serialGps.libraryVersion());
            serialGpsStats();
            return;
        }
        // speed rx tx format
        if (cp->getParamCount() == 4) {
            options.gps_rx_pin = atoi(cp->getCmdParam(1));
            options.gps_tx_pin = atoi(cp->getCmdParam(2));
            options.gps_speed  = atoi(cp->getCmdParam(3));
            Console.fmtln("serial GPS - params set to: rx={} tx={} speed={}",
                          options.gps_rx_pin, options.gps_tx_pin,
                          options.gps_speed);
            init_serial_gps(options);
        }
    });
#endif
    cmdCallback.addCmd("FS", [](CmdParser *cp) {
        if (cp->getParamCount() != 2) {
            Console.fmtln("current flowsensor pin: {}", options.flowsensor_pin);
            return;
        }
        options.flowsensor_pin = atoi(cp->getCmdParam(1));
        Console.fmtln("flowsensor pin set to:: {}", options.flowsensor_pin);
    });
    cmdCallback.addCmd("QS", [](CmdParser *cp) {
        if (cp->getParamCount() != 3) {
            Console.fmtln("current quadrature sensor pins: {} {}",
                          options.qs_pinA, options.qs_pinB);
            return;
        }
        options.qs_pinA = atoi(cp->getCmdParam(1));
        options.qs_pinB = atoi(cp->getCmdParam(2));
        Console.fmtln("quadrature sensor pins set to: {} {}", options.qs_pinA,
                      options.qs_pinB);
    });
    customCommands(config, options);
    buffer.setEcho(true);
}

void serialwriteFunc(const uint8_t writeChar) {
    Console.write(writeChar);
}

void serialCmdComplete(CmdParser &cmdParser, bool found) {
    if (!found) {
        const char *cmd = cmdParser.getCommand();
        if ((*cmd == '#') || (*cmd == ';')) {
            // a comment. ignore.
            return;
        }
        Console.fmtln("command not found: '{}' ", cmd);
        Console.fmtln("type 'help' for a list of commands");
    }
}

void testSerial() {
    while (Serial && Serial.available()) {
        uint8_t readChar = Serial.read();
        cmdCallback.updateCmdProcessing(&shell, &buffer, readChar,
                                        serialwriteFunc, serialCmdComplete);
    }
}

void printHelp(options_t &options, config_t &config) {
    help(options, config, &shell);
}