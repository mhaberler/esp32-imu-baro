
#include "custom.hpp"
#include "defs.hpp"
#include <ESPmDNS.h>
// #include <StreamLib.h>
#include <WiFi.h>

CmdCallback<NUM_COMMANDS> cmdCallback;
CmdBuffer<CMD_BUFSIZE> buffer;
CmdParser shell;

// remain in sync with use_baro_t
const char *baro_types[] = {"none", "lps22", "dps3xx", "bmp3xx"};

static int num_services = -1;
extern MDNSResponder MDNS;
extern WiFiClass WiFi;

void printHelp(options_t &options);

void printDevs(void) {
  dev_entry_t *d = imu_devices;

  while (d->dev != DEV_ENDMARK) {
    Console.fmt("\t{} .. {}\n", d->dev, d->name);
    d++;
  }
}

void help(options_t &options, CmdParser *cp) {
  Console.fmt("Type your commands. Supported: \n");
  Console.fmt("  help\n");
  Console.fmt("  ssid <ssid1> [ssid2] ...# set WiFi SSID's to connect to, "
              "currently {}:\n",
              options.num_ssid);
  for (auto i = 0; i < options.num_ssid; i++) {
    Console.fmt("\t{} {}\n", options.ssids[i], options.passwords[i]);
  }
  Console.fmt("  pass <pass1> [pass2] ...# set corresponding WiFi passwords\n");

  Console.fmt("  tpdest <IP address> <port number> # teleplot UDP "
              "destination: {}:{}\n",
              options.tpHost, options.tpPort);
  Console.fmt("  sd [service [proto]] # mDNS discovery, "
              "defaullt: teleplot udp\n");
  Console.fmt(
      "  ss <index> # set teleplot destination to mDNS scan result by index\n");
  Console.fmt("  i2c  # run i2c scan\n");
  Console.fmt("  fs [pin] # report or set the flowsensor pin: {}\n",
              options.flowsensor_pin);
  Console.fmt("  dev <device> # set IMU device: {} ({})\n",
              options.selected_imu, options.selected_imu_name);
  Console.fmt("  use <int> # select baro sensor as KF input"
              " (0=none, 1=lps22, 2=dps3xx, 3=bmp3xx)?) :{} .. {}\n",
              options.which_baro, baro_types[options.which_baro]);
  Console.fmt("  report <freq> # set reporting rate (HZ): {:.1f}\n",
              options.report_rate);
  Console.fmt("  imu  <freq> # set imu sampling rate (HZ): {:.1f}\n",
              options.imu_rate);
  Console.fmt("  br  <freq> # set background task rate (HZ): {:.1f}\n",
              options.background_rate);

  Console.fmt("  baro # toggle 'report barometer sensors' flag: {}\n",
              B2S(options.report_baro));

  Console.fmt("  hpr # toggle 'report heading/pitch/roll' flag: {}\n",
              B2S(options.report_hpr));
  Console.fmt("  quat # toggle 'report quaternions' flag: {}\n",
              B2S(options.report_quat));
  Console.fmt("  raw # toggle 'report raw sensor values' flag: {}\n",
              B2S(options.report_raw));
  Console.fmt("  filter # toggle AHRS filter execution: {}\n",
              B2S(options.run_filter));

  Console.fmt("  grav # toggle 'report gravity vector' flag: {}\n",
              B2S(options.report_grav));
  Console.fmt("  ned # toggle NED translation before AHRS' flag: {}\n",
              B2S(options.ned));

  Console.fmt("  ts # toggle timing statistics flag: {}\n",
              B2S(options.timing_stats));

  Console.fmt("  mu # toggle memory usage flag: {}\n",
              B2S(options.memory_usage));

  Console.fmt("  apply # toggle 'apply calibration' flag: {}\n",
              B2S(options.apply_cal));
  Console.fmt("  tele # toggle 'generate output for teleplot' "
              "flag: {}\n",
              B2S(options.teleplot_viewer));

  Console.fmt("  mcal # enter MotionCal compass calibration\n");
  Console.fmt("  gcal # run gyro calibration and store offsets\n");
  Console.fmt("  par # show key params\n");

  Console.fmt("  sg # show serial GPS status\n");
  Console.fmt("  sg rx tx speed uart # set serial GPS pins, speed, uart#- save "
              "and reboot\n");

  Console.fmt("  ahrs <integer> # select AHRS algorithm: {}\n",
              options.ahrs_algo);
  Console.fmt("     0 # NXP Fusion\n");
  Console.fmt("     1 # Madgewick\n");
  Console.fmt("     2 # Mahoney\n");
  // Console.fmt("     4 # har-in-air Mahoney\n");

  Console.fmt("  debug  [integer] # show or set debug level: {}\n",
              options.debug);

  Console.fmt("  save\n");
  Console.fmt(
      "  wipe  # erase preferences and set to default (factory reset)\n");
  Console.fmt("  ndjson # toggle NDJSON printing: {}\n", B2S(options.ndjson));

  Console.fmt("  reboot\n");
  customHelpText(config, options);
  Console.fmt("currently active sensors: ");
  printSensorsDetected();
  Console.fmt("supported devices:\n");
  printDevs();
  printCurrentCalibration();
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

extern options_t options;
extern config_t config;
void initShell(void) {

  cmdCallback.addCmd("HELP", [](CmdParser *cp) { printHelp(options); });
  cmdCallback.addCmd("SSID", [](CmdParser *cp) {
    if (cp->getParamCount() < 2) {
      Console.fmt("ssid: at least one SSID needed\n");
      return;
    }
    for (auto i = 1; i < cp->getParamCount(); i++) {
      strcpy(options.ssids[i - 1], cp->getCmdParam(i));
      Console.fmt("adding SSID '{}'\n", options.ssids[i - 1]);
    }
    options.num_ssid = cp->getParamCount() - 1;
  });
  cmdCallback.addCmd("PASS", [](CmdParser *cp) {
    if (cp->getParamCount() != (options.num_ssid + 1)) {
      Console.fmt("pass: same number as SSID's must be given\n");
      return;
    }
    for (auto i = 1; i < cp->getParamCount(); i++) {
      strcpy(options.passwords[i - 1], cp->getCmdParam(i));
      Console.fmt("adding password '{}' for '{}'\n", options.passwords[i - 1],
                  options.ssids[i - 1]);
    }
  });
  cmdCallback.addCmd("TPDEST", [](CmdParser *cp) {
    if (cp->getParamCount() != 3) {
      Console.fmt("format:  tpdest <IP address> <port number>\n");
      return;
    }
    strcpy(options.tpHost, cp->getCmdParam(1));
    options.tpPort = atoi(cp->getCmdParam(2));
    Console.fmt("teleplot USP destination set to: {}:{}\n", options.tpHost,
                options.tpPort);
  });
  cmdCallback.addCmd("SD", [](CmdParser *cp) {
    const char *service = "teleplot";
    const char *proto = "udp";
    switch (cp->getParamCount()) {
    case 1:
      break;
    case 2:
      service = cp->getCmdParam(1);
      break;
    case 3:
      service = cp->getCmdParam(1);
      proto = cp->getCmdParam(2);
      break;
    default:
      Console.fmt("format:  sd [<service> [<proto>]]\n");
      return;
    }
    num_services = browseService(service, proto);
  });

  cmdCallback.addCmd("SS", [](CmdParser *cp) {
    if (cp->getParamCount() != 2) {
      Console.fmt("format:  ss <index>\n");
      return;
    }
    if (num_services < 0) {
      Console.fmt("ss: run 'sd <service> <proto>' first\n");
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
      Console.fmt("ss: index out of bounds: {}\n", cp->getCmdParam(1));
      return;
    }
    memcpy(options.tpHost, MDNS.IP(index).toString().c_str(),
           sizeof(options.tpHost));
    ;
    options.tpPort = MDNS.port(index);
    Console.fmt("ss: teleplot destination {}:{}\n", options.tpHost,
                options.tpPort);
  });
  cmdCallback.addCmd("I2C", [](CmdParser *cp) { i2cScan(); });
  cmdCallback.addCmd("DEV", [](CmdParser *cp) {
    if (cp->getParamCount() != 2) {
      Console.fmt("need an int parameter\n");
      return;
    }
    options.selected_imu = (device_type_t)atoi(cp->getCmdParam(1));
    options.selected_imu_name = imu_devices[options.selected_imu].name;
    Console.fmt("selected dev {} {}\n", options.selected_imu,
                options.selected_imu_name);
  });
  cmdCallback.addCmd("USE", [](CmdParser *cp) {
    if (cp->getParamCount() != 2) {
      Console.fmt("need an int parameter\n");
      return;
    }
    options.which_baro = (use_baro_t)atoi(cp->getCmdParam(1));
    Console.fmt("using baro {} .. {}\n", options.which_baro,
                baro_types[options.which_baro]);
  });
  cmdCallback.addCmd("REPORT", [](CmdParser *cp) {
    if (cp->getParamCount() != 2) {
      Console.fmt("current report rate: {:.1f} Hz\n", options.report_rate);
      return;
    }
    options.report_rate = atof(cp->getCmdParam(1));
    reporterTask->setRate(1000.0 / options.report_rate);
    Console.fmt("report rate set to {:.1f} Hz\n", options.report_rate);
  });
  cmdCallback.addCmd("IMU", [](CmdParser *cp) {
    if (cp->getParamCount() != 2) {
      Console.fmt("current IMU rate: {:.1f} Hz\n", options.imu_rate);
      return;
    }
    options.imu_rate = atof(cp->getCmdParam(1));
    sensorTask->setRate(1000.0 / options.imu_rate);
    Console.fmt("IMU update rate set to {:.1f} Hz\n", options.imu_rate);
  });
    cmdCallback.addCmd("BR", [](CmdParser *cp) {
    if (cp->getParamCount() != 2) {
      Console.fmt("current background rate: {:.1f} Hz\n", options.background_rate);
      return;
    }
    options.imu_rate = atof(cp->getCmdParam(1));
    backgroundTask->setRate(1000.0 / options.background_rate);
    Console.fmt("IMU update rate set to {:.1f} Hz\n", options.background_rate);
  });
  cmdCallback.addCmd("HPR", [](CmdParser *cp) {
    options.report_hpr = !options.report_hpr;
    Console.fmt("heading/ptch/roll reporting: {}\n", B2S(options.report_hpr));
  });
  cmdCallback.addCmd("QUAT", [](CmdParser *cp) {
    options.report_quat = !options.report_quat;
    Console.fmt("quaternion reporting: {}\n", B2S(options.report_quat));
  });
  cmdCallback.addCmd("RAW", [](CmdParser *cp) {
    options.report_raw = !options.report_raw;
    Console.fmt("raw accel/gyro/mag reporting: {}\n", B2S(options.report_raw));
  });
  cmdCallback.addCmd("GRAV", [](CmdParser *cp) {
    options.report_grav = !options.report_grav;
    Console.fmt("gravity vector reporting: {}\n", B2S(options.report_grav));
  });
  cmdCallback.addCmd("BARO", [](CmdParser *cp) {
    options.report_baro = !options.report_baro;
    Console.fmt("baro sensor reporting: {}\n", B2S(options.report_baro));
  });
  cmdCallback.addCmd("FILTER", [](CmdParser *cp) {
    options.run_filter = !options.run_filter;
    Console.fmt("AHRS filter execution: {}\n", B2S(options.run_filter));
  });
  cmdCallback.addCmd("TELE", [](CmdParser *cp) {
    options.teleplot_viewer = !options.teleplot_viewer;
    Console.fmt("teleplot mode reporting: {}\n", B2S(options.teleplot_viewer));
  });
  cmdCallback.addCmd("APPLY", [](CmdParser *cp) {
    options.apply_cal = !options.apply_cal;
    Console.print(options.apply_cal ? "applying calibration\n"
                                    : "reporting uncalibrated values\n");
  });
  cmdCallback.addCmd("MCAL", [](CmdParser *cp) {
    motion_cal = true;
    Console.fmt("Starting MotionCal mode\n");
  });
  cmdCallback.addCmd("GCAL", [](CmdParser *cp) {
    config.gcal_samples = GYRO_SAMPLES;
    Console.fmt("starting gyro calibration, {} samples..\n",
                config.gcal_samples);
  });

  cmdCallback.addCmd("AHRS", [](CmdParser *cp) {
    if (cp->getParamCount() != 2) {
      Console.fmt("need an int parameter\n");
      return;
    }
    options.ahrs_algo = (ahrs_algo_t)atoi(cp->getCmdParam(1));
    selectAHRS(options, config);
    Console.fmt("AHRS algorithm set to: {}\n", algoName(options.ahrs_algo));
  });
  cmdCallback.addCmd("DEB", [](CmdParser *cp) {
    if (cp->getParamCount() != 2) {
      Console.fmt("debug level: {}\n", options.debug);
      return;
    }
    options.debug = atoi(cp->getCmdParam(1));
    Console.fmt("debug level set to: {}\n", options.debug);
  });
  cmdCallback.addCmd("NED", [](CmdParser *cp) {
    options.ned = !options.ned;
    Console.fmt("North-East-Down adjustment: {}\n", B2S(options.ned));
  });
  cmdCallback.addCmd("TS", [](CmdParser *cp) {
    options.timing_stats = !options.timing_stats;
    Console.fmt("timing statistics: {}\n", B2S(options.timing_stats));
  });
  cmdCallback.addCmd("MU", [](CmdParser *cp) {
    options.memory_usage = !options.memory_usage;
    Console.fmt("memory usage: {}\n", B2S(options.memory_usage));
  });

  cmdCallback.addCmd("PAR", [](CmdParser *cp) {
    Console.fmt("imu={} baro={} ned={} cal={} kf={}\n",
                options.selected_imu_name, baro_types[options.which_baro],
                B2S(options.ned), B2S(options.apply_cal), options.which_kftype);
    Console.fmt("irate={:.1f} rrate={:.1f} filter={}\n", options.imu_rate,
                options.report_rate, B2S(options.run_filter));
    Console.fmt("mDNS tpdest={}:{}\n", config.tpHost.toString().c_str(),
                config.tpPort);
    Console.fmt("IP={} GW={} netmask={} rssi={}\n",
                WiFi.localIP().toString().c_str(),
                WiFi.gatewayIP().toString().c_str(),
                WiFi.subnetMask().toString().c_str(), WiFi.RSSI());
    Console.fmt("flowsensor pin={} IRQs={} enabled={} valid={}\n",
                flow_sensor.pinNum(), flow_sensor.numIrqs(),
                flow_sensor.enabled(), digitalPinIsValid(flow_sensor.pinNum()));
  });
  cmdCallback.addCmd("NDJSON",
                     [](CmdParser *cp) { options.ndjson = !options.ndjson; });
  cmdCallback.addCmd("SAVE", [](CmdParser *cp) {
    Console.fmt("Saving preferences...\n");
    savePrefs(options);
    Console.fmt("done.\n");
  });
  cmdCallback.addCmd("WIPE", [](CmdParser *cp) {
    Console.fmt("factory reset...");
    wipePrefs();
    ESP.restart();
  });
  cmdCallback.addCmd("REBOOT", [](CmdParser *cp) {
    Console.fmt("Rebooting...");
    ESP.restart();
  });
  cmdCallback.addCmd("SG", [](CmdParser *cp) {
    if (cp->getParamCount() == 1) {
      Console.fmt("serial GPS status:");
      Console.fmt("configured for: rx={} tx={} speed={} "
                  "UART{} TinyGPSPlus={}\n",
                  options.gps_rx_pin, options.gps_tx_pin, options.gps_speed,
                  GPS_UART, serialGps.libraryVersion());
      serialGpsStats();
      return;
    }
    // speed rx tx format
    if (cp->getParamCount() == 4) {
      options.gps_rx_pin = atoi(cp->getCmdParam(1));
      options.gps_tx_pin = atoi(cp->getCmdParam(2));
      options.gps_speed = atoi(cp->getCmdParam(3));
      // options.gps_uart = atoi(cp->getCmdParam(4));
      Console.fmt("serial GPS - params set to: rx={} tx={} speed={}\n",
                  options.gps_rx_pin, options.gps_tx_pin, options.gps_speed);
      init_serial_gps(options);
    }
  });

  cmdCallback.addCmd("FS", [](CmdParser *cp) {
    if (cp->getParamCount() != 2) {
      Console.fmt("current flowsensor pin: {}\n", options.flowsensor_pin);
      return;
    }
    options.flowsensor_pin = atoi(cp->getCmdParam(1));
    Console.fmt("flowsensor pin set to:: {}\n", options.flowsensor_pin);
  });
  customCommands(config, options);
  buffer.setEcho(true);
}

void serialwriteFunc(const uint8_t writeChar) { Console.write(writeChar); }

void serialCmdComplete(CmdParser &cmdParser, bool found) {
  if (!found) {
    const char *cmd = cmdParser.getCommand();
    if ((*cmd == '#') || (*cmd == ';')) {
      // a comment. ignore.
      return;
    }
    Console.fmt("command not found: '{}' \n", cmd);
    Console.fmt("type 'help' for a list of commands\n");
  }
}

void testSerial() {
  while (Serial && Serial.available()) {
    uint8_t readChar = Serial.read();
    cmdCallback.updateCmdProcessing(&shell, &buffer, readChar, serialwriteFunc,
                                    serialCmdComplete);
  }
}

void printHelp(options_t &options) { help(options, &shell); }