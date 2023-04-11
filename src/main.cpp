#include <M5Unified.h>

#include "Adafruit_BMP3XX.h"
#include "dps3xx.h"
#include "i2cscanner.h"
#include "util.h"
#include <ArduinoJson.h>

#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_LPS2X.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define INTERVAL 500 // msec
#define JSON_SIZE 4096

#define BMP3XX_ALTERNATE_ADDRESS (0x76)
#define LPS2X_ALTERNATE_ADDRESS (0x5c)

StaticJsonDocument<JSON_SIZE> doc;

Adafruit_FXOS8700 accelmag_fxos = Adafruit_FXOS8700(0x8700A, 0x8700B);
Adafruit_FXAS21002C gyro_fxas = Adafruit_FXAS21002C(0x0021002C);
Adafruit_BMP3XX bmp;
Adafruit_LPS22 lps;
Adafruit_Sensor *lps_temp, *lps_pressure;

bool dps368_avail = false;
bool bmp390_avail = false;
bool lps22_avail = false;
bool fxos8700_avail = false;
bool fxas21002_avail = false;
bool imu_intern_avail = false;

bool pretty = true;
int interval = INTERVAL;
void displaySensorDetails(void);

void setup(void) {

  auto cfg = M5.config();
  cfg.serial_baudrate =
      115200; // default=115200. if "Serial" is not needed, set it to 0.
  cfg.internal_imu = true;  // default=true. use internal IMU.
  cfg.external_imu = true;  // default=false. use Unit Accel & Gyro.
  cfg.led_brightness = 128; // default= 0. system LED brightness (0=off /
                            // 255=max) (※ not NeoPixel)
  M5.begin(cfg);

  /* Wait for the Serial Monitor */
  while (!Serial) {
    delay(1);
  }

  i2cScanner(&Wire, "Wire");
  i2cScanner(&Wire1, "Wire1");
  Serial.println();

#if 1
  // try the internal IMU
  const char *name = "";
  if (imu_intern_avail = M5.Imu.begin()) {
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
    default:
      name = "none";
      break;
    }
  };
  Serial.printf("Internal IMU %s %sfound\n", name,
                imu_intern_avail ? "" : "not ");
#endif
  // try red-port connected sensors before internally wired sensors
  // so they take precedence

  // dps368 MUST be at alternate i2c address - collision with bmp3xx
  dps368_avail = dps368_i2c_init(Wire1, DPS368_ALTERNATE_ADDRESS) ||
                 dps368_i2c_init(Wire, DPS368_ALTERNATE_ADDRESS);
  Serial.printf("DPS368 %sfound\n", dps368_avail ? "" : "not ");

  // bmp390 MUST be at default address 0x77 - collision with dps368
  bmp390_avail = bmp.begin_I2C(BMP3XX_DEFAULT_ADDRESS, &Wire1) ||
                 bmp.begin_I2C(BMP3XX_DEFAULT_ADDRESS, &Wire);
  Serial.printf("BMP3XX %sfound\n", bmp390_avail ? "" : "not ");

  // try red port first, then internal, any address
  lps22_avail = lps.begin_I2C(LPS2X_I2CADDR_DEFAULT, &Wire1) ||
                lps.begin_I2C(LPS2X_ALTERNATE_ADDRESS, &Wire1) ||
                lps.begin_I2C(LPS2X_I2CADDR_DEFAULT, &Wire) ||
                lps.begin_I2C(LPS2X_ALTERNATE_ADDRESS, &Wire);
  Serial.printf("LPS22 %sfound\n", lps22_avail ? "" : "not ");
  if (lps22_avail) {
    lps_temp = lps.getTemperatureSensor();
    lps_pressure = lps.getPressureSensor();
  }

  fxas21002_avail =
      gyro_fxas.begin(0x21, &Wire1) || gyro_fxas.begin(0x21, &Wire);
  Serial.printf("FXAS21002C %sfound\n", fxas21002_avail ? "" : "not ");
  if (fxas21002_avail) {
    /* Set gyro range. (optional, default is 250 dps) */
    // gyro_fxas.setRange(GYRO_RANGE_2000DPS);
  }

  fxos8700_avail =
      accelmag_fxos.begin(0x1F, &Wire1) || accelmag_fxos.begin(0x1F, &Wire);
  Serial.printf("FXOS8700 %sfound\n", fxos8700_avail ? "" : "not ");
  if (fxos8700_avail) {
    /* Set accelerometer range (optional, default is 2G) */
    // accelmag_fxos.setAccelRange(ACCEL_RANGE_8G);

    /* Set the sensor mode (optional, default is hybrid mode) */
    // accelmag_fxos.setSensorMode(ACCEL_ONLY_MODE);

    /* Set the magnetometer's oversampling ratio (optional, default is 7) */
    // accelmag_fxos.setMagOversamplingRatio(MAG_OSR_7);

    /* Set the output data rate (optional, default is 100Hz) */
    accelmag_fxos.setOutputDataRate(ODR_100HZ);
  }
  displaySensorDetails();
}

void loop(void) {

  if (fxos8700_avail) {
    sensors_event_t aevent, mevent;

    accelmag_fxos.getEvent(&aevent, &mevent);
    uint32_t now = micros();
    JsonObject j = doc.createNestedObject("accel");
    j["t"] = now;
    j["x"] = aevent.acceleration.x;
    j["y"] = aevent.acceleration.y;
    j["z"] = aevent.acceleration.z;

    JsonObject m = doc.createNestedObject("mag");
    j["t"] = now;
    m["x"] = mevent.magnetic.x;
    m["y"] = mevent.magnetic.y;
    m["z"] = mevent.magnetic.z;
  }

  if (fxas21002_avail) {
    sensors_event_t gevent;
    uint32_t now = micros();
    gyro_fxas.getEvent(&gevent);
    JsonObject j = doc.createNestedObject("gyro");
    j["t"] = now;
    j["x"] = gevent.gyro.x;
    j["y"] = gevent.gyro.y;
    j["z"] = gevent.gyro.z;
  }

  if (M5.Imu.isEnabled()) {
    float ax, ay, az;
    float x, y, z;
    uint32_t now = micros();
    M5.Imu.getAccel(&ax, &ay, &az);
    M5.Imu.getGyro(&x, &y, &z);
    JsonObject a = doc.createNestedObject("accel");
    a["t"] = now;
    a["x"] = ax;
    a["y"] = ay;
    a["z"] = az;
    JsonObject j = doc.createNestedObject("gyro");
    j["t"] = now;
    j["x"] = x;
    j["y"] = y;
    j["z"] = z;
  }

  if (bmp390_avail && bmp.performReading()) {
    uint32_t now = micros();

    float hPa = bmp.pressure / 100.0;
    float meter = Pascal2meters(bmp.pressure, SEALEVELPRESSURE_HPA);
    JsonObject j = doc.createNestedObject("bmp3xx");
    j["t"] = now;
    j["hpa"] = hPa;
    j["alt"] = meter;
  }

  if (lps22_avail) {
    uint32_t now = micros();
    sensors_event_t lpsp;
    lps_pressure->getEvent(&lpsp);
    float meter = Pascal2meters(lpsp.pressure, SEALEVELPRESSURE_HPA);
    JsonObject j = doc.createNestedObject("lps2x");
    j["t"] = now;
    j["hpa"] = lpsp.pressure;
    j["alt"] = meter;
  }

  if (dps368_avail) {
    dps368_i2c_update(doc, false, true);
  }

  if (measureJson(doc)) {
    if (pretty) {
      serializeJsonPretty(doc, Serial);

    } else {
      serializeJson(doc, Serial);
    }
  }
  Serial.print("\n");
  doc.clear();

  // delay(1000);
  vTaskDelay(interval / portTICK_RATE_MS);
}

void displaySensorDetails(void) {
  if (fxos8700_avail) {

    sensor_t accel, mag;
    accelmag_fxos.getSensor(&accel, &mag);
    Serial.println("------------------------------------");
    Serial.println("ACCELEROMETER");
    Serial.println("------------------------------------");
    Serial.print("Sensor:       ");
    Serial.println(accel.name);
    Serial.print("Driver Ver:   ");
    Serial.println(accel.version);
    Serial.print("Unique ID:    0x");
    Serial.println(accel.sensor_id, HEX);
    Serial.print("Min Delay:    ");
    Serial.print(accel.min_delay);
    Serial.println(" s");
    Serial.print("Max Value:    ");
    Serial.print(accel.max_value, 4);
    Serial.println(" m/s^2");
    Serial.print("Min Value:    ");
    Serial.print(accel.min_value, 4);
    Serial.println(" m/s^2");
    Serial.print("Resolution:   ");
    Serial.print(accel.resolution, 8);
    Serial.println(" m/s^2");
    Serial.println("------------------------------------");
    Serial.println("");
    Serial.println("------------------------------------");
    Serial.println("MAGNETOMETER");
    Serial.println("------------------------------------");
    Serial.print("Sensor:       ");
    Serial.println(mag.name);
    Serial.print("Driver Ver:   ");
    Serial.println(mag.version);
    Serial.print("Unique ID:    0x");
    Serial.println(mag.sensor_id, HEX);
    Serial.print("Min Delay:    ");
    Serial.print(accel.min_delay);
    Serial.println(" s");
    Serial.print("Max Value:    ");
    Serial.print(mag.max_value);
    Serial.println(" uT");
    Serial.print("Min Value:    ");
    Serial.print(mag.min_value);
    Serial.println(" uT");
    Serial.print("Resolution:   ");
    Serial.print(mag.resolution);
    Serial.println(" uT");
    Serial.println("------------------------------------");
    Serial.println("");
  }
  if (fxas21002_avail) {
    sensor_t sensor;
    gyro_fxas.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print("Sensor:       ");
    Serial.println(sensor.name);
    Serial.print("Driver Ver:   ");
    Serial.println(sensor.version);
    Serial.print("Unique ID:    0x");
    Serial.println(sensor.sensor_id, HEX);
    Serial.print("Max Value:    ");
    Serial.print(sensor.max_value);
    Serial.println(" rad/s");
    Serial.print("Min Value:    ");
    Serial.print(sensor.min_value);
    Serial.println(" rad/s");
    Serial.print("Resolution:   ");
    Serial.print(sensor.resolution);
    Serial.println(" rad/s");
    Serial.println("------------------------------------");
    Serial.println("");
  }

  if (lps22_avail) {
    lps_temp->printSensorDetails();
    lps_pressure->printSensorDetails();
  }
}

//     Serial.printf("BMP390 hPa=%f meter=%f\n", hPa / 100.0, meter);

// /* Display the accel results (acceleration is measured in m/s^2) */
// Serial.print("A ");
// Serial.print("X: ");
// Serial.print(aevent.acceleration.x, 4);
// Serial.print("  ");
// Serial.print("Y: ");
// Serial.print(aevent.acceleration.y, 4);
// Serial.print("  ");
// Serial.print("Z: ");
// Serial.print(aevent.acceleration.z, 4);
// Serial.print("  ");
// Serial.println("m/s^2");

// /* Display the mag results (mag data is in uTesla) */
// Serial.print("M ");
// Serial.print("X: ");
// Serial.print(mevent.magnetic.x, 1);
// Serial.print("  ");
// Serial.print("Y: ");
// Serial.print(mevent.magnetic.y, 1);
// Serial.print("  ");
// Serial.print("Z: ");
// Serial.print(mevent.magnetic.z, 1);
// Serial.print("  ");
// Serial.println("uT");

// /* Display the results (speed is measured in rad/s) */
// Serial.print("X: ");
// Serial.print(gevent.gyro.x);
// Serial.print("  ");
// Serial.print("Y: ");
// Serial.print(gevent.gyro.y);
// Serial.print("  ");
// Serial.print("Z: ");
// Serial.print(gevent.gyro.z);
// Serial.print("  ");
// Serial.println("rad/s ");

// Serial.println("");