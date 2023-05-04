#include "defs.hpp"

void receiveCalibration(const options_t &options, config_t &config);

byte caldata[68]; // buffer to receive magnetic calibration data
byte calcount = 0;
int loopcount = 0;

uint16_t crc16_update(uint16_t crc, uint8_t a) {
  int i;
  crc ^= a;
  for (i = 0; i < 8; i++) {
    if (crc & 1) {
      crc = (crc >> 1) ^ 0xA001;
    } else {
      crc = (crc >> 1);
    }
  }
  return crc;
}
void MotionCal(sensor_state_t &state, const options_t &options,
               config_t &config) {
  // 'Raw' values to match expectation of MotionCal
  Console.print("Raw:");
  Console.print(int(state.accel.acceleration.x * 8192 / 9.8));
  Console.print(",");
  Console.print(int(state.accel.acceleration.y * 8192 / 9.8));
  Console.print(",");
  Console.print(int(state.accel.acceleration.z * 8192 / 9.8));
  Console.print(",");
  Console.print(int(state.gyro.gyro.x * 16));
  Console.print(",");
  Console.print(int(state.gyro.gyro.y * 16));
  Console.print(",");
  Console.print(int(state.gyro.gyro.z * 16));
  Console.print(",");
  Console.print(int(state.mag.magnetic.x * 10));
  Console.print(",");
  Console.print(int(state.mag.magnetic.y * 10));
  Console.print(",");
  Console.print(int(state.mag.magnetic.z * 10));
  Console.println("");

  // unified data
  Console.print("Uni:");
  Console.print(state.accel.acceleration.x);
  Console.print(",");
  Console.print(state.accel.acceleration.y);
  Console.print(",");
  Console.print(state.accel.acceleration.z);
  Console.print(",");
  Console.print(state.gyro.gyro.x, 4);
  Console.print(",");
  Console.print(state.gyro.gyro.y, 4);
  Console.print(",");
  Console.print(state.gyro.gyro.z, 4);
  Console.print(",");
  Console.print(state.mag.magnetic.x);
  Console.print(",");
  Console.print(state.mag.magnetic.y);
  Console.print(",");
  Console.print(state.mag.magnetic.z);
  Console.println("");
  loopcount++;
  receiveCalibration(options, config);
  // occasionally print calibration
  if (loopcount == 50 || loopcount > 100) {
    Console.print("Cal1:");
    for (int i = 0; i < 3; i++) {
      Console.print(config.cal.accel_zerog[i], 3);
      Console.print(",");
    }
    for (int i = 0; i < 3; i++) {
      Console.print(config.cal.gyro_zerorate[i], 3);
      Console.print(",");
    }
    for (int i = 0; i < 3; i++) {
      Console.print(config.cal.mag_hardiron[i], 3);
      Console.print(",");
    }
    Console.println(config.cal.mag_field, 3);
    loopcount++;
  }
  if (loopcount >= 100) {
    Console.print("Cal2:");
    for (int i = 0; i < 9; i++) {
      Console.print(config.cal.mag_softiron[i], 4);
      if (i < 8)
        Console.print(',');
    }
    Console.println();
    loopcount = 0;
  }
  return;
}

void receiveCalibration(const options_t &options, config_t &config) {
  uint16_t crc;
  byte b, i;

  while (Serial.available()) {
    b = Serial.read();
    if (calcount == 0 && b != 117) {
      // first byte must be 117
      return;
    }
    if (calcount == 1 && b != 84) {
      // second byte must be 84
      calcount = 0;
      return;
    }
    // store this byte
    caldata[calcount++] = b;
    if (calcount < 68) {
      // full calibration message is 68 bytes
      return;
    }
    // verify the crc16 check
    crc = 0xFFFF;
    for (i = 0; i < 68; i++) {
      crc = crc16_update(crc, caldata[i]);
    }
    if (crc == 0) {
      // data looks good, use it
      float offsets[16];
      memcpy(offsets, caldata + 2, 16 * 4);
      config.cal.accel_zerog[0] = offsets[0];
      config.cal.accel_zerog[1] = offsets[1];
      config.cal.accel_zerog[2] = offsets[2];

      config.cal.gyro_zerorate[0] = offsets[3];
      config.cal.gyro_zerorate[1] = offsets[4];
      config.cal.gyro_zerorate[2] = offsets[5];

      config.cal.mag_hardiron[0] = offsets[6];
      config.cal.mag_hardiron[1] = offsets[7];
      config.cal.mag_hardiron[2] = offsets[8];

      config.cal.mag_field = offsets[9];

      config.cal.mag_softiron[0] = offsets[10];
      config.cal.mag_softiron[1] = offsets[13];
      config.cal.mag_softiron[2] = offsets[14];
      config.cal.mag_softiron[3] = offsets[13];
      config.cal.mag_softiron[4] = offsets[11];
      config.cal.mag_softiron[5] = offsets[15];
      config.cal.mag_softiron[6] = offsets[14];
      config.cal.mag_softiron[7] = offsets[15];
      config.cal.mag_softiron[8] = offsets[12];

      if (!config.cal.saveCalibration()) {
        Console.println("**WARNING** Couldn't save calibration");
      } else {
        Console.println("Wrote calibration");
      }
      config.cal.printSavedCalibration();
      calcount = 0;

      ESP.restart();

      return;
    }
    // look for the 117,84 in the data, before discarding
    for (i = 2; i < 67; i++) {
      if (caldata[i] == 117 && caldata[i + 1] == 84) {
        // found possible start within data
        calcount = 68 - i;
        memmove(caldata, caldata + i, calcount);
        return;
      }
    }
    // look for 117 in last byte
    if (caldata[67] == 117) {
      caldata[0] = 117;
      calcount = 1;
    } else {
      calcount = 0;
    }
  }
}
