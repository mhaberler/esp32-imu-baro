# IMU and baro sensor reporting for ESP32 platforms

tested on: M5Stack Core2 with M5Unified,  ESP-WROVER-KIT V4.1 
should work on any esp32 

baro pressure sensors connected to I2C 0 (Wire) or I2C 1 (Wire1) as available per platform external I2C port will be autodetected

the IMU type needs to be set manually.

## screenshot using plotting via [Teleplot](https://teleplot.fr)
<img src="docs/teleplot3d.png"/>

## supported IMUs + magnetometers:
- NXP FXOS8700/FXAS21002C
- MPU6886
- MPU9250
- MPU6050
- BMI270/BMM150
- ICM20948
- BNO08x

## supported baro sensors:
- LPS22
- DPS310
- DPS368
- BMP388/390

## supported GPS:
- serial generic NMEA gps on Serial1, using [TinyGPSPlus](https://github.com/mikalhart/TinyGPSPlus)

## reporting format
- JSON - per-line (NDJSON http://ndjson.org/) or pretty printed 
- support for Teleplot-format output of sensors and the AHRS quaternion

## command line
connect to the console, type `help` to see available commands:
````
help
Type your commands. Supported: 
  help
  ssid <ssid1> [ssid2] ...# set WiFi SSID's to connect to, currently 5:
      ... ssid's ...
      ... some password...
  pass <pass1> [pass2] ...# set corresponding WiFi passwords
  tpdest <IP address> <port number> # teleplot UDP destination: :-1
  sd [service [proto]] # mDNS discovery, defaullt: teleplot udp
  ss <index> # set teleplot destination to mDNS scan result by index
  i2c  # run i2c scan
  fs [pin] # report or set the flowsensor pin: 36
  dev <device> # set IMU device: 1 (FXOS8700/FXAS21002C)
  use <int> # select baro sensor as KF input (0=none, 1=lps22, 2=dps3xx, 3=bmp3xx)?) :2 .. dps3xx
  report <freq> # set reporting rate (HZ): 20.0
  imu  <freq> # set imu sampling rate (HZ): 100.0
  baro # toggle 'report barometer sensors' flag: true
  hpr # toggle 'report heading/pitch/roll' flag: true
  quat # toggle 'report quaternions' flag: false
  raw # toggle 'report raw sensor values' flag: false
  filter # toggle AHRS filter execution: true
  grav # toggle 'report gravity vector' flag: false
  ned # toggle NED translation before AHRS' flag: false
  ts # toggle timing statistics flag: false
  mu # toggle memory usage flag: false
  apply # toggle 'apply calibration' flag: false
  tele # toggle 'generate output for teleplot' flag: true
  mcal # enter MotionCal compass calibration
  gcal # run gyro calibration and store offsets
  par # show key params
  sg # show serial GPS status
 values' flag: false
  filter # toggns, speed - save and reboot
  ahrs <integer> # select AHRS algorithm: 2
     0 # NXP Fusion
     1 # Madgewick
     2 # Mahoney
  save
  wipe  # erase preferences and set to default (factory reset)
  ndjson # toggle NDJSON printing: true
  reboot
  alpha # set smoothing alpha: 0.8
currently active sensors: dps3xx bmp3xx lps22 flowsensor a=FXOS8700 g=FXAS21002C m=FXOS8700
supported devices:
        0 .. no device set
        1 .. FXOS8700/FXAS21002C
        2 .. MPU6886
        3 .. MPU9250
        4 .. MPU6050
        5 .. BMI270/BMM150
        6 .. ICM20948
        7 .. BNO08x
accel offsets for zero-g, in m/s^2: 0.0000 0.0000 0.0000 
gyro offsets for zero-rate, in deg/s: 0.3281 -0.0937 0.4375 
mag offsets for hard iron calibration (in uT): 0.0000 0.0000 0.0000 
mag field magnitude (in uT): 50.0000
````

Save parameters with `save`.

New parameters are applied only after a `reboot`, so when done setting up, type
````
save
reboot
````

## setup example for M5Stack Core2 - teleplot format

assumes the sensor is the builtin MPU6886. No pressure sensor.

````
;
; to clear all old settings, use 'wipe' - device will automatically reboot
; then continue below:
;
;
; setup wifi SSID
ssid SSID
;
; wifi password
pass PASSWORT
;
; set teleplot destination (UDP)
; adjust to UDP port number given to you by teleplot.fr:
; tpdest teleplot.fr <PORT>
tpdest teleplot.fr FIXME
;
; turn on teleplot reporting
tele
;
; select the IMU device
; supported devices:
;         0 .. no device set
;         1 .. FXOS8700/FXAS21002C
;         2 .. MPU6886
;         3 .. MPU9250
;         4 .. MPU6050
;         5 .. BMI270/BMM150
;         6 .. ICM20948
;         7 .. BNO08x
; use the builtin MPU6886
dev 2
;
; select the baro sensor
;   use <int> # select baro sensor as KF input (0=none, 1=lps22, 2=dps3xx, 3=bmp3xx)?) :2 .. dps3xx
; use 2
use 0
;
; set reporting rate
report 20
;
; set IMU rate
imu 100
;
; save the setup
save
;
; restart so new settings become active
reboot
````
Then open the console port in https://teleplot.fr


## setup example for generic ESP32, IMU ICM20948, and baro sensors - teleplot format

````
dev 6
tele
baro
report 20
save
reboot
````
Then open the console port in https://teleplot.fr

## setup example - reporting in JSON
once per second:
````
baro
hpr
quat
raw
grav
report 1
save
reboot
````
## the `ned` flag

Setting the `ned` flag will translate accel and gyro values from sensor axes to
AHRS NED (north-east-down) right handed coordinate frame.

See the [example code here](https://github.com/har-in-air/ESP32_IMU_BARO_GPS_VARIO/blob/dca3b7298c764fcdb2100c89898cae44a715a2eb/src/main.cpp#L312-L314) .

# Calibration
````
dev 6
mcal
report 20
save
reboot
````

## Build

````
git clone  https://github.com/mhaberler/esp32-imu-baro.git
cd esp32-imu-baro/
git submodule update --init --recursive

````
Choose target and build.

## plans:
- add calibration

## supported baro sensors: LPS22, DPS310, DPS36x,BMP3xx

any BMP3xx must be configured for the default I2C address 0x77
any DPS3xx or DPS310 must be configured for the alternate I2C address 0x76 
for the DPS310 a custom version with -DTEST_DPS310 must be built as autodetection fails

# using teleplot

recommending to use the https://teleplot.fr web service - the web service supports 3D display of attitude, whereas the [Teleplot VScode extension](https://marketplace.visualstudio.com/items?itemName=alexnesnes.teleplot) plugin does not


# timing pins 
I have a [Core2 Bottom2](https://docs.m5stack.com/en/base/m5go_bottom2) which brings out two GPIO pins on the HY2.0-4P-PortB (blue): GPIO13	and GPIO14 (normally UART2 which we dont use)

by defining the symbols -DIMU_PIN=13 and -DREPORT_PIN=14 code will be compiled in to toggle those pins during the IMU and reporting handlers. That signal can be used for measuring timing.

if available, a third pin EXTRA_PIN can be used to time the AHRS step.

Measurement showed that at I2C clock 100kHz handleImu() needs close to 10mS which suggests about 90Hz upper IMU rate (ICM20948). Setting the I2C clock to 400kHz brings this down to about 4.3mS with a 240Mhz esp32.

handleImu() takes about 1.3mS for handleImu() (AHRS=Mahoney),  about 1.46mS (AHRS=Madgewick) and about 1.8mS (AHRS=NXP Fusion) with the NXP FXOS8700/FXAS21002C IMU.

The AHRS step is about 500uS (Mahoney).

# open issues

- timestamps are in a state of flix...
- ideally this UTC time would from a GPS or NTP

