#include <Wire.h>
#include <math.h>
#include <BMI160Gen.h>
#include <Kalman.h>
#include <MechaQMC5883.h>

#define BAUDRATE  115200
#define SENSE_RATE   100
#define GYRO_RANGE   250
#define ACCL_RANGE     2

#define SENITIVITY_2G 120.0 // 120.0 LSB / MicroTesla == 12000.0 LSB / Gauss
#define SENITIVITY_8G 30.0

#define ANKARA_DECLINATION 1.18333333333333333333333333 // east is positive

MechaQMC5883 qmc;
const float hard_iron[3] = {
  2.379999999999999, 16.98, 44.3
};

#define deg_to_rad(a) (a/180*M_PI)
#define rad_to_deg(a) (a/M_PI*180)

Kalman kalmanRoll;
Kalman kalmanPitch;

float azimuth = 0, old_azimuth;
float mag_x_hor = 0, mag_y_hor = 0;

float low_pass_filter(float old_val, float new_val) {
  return old_val * 0.8 + new_val * 0.2;
}

float convertRawGyro(int gRaw) {
  // ex) if the range is +/-500 deg/s: +/-32768/500 = +/-65.536 LSB/(deg/s)
  float lsb_omega = float(0x7FFF) / GYRO_RANGE;
  return gRaw / lsb_omega;  // deg/sec
}

float convertRawAccel(int aRaw) {
  // ex) if the range is +/-2g ; +/-32768/2 = +/-16384 LSB/g
  float lsb_g = float(0x7FFF) / ACCL_RANGE;
  return aRaw / lsb_g;
}

static float gyro_roll = 0, gyro_pitch = 0, gyro_yaw = 0;
static float comp_roll = 0, comp_pitch = 0;
static unsigned long last_mills = 0; 

void print_roll_pitch() 
{
  
  // read raw accl measurements from device
  int rawXAcc, rawYAcc, rawZAcc; // x, y, z
  BMI160.readAccelerometer(rawXAcc, rawYAcc, rawZAcc);
  float accX = convertRawAccel(rawXAcc);
  float accY = convertRawAccel(rawYAcc);
  float accZ = convertRawAccel(rawZAcc);

  float rad_a_roll = atan2(accY, accZ);
  float rad_a_pitch = atan2(-accX, sqrt(accY*accY + accZ*accZ));

  float accl_roll = rad_to_deg(rad_a_roll);
  float accl_pitch = rad_to_deg(rad_a_pitch);

  int rawRoll, rawPitch, rawYaw;
  BMI160.readGyro(rawRoll, rawPitch, rawYaw);
  float omega_roll  = convertRawGyro(rawRoll);
  float omega_pitch = convertRawGyro(rawPitch);
  float omega_yaw   = convertRawGyro(rawYaw);
  
  unsigned long cur_mills = micros();
  unsigned long duration = cur_mills - last_mills;
  last_mills = cur_mills;
  double dt = duration / 1000000.0; // us->s
  if (dt > 0.1) return;

  gyro_roll  += omega_roll  * dt; 
  gyro_pitch += omega_pitch * dt;
  gyro_yaw   += omega_yaw   * dt;
  
  float kalm_roll  = kalmanRoll.getAngle(accl_roll, omega_roll, dt);
  float kalm_pitch = kalmanPitch.getAngle(accl_pitch, omega_pitch, dt);

  int16_t raw_x = 0, raw_y = 0, raw_z = 0;

  Wire.beginTransmission(0x0D);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.requestFrom(0x0D, 6);
  raw_x = Wire.read(); // LSB  x
  raw_x |= Wire.read() << 8; // MSB  x
  raw_y = Wire.read(); // LSB  z
  raw_y |= Wire.read() << 8; // MSB z
  raw_z = Wire.read(); // LSB y
  raw_z |= Wire.read() << 8; // MSB y

  float x = raw_x / SENITIVITY_2G;
  float y = raw_y / SENITIVITY_2G;
  float z = raw_z / SENITIVITY_2G;

  float mag_data[3] = {x, y, z};
  for (uint8_t i = 0 ; i < 3 ; i++) {
    mag_data[i] = mag_data[i] - hard_iron[i];
  }

  float mag_pitch = kalm_roll * DEG_TO_RAD;
  float mag_roll = -kalm_pitch * DEG_TO_RAD;

  mag_x_hor = mag_data[0] * cos(mag_pitch) + mag_data[1] * sin(mag_roll) * sin(mag_pitch) - mag_data[2] * cos(mag_roll) * sin(mag_pitch);
  mag_y_hor = mag_data[1] * cos(mag_roll) + mag_data[2] * sin(mag_roll);

  old_azimuth = azimuth;
  azimuth = rad_to_deg(atan2(mag_x_hor, mag_y_hor));

  azimuth += ANKARA_DECLINATION + 90.0;
  azimuth = azimuth < 0 ? 360 + azimuth : azimuth;

  azimuth = low_pass_filter(old_azimuth, azimuth);

  static int n = 0;
  if (n != 50) {
    ++n; return;
  }
  n = 0;

  Serial.print(kalm_roll);
  Serial.print("/");
  Serial.print(kalm_pitch);
  Serial.print("/");
  Serial.print(azimuth);

  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while(!Serial) {}
  
  if (!Wire.begin(21, 22, 400000)) {
    Serial.println("I2C couldn't be initialized!");
    while(false);
  }

  uint8_t qmc_sense_rate;
  switch (SENSE_RATE)
  {
  case 200:
    qmc_sense_rate = ODR_200Hz;
    break;
  case 100:
    qmc_sense_rate = ODR_100Hz;
    break;
  case 50:
    qmc_sense_rate = ODR_50Hz;
    break;
  case 10:
    qmc_sense_rate = ODR_10Hz;
    break;
  
  default:
    qmc_sense_rate = ODR_100Hz;
    Serial.println("ERROR: The sense rate chosen is not suppored in QMC5883!");
    break;
  }

  qmc.init();
  qmc.setMode(Mode_Continuous, ODR_100Hz, RNG_2G, OSR_512);

  if (!BMI160.begin(BMI160GenClass::I2C_MODE, Wire, 0x69)) {
    Serial.println("Failed to initialize BMI160!");
    while(true) {}
  }

  BMI160.setGyroRate(SENSE_RATE);
  BMI160.setAccelerometerRate(SENSE_RATE);
  BMI160.setGyroRange(GYRO_RANGE);
  BMI160.setAccelerometerRange(ACCL_RANGE);

  BMI160.autoCalibrateGyroOffset();
  BMI160.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  BMI160.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  BMI160.autoCalibrateAccelerometerOffset(Z_AXIS, 1); // Meaning that the Z axis should be pointing to the ground

  delay(100);

  int rawXAcc, rawYAcc, rawZAcc;
  BMI160.readAccelerometer(rawXAcc, rawYAcc, rawZAcc);
  float accX = convertRawAccel(rawXAcc);
  float accY = convertRawAccel(rawYAcc);
  float accZ = convertRawAccel(rawZAcc);
  
  float roll  = rad_to_deg(atan(accY / accZ));
  float pitch = rad_to_deg(atan(-accX / sqrt(accY * accY + accZ * accZ)));

  kalmanRoll.setAngle(roll);
  kalmanPitch.setAngle(pitch);
  gyro_roll = roll;
  gyro_pitch = pitch;

}

void loop() {
  print_roll_pitch();

}
