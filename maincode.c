#include <Wire.h>

#define SDA_PIN  21
#define SCL_PIN  22
#define MPU_ADDR 0x68

#define ACCEL_SENS  16384.0f
#define GYRO_SENS   131.0f

float angle = 0.0f;
float alpha = 0.98f;
unsigned long lastTime = 0;
// PID
float Kp = 20.0f;
float Ki = 0.0f;
float Kd = 0.8f;
float setpoint   = 3.2f;  // change this to your upright angle
float pidError   = 0.0f;
float prevError  = 0.0f;
float integral   = 0.0f;
float pidOutput  = 0.0f;

// Offsets (calculated during calibration at startup)
float gyroXoffset  = 0.0f;
float accelXoffset = 0.0f;

void writeMPU(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}
// Motor A
#define PWMA 25
#define AIN1 26
#define AIN2 27
// Motor B
#define PWMB 14
#define BIN1 33
#define BIN2 32

#define PWM_FREQ  20000
#define PWM_RES   8
#define DEADBAND  30
#define FALL_ANGLE 40.0f

void setMotorA(int spd) {
  if (spd > 0)      { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); }
  else if (spd < 0) { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); spd = -spd; }
  else              { digitalWrite(AIN1, LOW);   digitalWrite(AIN2, LOW); }
  ledcWrite(PWMA, constrain(spd, 0, 255));
}

void setMotorB(int spd) {
  if (spd > 0)      { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); }
  else if (spd < 0) { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); spd = -spd; }
  else              { digitalWrite(BIN1, LOW);   digitalWrite(BIN2, LOW); }
  ledcWrite(PWMB, constrain(spd, 0, 255));
}

void stopMotors() { setMotorA(0); setMotorB(0); }

void driveMotors(int out) {
  // Dead zone — motors don't move for tiny corrections
  if (abs(out) < DEADBAND) {
    stopMotors();
    return;
  }

  // Soft start — scale up from DEADBAND to 255
  // This prevents sudden jerks
  int pwm;
  if (out > 0) {
    pwm = map(out, DEADBAND, 255, 60, 255);
  } else {
    pwm = map(-out, DEADBAND, 255, 60, 255);
  }
  pwm = constrain(pwm, 0, 255);

  if (out > 0) {
    // Lean forward → drive forward to catch
    setMotorA(pwm);
    setMotorB(-pwm);  // B is physically reversed
  } else {
    // Lean backward → drive backward to catch
    setMotorA(-pwm);
    setMotorB(pwm);
  }
}

void readMPU(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // skip temp
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
}

void setup() {
  Serial.begin(115200);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  ledcAttach(PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PWMB, PWM_FREQ, PWM_RES);
  stopMotors();
  delay(2000);

  // Pull-ups BEFORE Wire.begin
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // Wake up MPU
  writeMPU(0x6B, 0x00);
  delay(200);

  // Check if MPU is responding
  Wire.beginTransmission(MPU_ADDR);
  byte error = Wire.endTransmission();
  if (error != 0) {
    Serial.println("MPU6050 NOT FOUND - check wiring!");
    while (1);  // stop here
  }
  Serial.println("MPU6050 FOUND");

  // Calibrate - keep bot still
  Serial.println("Calibrating - keep perfectly still...");
  long sumGX = 0, sumAX = 0;
  int16_t ax, ay, az, gx, gy, gz;
  for (int i = 0; i < 500; i++) {
    readMPU(ax, ay, az, gx, gy, gz);
    sumGX += gx;
    sumAX += ax;
    delay(3);
  }
  gyroXoffset  = sumGX / 500.0f;
  accelXoffset = sumAX / 500.0f;
  Serial.print("Gyro offset: ");  Serial.println(gyroXoffset);
  Serial.print("Accel offset: "); Serial.println(accelXoffset);
  Serial.println("Calibration done - starting filter...");

  // Seed filter with accel angle
  readMPU(ax, ay, az, gx, gy, gz);
  float axf = (ax - accelXoffset) / ACCEL_SENS;
  float azf = az / ACCEL_SENS;
  angle = atan2(axf, azf) * 180.0f / PI;

  lastTime = micros();
}

void loop() {
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0f;
  lastTime = now;
  if (dt <= 0 || dt > 0.1f) dt = 0.01f;

  int16_t ax, ay, az, gx, gy, gz;
  readMPU(ax, ay, az, gx, gy, gz);

  float axf      = (ax - accelXoffset) / ACCEL_SENS;
  float azf      = az / ACCEL_SENS;
  float gyroRate = (gx - gyroXoffset) / GYRO_SENS;

  float accelAngle = atan2(axf, azf) * 180.0f / PI;
  angle = alpha * (angle + gyroRate * dt) + (1.0f - alpha) * accelAngle;

  // Safety
  if (fabs(angle) > FALL_ANGLE) {
    stopMotors();
    integral = 0.0f;
    Serial.print("FALLEN | angle: "); Serial.println(angle);
    delay(100);
    return;
  }

  // PID
  pidError   = angle - setpoint;
  integral  += pidError * dt;
  integral   = constrain(integral, -200.0f, 200.0f);
  float derivative = (pidError - prevError) / dt;
  prevError  = pidError;
  pidOutput  = Kp*pidError + Ki*integral + Kd*derivative;
  pidOutput  = constrain(pidOutput, -255.0f, 255.0f);

  driveMotors((int)pidOutput);

  Serial.print("Angle: ");  Serial.print(angle, 2);
  Serial.print(" | PID: "); Serial.print(pidOutput, 1);
  Serial.print(" | err: "); Serial.println(pidError, 2);

  delay(5); // 200Hz
}
