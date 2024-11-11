#include <Wire.h>
#include <Arduino.h>
const int MPU = 0x68; // MPU6050 I2C address
int16_t tempAccX, tempAccY, tempAccZ;
float AccX, AccY, AccZ;
int16_t tempGyroX, tempGyroY, tempGyroZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
unsigned long elapsedTime, currentTime, previousTime;
const int numCalibrationSamples = 500;
const float alpha = 0.95; // Complementary filter constant

const float Q = 1;
const float R = 1;
float GyroX_est, GyroY_est, GyroZ_est;
float PX_est, PY_est, PZ_est;

void readAcc(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU, (uint8_t)6, (uint8_t)true);
  tempAccX = (Wire.read() << 8 | Wire.read());
  tempAccY = (Wire.read() << 8 | Wire.read());
  tempAccZ = (Wire.read() << 8 | Wire.read());

  AccX = ((float)tempAccX) / 16384.0;
  AccY = ((float)tempAccY) / 16384.0;
  AccZ = ((float)tempAccZ) / 16384.0;

  accAngleX = (atan2(AccY, sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX;
  accAngleY = (atan2(-AccX, sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + AccErrorY;
};

void readGyro(){
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU, (uint8_t)6, (uint8_t)true);
  tempGyroX = (Wire.read() << 8 | Wire.read());
  tempGyroY = (Wire.read() << 8 | Wire.read());
  tempGyroZ = (Wire.read() << 8 | Wire.read());

  GyroX = ((float)tempGyroX) / 131.0;
  GyroY = ((float)tempGyroY) / 131.0;
  GyroZ = ((float)tempGyroZ) / 131.0;

  // add the gyro error to the gyro values
  GyroX = GyroX - GyroErrorX;
  GyroY = GyroY - GyroErrorY;
  GyroZ = GyroZ - GyroErrorZ;

  // kalman filter
  P_est += Q; // Prediksi error dari estimasi

  // Update
  float K = P_est / (P_est + R);
  GyroX_est = GyroX_est + K * (GyroX - GyroX_est);
  GyroY_est = GyroY_est + K * (GyroY - GyroY_est);
  GyroZ_est = GyroZ_est + K * (GyroZ - GyroZ_est);
  P_est = (1 - K) * P_est;

  GyroX = GyroX_est;
  GyroY = GyroY_est;
  GyroZ = GyroZ_est;
};

void setup(){
  Serial.begin(19200);
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  for(int i = 0; i < numCalibrationSamples; i++){
    // Calculate gyro error
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU, (uint8_t)6, (uint8_t)true);
    tempGyroX = (Wire.read() << 8 | Wire.read());
    tempGyroY = (Wire.read() << 8 | Wire.read());
    tempGyroZ = (Wire.read() << 8 | Wire.read());

    GyroErrorX = GyroErrorX + ((float)tempGyroX / 131.0);
    GyroErrorY = GyroErrorY + ((float)tempGyroY / 131.0);
    GyroErrorZ = GyroErrorZ + ((float)tempGyroZ / 131.0);
  }
  GyroErrorX = GyroErrorX / numCalibrationSamples;
  GyroErrorY = GyroErrorY / numCalibrationSamples;
  GyroErrorZ = GyroErrorZ / numCalibrationSamples;

  tempGyroX = (Wire.read() << 8 | Wire.read());
  tempGyroY = (Wire.read() << 8 | Wire.read());
  tempGyroZ = (Wire.read() << 8 | Wire.read());

  GyroX_est = ((float)tempGyroX) / 131.0;
  GyroY_est = ((float)tempGyroY) / 131.0;
  GyroZ_est = ((float)tempGyroZ) / 131.0;

  Serial.print("Gyro Error X: ");
  Serial.print(GyroErrorX);
  Serial.print("\t");
  Serial.print("Gyro Error Y: ");
  Serial.print(GyroErrorY);
  Serial.print("\t");
  Serial.print("Gyro Error Z: ");
  Serial.print(GyroErrorZ);
  Serial.println("\t");
  delay(20);
}

void loop(){
  // Calculate Milliseconds
  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime);

  // Read Accelerometer Data
  readAcc();

  // read Gyro
  readGyro();

  // Calculate the angle
  gyroAngleX = gyroAngleX + GyroX * (elapsedTime / 1000.0);
  gyroAngleY = gyroAngleY + GyroY * (elapsedTime / 1000.0);
  yaw = yaw + GyroZ * (elapsedTime / 1000.0);
  
  // Complementary Filter for Roll and Pitch
  roll = alpha * (roll + GyroX * (elapsedTime / 1000.0)) + (1 - alpha) * accAngleX;
  pitch = alpha * (pitch + GyroY * (elapsedTime / 1000.0)) + (1 - alpha) * accAngleY;
  yaw = (yaw > 180 ? fmod(yaw, 180.0) : yaw);

  // Print the values on the serial monitor
  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print("\t");
  Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.print("\t");
  Serial.print("Yaw: ");
  Serial.print(yaw);
  Serial.print("\t");
  Serial.print("GyroX: ");
  Serial.print(GyroX);
  Serial.print("\t");
  Serial.print("GyroY: ");
  Serial.print(GyroY);
  Serial.print("\t");
  Serial.print("GyroZ: ");
  Serial.print(GyroZ);
  Serial.println("\t");
  delay(20);
}