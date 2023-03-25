// https://www.chisatofu.com/archives/26862572.html

#include <Wire.h>

//アドレス　レジスタ　設定
#define MPU6050_WHO_AM_I     0x75  // Read Only
#define MPU6050_PWR_MGMT_1   0x6B  // Read and Write
#define MPU_ADDRESS  0x68
int16_t temperature; // variables for temperature data
char tmp_str[7]; // temporary variable used in convert function
char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

#define BUCKET_PIN 12
#define ARM_PIN 11
#define BOOM_PIN 10
#define SWING_PIN 9

void setup() {
  Wire.begin();
  Serial.begin(115200); //115200bps

  pinMode(BUCKET_PIN,OUTPUT);
  pinMode(ARM_PIN,OUTPUT);
  pinMode(BOOM_PIN,OUTPUT);
  pinMode(SWING_PIN,OUTPUT);
  digitalWrite(BUCKET_PIN,HIGH);
  digitalWrite(ARM_PIN,HIGH);
  digitalWrite(BOOM_PIN,HIGH);
  digitalWrite(SWING_PIN,HIGH);

  //1個目の設定
  digitalWrite(BUCKET_PIN, LOW);
 
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU6050_WHO_AM_I);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU6050_PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();
  
  digitalWrite(BUCKET_PIN,HIGH);
  
  //2個目の設定
  digitalWrite(ARM_PIN, LOW);
 
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU6050_WHO_AM_I);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU6050_PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();
  
  digitalWrite(ARM_PIN,HIGH);
  
  //3個目の設定
  digitalWrite(BOOM_PIN, LOW);
 
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU6050_WHO_AM_I);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU6050_PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();
  
  digitalWrite(BOOM_PIN,HIGH);
  
  // //4個目の設定
  // digitalWrite(SWING_PIN, LOW);
 
  // Wire.beginTransmission(MPU_ADDRESS);
  // Wire.write(MPU6050_WHO_AM_I);
  // Wire.write(0x00);
  // Wire.endTransmission();

  // Wire.beginTransmission(MPU_ADDRESS);
  // Wire.write(MPU6050_PWR_MGMT_1);
  // Wire.write(0x00);
  // Wire.endTransmission();
  
  // digitalWrite(SWING_PIN,HIGH);
}


void loop() {
  //1個目の測定
  digitalWrite(BUCKET_PIN, LOW);
  
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);
  while (Wire.available() < 14);

  // int16_t axRaw_1, ayRaw_1, azRaw_1;
  // int16_t gxRaw_1, gyRaw_1, gzRaw_1;

  int16_t axRaw_1 = Wire.read() << 8 | Wire.read();
  int16_t ayRaw_1 = Wire.read() << 8 | Wire.read();
  int16_t azRaw_1 = Wire.read() << 8 | Wire.read();
  temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  int16_t gxRaw_1 = Wire.read() << 8 | Wire.read();
  int16_t gyRaw_1 = Wire.read() << 8 | Wire.read();
  int16_t gzRaw_1 = Wire.read() << 8 | Wire.read();

  float acc_x_1 = axRaw_1 / 16384.0;
  float acc_y_1 = ayRaw_1 / 16384.0;
  float acc_z_1 = azRaw_1 / 16384.0;
  float gyr_x_1 = gxRaw_1 / 16384.0;
  float gyr_y_1 = gyRaw_1 / 16384.0;
  float gyr_z_1 = gzRaw_1 / 16384.0;

  // // Serial.print(convert_int16_to_str(axRaw_1));  Serial.print("\t");
  // // Serial.print(convert_int16_to_str(ayRaw_1));  Serial.print("\t");
  // // Serial.print(convert_int16_to_str(azRaw_1));  Serial.print("\t");
  // // Serial.print(convert_int16_to_str(gxRaw_1));  Serial.print("\t");
  // // Serial.print(convert_int16_to_str(gyRaw_1));  Serial.print("\t");
  // // Serial.print(convert_int16_to_str(gzRaw_1));  Serial.print("\t");
  Serial.print(acc_x_1);  Serial.print("\t");
  Serial.print(acc_y_1);  Serial.print("\t");
  Serial.print(acc_z_1);  Serial.print("\t");
  Serial.print(gyr_x_1);  Serial.print("\t");
  Serial.print(gyr_y_1);  Serial.print("\t");
  Serial.print(gyr_z_1);  Serial.print("\t");
  Serial.print("1\n");
  
  digitalWrite(BUCKET_PIN,HIGH);
  
  //2個目の測定
  digitalWrite(ARM_PIN, LOW);
  
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);
  while (Wire.available() < 14);

  // int16_t axRaw_2, ayRaw_2;

  int16_t axRaw_2 = Wire.read() << 8 | Wire.read();
  int16_t ayRaw_2 = Wire.read() << 8 | Wire.read();
  int16_t azRaw_2 = Wire.read() << 8 | Wire.read();
  temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  int16_t gxRaw_2 = Wire.read() << 8 | Wire.read();
  int16_t gyRaw_2 = Wire.read() << 8 | Wire.read();
  int16_t gzRaw_2 = Wire.read() << 8 | Wire.read();

  float acc_x_2 = axRaw_2 / 16384.0;
  float acc_y_2 = ayRaw_2 / 16384.0;
  float acc_z_2 = azRaw_2 / 16384.0;
  float gyr_x_2 = gxRaw_2 / 16384.0;
  float gyr_y_2 = gyRaw_2 / 16384.0;
  float gyr_z_2 = gzRaw_2 / 16384.0;
  
  Serial.print(acc_x_2);  Serial.print("\t");
  Serial.print(acc_y_2);  Serial.print("\t");
  Serial.print(acc_z_2);  Serial.print("\t");
  Serial.print(gyr_x_2);  Serial.print("\t");
  Serial.print(gyr_y_2);  Serial.print("\t");
  Serial.print(gyr_z_2);  Serial.print("\t");
  Serial.print("2\n");
  
  digitalWrite(ARM_PIN,HIGH);
  
  //3個目の測定
  digitalWrite(BOOM_PIN, LOW);
  
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);
  while (Wire.available() < 14);

  // int16_t axRaw_3, ayRaw_3, azRaw_3;
  // int16_t gxRaw_3, gyRaw_3, gzRaw_3;

  int16_t axRaw_3 = Wire.read() << 8 | Wire.read();
  int16_t ayRaw_3 = Wire.read() << 8 | Wire.read();
  int16_t azRaw_3 = Wire.read() << 8 | Wire.read();
  temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x43 (TEMP_OUT_L)
  int16_t gxRaw_3 = Wire.read() << 8 | Wire.read();
  int16_t gyRaw_3 = Wire.read() << 8 | Wire.read();
  int16_t gzRaw_3 = Wire.read() << 8 | Wire.read();

  float acc_x_3 = axRaw_3 / 16384.0;
  float acc_y_3 = ayRaw_3 / 16384.0;
  float acc_z_3 = azRaw_3 / 16384.0;
  float gyr_x_3 = gxRaw_3 / 16384.0;
  float gyr_y_3 = gyRaw_3 / 16384.0;
  float gyr_z_3 = gzRaw_3 / 16384.0;

  Serial.print(acc_x_3);  Serial.print("\t");
  Serial.print(acc_y_3);  Serial.print("\t");
  Serial.print(acc_z_3);  Serial.print("\t");
  Serial.print(gyr_x_3);  Serial.print("\t");
  Serial.print(gyr_y_3);  Serial.print("\t");
  Serial.print(gyr_z_3);  Serial.print("\t");
  Serial.print("3\n");
  
  digitalWrite(BOOM_PIN,HIGH);
  
  // //4個目の測定
  // digitalWrite(SWING_PIN, LOW);
  
  // Wire.beginTransmission(0x68);
  // Wire.write(0x3B);
  // Wire.endTransmission(false);
  // Wire.requestFrom(0x68, 14, true);
  // while (Wire.available() < 14);

  // int16_t axRaw_4, ayRaw_4;

  // axRaw_4 = Wire.read() << 8 | Wire.read();
  // ayRaw_4 = Wire.read() << 8 | Wire.read();

  // float acc_x_4 = axRaw_4 / 16384.0;
  // float acc_y_4 = ayRaw_4 / 16384.0;
  
  // Serial.print(acc_x_4);  Serial.print("\t");
  // Serial.print(acc_y_4);  Serial.println("");
  
  // digitalWrite(SWING_PIN,HIGH);
  delay(100);
}