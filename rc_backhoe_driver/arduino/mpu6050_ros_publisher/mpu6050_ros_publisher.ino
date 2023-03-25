// #undef USBCON // for Arduino Due

// https://shizenkarasuzon.hatenablog.com/entry/2019/02/16/162647
// https://www.chisatofu.com/archives/26862572.html
// https://lastminuteengineers.com/mpu6050-accel-gyro-arduino-tutorial/

// #include <Arduino.h>
#include <Wire.h>

// ros msgs
#include <ros.h>
// #include <sensor_msgs/Imu.h>
// #include <std_msgs/Float32.h>
// #include <std_msgs/Int8.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>
// #include <std_msgs/String.h>

//アドレス　レジスタ　設定
#define MPU6050_WHO_AM_I     0x75  // Read Only
#define MPU6050_PWR_MGMT_1   0x6B  // Read and Write
#define MPU_ADDRESS  0x68

#define BUCKET_PIN 12
#define ARM_PIN 11
#define BOOM_PIN 10
#define SWING_PIN 9

ros::NodeHandle nh;

// sensor_msgs::Imu boom_imu_msg;
// ros::Publisher pub_boom_imu("/boom/imu/data_raw", &boom_imu_msg);
// sensor_msgs::Imu arm_imu_msg;
// ros::Publisher pub_arm_imu("/arm/imu/data_raw", &arm_imu_msg);
// sensor_msgs::Imu bucket_imu_msg;
// ros::Publisher pub_bucket_imu("/bucket/imu/data_raw", &bucket_imu_msg);
// sensor_msgs::Imu swing_imu_msg;
// ros::Publisher pub_swing_imu("/swing/imu/data_raw", &swing_imu_msg);

void cb_empty(const std_msgs::Bool& msg){}
ros::Subscriber<std_msgs::Bool> sub_empty("empty",  &cb_empty);

std_msgs::Int32MultiArray boom_imu_msg;
ros::Publisher pub_boom_imu("boom/imu/data_raw_bit", &boom_imu_msg);
std_msgs::Int32MultiArray arm_imu_msg;
ros::Publisher pub_arm_imu("arm/imu/data_raw_bit", &arm_imu_msg);
std_msgs::Int32MultiArray bucket_imu_msg;
ros::Publisher pub_bucket_imu("bucket/imu/data_raw_bit", &bucket_imu_msg);
std_msgs::Int32MultiArray swing_imu_msg;
ros::Publisher pub_swing_imu("swing/imu/data_raw_bit", &swing_imu_msg);

// std_msgs::Int8 test_msg;
// ros::Publisher pub_test("/test", &test_msg);

long old_time = millis();
long old_spin_time = millis();
int16_t temperature; // variables for temperature data

void setup() {
    Wire.begin();
    // Serial.begin(9600); //115200bps

    pinMode(BOOM_PIN,OUTPUT);
    pinMode(ARM_PIN,OUTPUT);
    pinMode(BUCKET_PIN,OUTPUT);
    pinMode(SWING_PIN,OUTPUT);
    digitalWrite(BOOM_PIN,HIGH);
    digitalWrite(ARM_PIN,HIGH);
    digitalWrite(BUCKET_PIN,HIGH);
    digitalWrite(SWING_PIN,HIGH);

    delay(50);

    //1個目の設定
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
    delay(50);

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
    delay(50);
    
    //3個目の設定
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
    delay(50);
    
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

    boom_imu_msg.data_length = 6;
    boom_imu_msg.data = (int32_t *)malloc(sizeof(int32_t) * 6);
    arm_imu_msg.data_length = 6;
    arm_imu_msg.data = (int32_t *)malloc(sizeof(int32_t) * 6);
    bucket_imu_msg.data_length = 6;
    bucket_imu_msg.data = (int32_t *)malloc(sizeof(int32_t) * 6);
    swing_imu_msg.data_length = 6;
    swing_imu_msg.data = (int32_t *)malloc(sizeof(int32_t) * 6);

    delay(100);
    //--- ROS初期化
    nh.initNode();
    nh.getHardware()->setBaud(57600);

    //--- Publisher初期化
    nh.advertise(pub_boom_imu);
    nh.advertise(pub_arm_imu);
    nh.advertise(pub_bucket_imu);
    nh.advertise(pub_swing_imu);
    // nh.advertise(pub_test);
    nh.subscribe(sub_empty);

}


void loop() {
    if ((millis()-old_time) >= 10){
        old_time = millis();
        //1個目の測定
        digitalWrite(BOOM_PIN, LOW);
        
        Wire.beginTransmission(0x68);
        Wire.write(0x3B);
        Wire.endTransmission(false);
        Wire.requestFrom(0x68, 14, true);
        while (Wire.available() < 14);

        int16_t boom_raw_ax = Wire.read() << 8 | Wire.read();
        int16_t boom_raw_ay = Wire.read() << 8 | Wire.read();
        int16_t boom_raw_az = Wire.read() << 8 | Wire.read();
        temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
        int16_t boom_raw_gx = Wire.read() << 8 | Wire.read();
        int16_t boom_raw_gy = Wire.read() << 8 | Wire.read();
        int16_t boom_raw_gz = Wire.read() << 8 | Wire.read();
        // 加速度値を分解能で割って加速度(G)に変換する
        // FS_SEL_0 16,384 LSB / g
        // boom_imu_msg.linear_acceleration.x = boom_raw_ax / 16384.0;
        // boom_imu_msg.linear_acceleration.y = boom_raw_ay / 16384.0;
        // boom_imu_msg.linear_acceleration.z = boom_raw_az / 16384.0;
        // // 角速度値を分解能で割って角速度(degrees per sec)に変換する
        // // FS_SEL_0 131 LSB / (°/s)
        // boom_imu_msg.angular_velocity.x = boom_raw_gx / 131.0 * 0.0174533;
        // boom_imu_msg.angular_velocity.y = boom_raw_gy / 131.0 * 0.0174533;
        // boom_imu_msg.angular_velocity.z = boom_raw_gz / 131.0 * 0.0174533;
        // boom_imu_msg.linear_acceleration.x = 0;//boom_raw_ax * 0.00059851074;
        // boom_imu_msg.linear_acceleration.y = 0;//boom_raw_ay * 0.00059851074;
        // boom_imu_msg.linear_acceleration.z = 0;//boom_raw_az * 0.00059851074;
        // boom_imu_msg.angular_velocity.x = 0;//boom_raw_gx * 0.00013323129;
        // boom_imu_msg.angular_velocity.y = 0;//boom_raw_gy * 0.00013323129;
        // boom_imu_msg.angular_velocity.z = 0;//boom_raw_gz * 0.00013323129;
        boom_imu_msg.data[0] = boom_raw_ax;
        boom_imu_msg.data[1] = boom_raw_ay;
        boom_imu_msg.data[2] = boom_raw_az;
        boom_imu_msg.data[3] = boom_raw_gx;
        boom_imu_msg.data[4] = boom_raw_gy;
        boom_imu_msg.data[5] = boom_raw_gz;
        digitalWrite(BOOM_PIN,HIGH);
        delay(1);
        
        //2個目の測定
        digitalWrite(ARM_PIN, LOW);
        Wire.beginTransmission(0x68);
        Wire.write(0x3B);
        Wire.endTransmission(false);
        Wire.requestFrom(0x68, 14, true);
        while (Wire.available() < 14);
        int16_t arm_raw_ax = Wire.read() << 8 | Wire.read();
        int16_t arm_raw_ay = Wire.read() << 8 | Wire.read();
        int16_t arm_raw_az = Wire.read() << 8 | Wire.read();
        temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
        int16_t arm_raw_gx = Wire.read() << 8 | Wire.read();
        int16_t arm_raw_gy = Wire.read() << 8 | Wire.read();
        int16_t arm_raw_gz = Wire.read() << 8 | Wire.read();
        arm_imu_msg.data[0] = arm_raw_ax;
        arm_imu_msg.data[1] = arm_raw_ay;
        arm_imu_msg.data[2] = arm_raw_az;
        arm_imu_msg.data[3] = arm_raw_gx;
        arm_imu_msg.data[4] = arm_raw_gy;
        arm_imu_msg.data[5] = arm_raw_gz;
        digitalWrite(ARM_PIN,HIGH);
        delay(1);
        
        //3個目の測定
        digitalWrite(BUCKET_PIN, LOW);        
        Wire.beginTransmission(0x68);
        Wire.write(0x3B);
        Wire.endTransmission(false);
        Wire.requestFrom(0x68, 14, true);
        while (Wire.available() < 14);
        int16_t bucket_raw_ax = Wire.read() << 8 | Wire.read();
        int16_t bucket_raw_ay = Wire.read() << 8 | Wire.read();
        int16_t bucket_raw_az = Wire.read() << 8 | Wire.read();
        temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x43 (TEMP_OUT_L)
        int16_t bucket_raw_gx = Wire.read() << 8 | Wire.read();
        int16_t bucket_raw_gy = Wire.read() << 8 | Wire.read();
        int16_t bucket_raw_gz = Wire.read() << 8 | Wire.read();
        bucket_imu_msg.data[0] = bucket_raw_ax;
        bucket_imu_msg.data[1] = bucket_raw_ay;
        bucket_imu_msg.data[2] = bucket_raw_az;
        bucket_imu_msg.data[3] = bucket_raw_gx;
        bucket_imu_msg.data[4] = bucket_raw_gy;
        bucket_imu_msg.data[5] = bucket_raw_gz;
        digitalWrite(BUCKET_PIN,HIGH);
        delay(1);
        
        // //4個目の測定
        // digitalWrite(SWING_PIN, LOW);        
        // Wire.beginTransmission(0x68);
        // Wire.write(0x3B);
        // Wire.endTransmission(false);
        // Wire.requestFrom(0x68, 14, true);
        // while (Wire.available() < 14);
        // int16_t swing_raw_ax = Wire.read() << 8 | Wire.read();
        // int16_t swing_raw_ay = Wire.read() << 8 | Wire.read();
        // int16_t swing_raw_az = Wire.read() << 8 | Wire.read();
        // temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x43 (TEMP_OUT_L)
        // int16_t swing_raw_gx = Wire.read() << 8 | Wire.read();
        // int16_t swing_raw_gy = Wire.read() << 8 | Wire.read();
        // int16_t swing_raw_gz = Wire.read() << 8 | Wire.read();
        // swing_imu_msg.data[0] = swing_raw_ax;
        // swing_imu_msg.data[1] = swing_raw_ay;
        // swing_imu_msg.data[2] = swing_raw_az;
        // swing_imu_msg.data[3] = swing_raw_gx;
        // swing_imu_msg.data[4] = swing_raw_gy;
        // swing_imu_msg.data[5] = swing_raw_gz;
        // digitalWrite(SWING_PIN,HIGH);
        // delay(1);

        pub_boom_imu.publish(&boom_imu_msg);
        pub_arm_imu.publish(&arm_imu_msg);
        pub_bucket_imu.publish(&bucket_imu_msg);
        pub_swing_imu.publish(&swing_imu_msg);
        nh.spinOnce();
    }
    delay(1);
}