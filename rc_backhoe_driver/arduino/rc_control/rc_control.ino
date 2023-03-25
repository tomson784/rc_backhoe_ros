#include <ros.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Int8.h>

#define BOOM_UP 4
#define BOOM_DOWN 5
#define ARM_UP 6
#define ARM_DOWN 7
#define BUCKET_UP 8
#define BUCKET_DOWN 9
#define SWING_RIGHT 10
#define SWING_LEFT 11

ros::NodeHandle nh;
void cb_cmd(const std_msgs::Int8MultiArray& msg);

int boom = 0;
int arm = 0;
int bucket = 0;
int swing = 0;
// int boom;

// publisher, subscriberのハンドル作り，PC側で行うやり方と微妙に異なるので要注意
ros::Subscriber<std_msgs::Int8MultiArray> sub_ctl_cmd("cmd/actuator",  &cb_cmd);

std_msgs::Int8MultiArray fb_test;
ros::Publisher pub_fb_test("/debug", &fb_test);

void cb_cmd(const std_msgs::Int8MultiArray& msg){
    // swing
    if (msg.data[0] == 0){ swing = 0; }
    else if (msg.data[0] == 1){ swing = 1; }
    else if (msg.data[0] == -1){ swing = -1; }
    // boom
    if (msg.data[1] == 0){ boom = 0; }
    else if (msg.data[1] == 1){ boom = 1; }
    else if (msg.data[1] == -1){ boom = -1; }
    // arm
    if (msg.data[2] == 0){ arm = 0; }
    else if (msg.data[2] == 1){ arm = 1; }
    else if (msg.data[2] == -1){ arm = -1; }
    // bucket
    if (msg.data[3] == 0){ bucket = 0; }
    else if (msg.data[3] == 1){ bucket = 1; }
    else if (msg.data[3] == -1){ bucket = -1; }
    // if (msg.data[4] == 0){ boom = 0; }
    // else if (msg.data[4] == 1){ boom = 1; }
    // else if (msg.data[4] == -1){ boom = -1; }
}

void setup() {
    pinMode(BOOM_UP, OUTPUT);
    pinMode(BOOM_DOWN, OUTPUT);
    pinMode(ARM_UP, OUTPUT);
    pinMode(ARM_DOWN, OUTPUT);
    pinMode(BUCKET_UP, OUTPUT);
    pinMode(BUCKET_DOWN, OUTPUT);
    pinMode(SWING_RIGHT, OUTPUT);
    pinMode(SWING_LEFT, OUTPUT);
    digitalWrite(BOOM_UP, HIGH);
    digitalWrite(BOOM_DOWN, HIGH);
    digitalWrite(ARM_UP, HIGH);
    digitalWrite(ARM_DOWN, HIGH);
    digitalWrite(BUCKET_UP, HIGH);
    digitalWrite(BUCKET_DOWN, HIGH);
    digitalWrite(SWING_RIGHT, HIGH);
    digitalWrite(SWING_LEFT, HIGH);
    delay(100);
    //--- ROS初期化
    nh.initNode();
    nh.getHardware()->setBaud(57600);

    //--- Publisher初期化
    nh.advertise(pub_fb_test);
    fb_test.data_length = 4;
    fb_test.data = (int8_t *)malloc(sizeof(int8_t) * 4);

    //--- Subscriber初期化
    nh.subscribe(sub_ctl_cmd);
    delay(100);
}

long old_time = millis();
long old_spin_time = millis();

void loop() {

    if ((millis()-old_time) >= 10){
        old_time = millis();
        // swing left or right
        if (swing == 1){
            digitalWrite(SWING_RIGHT, LOW);
            digitalWrite(SWING_LEFT, HIGH);
        }else if (swing == -1){
            digitalWrite(SWING_RIGHT, HIGH);
            digitalWrite(SWING_LEFT, LOW);
        }else{
            digitalWrite(SWING_RIGHT, HIGH);
            digitalWrite(SWING_LEFT, HIGH);
        }
        // boom up
        if (boom == 1){
            digitalWrite(BOOM_UP, LOW);
            digitalWrite(BOOM_DOWN, HIGH);
        // boom down
        }else if (boom == -1){
            digitalWrite(BOOM_UP, HIGH);
            digitalWrite(BOOM_DOWN, LOW);
        }else{
            digitalWrite(BOOM_UP, HIGH);
            digitalWrite(BOOM_DOWN, HIGH);
        }
        // arm up
        if (arm == 1){
            digitalWrite(ARM_UP, LOW);
            digitalWrite(ARM_DOWN, HIGH);
        // arm down
        }else if (arm == -1){
            digitalWrite(ARM_UP, HIGH);
            digitalWrite(ARM_DOWN, LOW);
        }else{
            digitalWrite(ARM_UP, HIGH);
            digitalWrite(ARM_DOWN, HIGH);
        }
        // bucket up
        if (bucket == 1){
            digitalWrite(BUCKET_UP, LOW);
            digitalWrite(BUCKET_DOWN, HIGH);
        // bucket down
        }else if (bucket == -1){
            digitalWrite(BUCKET_UP, HIGH);
            digitalWrite(BUCKET_DOWN, LOW);
        }else{
            digitalWrite(BUCKET_UP, HIGH);
            digitalWrite(BUCKET_DOWN, HIGH);
        }
    }
    if ((millis()-old_spin_time) >= 50){
        fb_test.data[0] = boom;
        fb_test.data[1] = arm;
        fb_test.data[2] = bucket;
        fb_test.data[3] = swing;
        pub_fb_test.publish(&fb_test);
        nh.spinOnce();
    }
}
