
#define BOOM_UP 4
#define BOOM_DOWN 5
#define ARM_UP 6
#define ARM_DOWN 7
#define BUCKET_UP 8
#define BUCKET_DOWN 9
#define SWING_RIGHT 10
#define SWING_LEFT 11

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
    delay(1000);
}
 
void loop() {
    digitalWrite(BOOM_UP, LOW);
    delay(500);
    digitalWrite(BOOM_UP, HIGH);
    delay(500);
    digitalWrite(BOOM_DOWN, LOW);
    delay(500);
    digitalWrite(BOOM_DOWN, HIGH);
    delay(500);
    digitalWrite(ARM_UP, LOW);
    delay(500);
    digitalWrite(ARM_UP, HIGH);
    delay(500);
    digitalWrite(ARM_DOWN, LOW);
    delay(500);
    digitalWrite(ARM_DOWN, HIGH);
    delay(500);
    digitalWrite(BUCKET_UP, LOW);
    delay(500);
    digitalWrite(BUCKET_UP, HIGH);
    delay(500);
    digitalWrite(BUCKET_DOWN, LOW);
    delay(500);
    digitalWrite(BUCKET_DOWN, HIGH);
    delay(500);
    digitalWrite(SWING_RIGHT, LOW);
    delay(500);
    digitalWrite(SWING_RIGHT, HIGH);
    delay(500);
    digitalWrite(SWING_LEFT, LOW);
    delay(500);
    digitalWrite(SWING_LEFT, HIGH);
    delay(500);
}
