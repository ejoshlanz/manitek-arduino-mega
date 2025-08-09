#include <Arduino.h>
#include "HX711.h"

// HX711 #1
#define DT1_PIN  22
#define SCK1_PIN 23
// HX711 #2
#define DT2_PIN  24
#define SCK2_PIN 25

// IR break beams
#define IR1_PIN  12
#define IR2_PIN  13

// HC-SR04
#define TRIG_PIN 10
#define ECHO_PIN 11

// TCS3200 color sensor
#define S0_PIN   4
#define S1_PIN   5
#define S2_PIN   6
#define S3_PIN   7
#define LED_PIN  8
#define OUT_PIN  9

HX711 scale1;
HX711 scale2;

float calibration_factor1 = -1450.0;
float calibration_factor2 = -1450.0;

void setup() {
    Serial.begin(9600);
    Serial1.begin(115200);

    // IR sensors
    pinMode(IR1_PIN, INPUT_PULLUP);
    pinMode(IR2_PIN, INPUT_PULLUP);

    // HC-SR04
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    // TCS3200
    pinMode(S0_PIN, OUTPUT);
    pinMode(S1_PIN, OUTPUT);
    pinMode(S2_PIN, OUTPUT);
    pinMode(S3_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(OUT_PIN, INPUT);

    // TCS3200 config
    digitalWrite(S0_PIN, HIGH); // Frequency scaling 100%
    digitalWrite(S1_PIN, HIGH);
    digitalWrite(LED_PIN, HIGH); // Turn on sensor LED

    // HX711 init
    scale1.begin(DT1_PIN, SCK1_PIN);
    scale2.begin(DT2_PIN, SCK2_PIN);
    delay(500);
    scale1.tare();
    scale2.tare();

    Serial.println("Arduino Mega - All Sensors Ready");
}

float readDistanceCM() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    return duration * 0.0343 / 2;
}

unsigned long readColorFrequency(bool s2State, bool s3State) {
    digitalWrite(S2_PIN, s2State);
    digitalWrite(S3_PIN, s3State);
    delay(50);
    return pulseIn(OUT_PIN, LOW);
}

void readColor(unsigned long &red, unsigned long &green, unsigned long &blue) {
    // Red
    red = readColorFrequency(LOW, LOW);
    // Green
    green = readColorFrequency(HIGH, HIGH);
    // Blue
    blue = readColorFrequency(LOW, HIGH);
}

void loop() {
    // HX711 weights
    float weight1 = scale1.is_ready() ? scale1.get_units(5) : 0.0;
    float weight2 = scale2.is_ready() ? scale2.get_units(5) : 0.0;

    // IR beams
    int ir1State = digitalRead(IR1_PIN) == HIGH ? 1 : 0;
    int ir2State = digitalRead(IR2_PIN) == HIGH ? 1 : 0;

    // Distance
    float distanceCM = readDistanceCM();

    // Color
    unsigned long red, green, blue;
    readColor(red, green, blue);

    // Debug print
    Serial.print("W1: "); Serial.print(weight1, 2);
    Serial.print(" kg | W2: "); Serial.print(weight2, 2);
    Serial.print(" kg | IR1: "); Serial.print(ir1State);
    Serial.print(" | IR2: "); Serial.print(ir2State);
    Serial.print(" | Dist: "); Serial.print(distanceCM, 1);
    Serial.print(" cm | R: "); Serial.print(red);
    Serial.print(" G: "); Serial.print(green);
    Serial.print(" B: "); Serial.println(blue);

    // Send to ESP32
    Serial1.print("W1:"); Serial1.println(weight1, 2);
    Serial1.print("W2:"); Serial1.println(weight2, 2);
    Serial1.print("IR1:"); Serial1.println(ir1State);
    Serial1.print("IR2:"); Serial1.println(ir2State);
    Serial1.print("DIST:"); Serial1.println(distanceCM, 1);
    Serial1.print("R:"); Serial1.println(red);
    Serial1.print("G:"); Serial1.println(green);
    Serial1.print("B:"); Serial1.println(blue);

    delay(500);
}
