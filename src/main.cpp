#include <Arduino.h>

#include <Wheels.h>
#include <Drive.h>

Wheel left(12, 14, 13);
Wheel right(1, 2, 3);

const int left_enc_1 = 26;
const int left_enc_2 = 27;

const int right_enc_1 = 4;
const int right_enc_2 = 5;

volatile long left_enc_count_1 = 0;
volatile long left_enc_count_2 = 0;
volatile long right_enc_count_1 = 0;
volatile long right_enc_count_2 = 0;

Drive drive(left, right, left_enc_count_1, right_enc_count_1);

void IRAM_ATTR left_enc_1_isr() {
	left_enc_count_1++;
}

void IRAM_ATTR left_enc_2_isr() {
	left_enc_count_2++;
}

void IRAM_ATTR right_enc_1_isr() {
right_enc_count_1++;
}

void IRAM_ATTR right_enc_2_isr() {
	right_enc_count_2++;
}

void setup() {
	Serial.begin(115200);

  // pinMode(LED_BUILTIN, OUTPUT);

  // Serial.println("HELLO!!!");
  // delay(1000);

	attachInterrupt(left_enc_1, left_enc_1_isr, FALLING);
  attachInterrupt(left_enc_2, left_enc_2_isr, FALLING);
  attachInterrupt(right_enc_1, right_enc_1_isr, FALLING);
  attachInterrupt(right_enc_2, right_enc_2_isr, FALLING);

  delay(2000);
  
  left.setSpeed(100);
  right.setSpeed(100);

  Serial.println("Starting wheels...");
  // left.forward();
  // right.forward();
  // Serial.println("Wheels started");
  // Serial.println("Left Encoder 1 Count: " + String(left_enc_count_1));
  // Serial.println("Left Encoder 2 Count: " + String(left_enc_count_2));
  // Serial.println("ENTERING LOOP");

  drive.forward(1000);
  drive.stop();
  delay(1000);
  drive.backward(1000);
  drive.stop();
}

void loop() {
  Serial.println(left_enc_count_1);
  int speed = max(0L, 2000 - left_enc_count_1);
  left.setSpeed(speed);
  right.setSpeed(speed);

}