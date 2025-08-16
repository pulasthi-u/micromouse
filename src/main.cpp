#include <Arduino.h>
#include "driver/pcnt.h"
#include <Wire.h>

// ---- Pins (your map)
#define PWM_A   17
#define PWM_B   16
#define STBY    4
#define AIN1    25
#define AIN2    33
#define BIN1    27
#define BIN2    14

#define ENC1_A  26  // Left CHA (pulse)
#define ENC1_B  32  // Left CHB (ctrl)
#define ENC2_A  18  // Right CHA (pulse)
#define ENC2_B  19  // Right CHB (ctrl)

#define IR_FRONT 34 // ADC1_CH3
#define IR_LEFT  39 // ADC1_CH6
#define IR_RIGHT 35 // ADC1_CH7

// ---- PWM (LEDC)
const int PWM_CH_A = 0, PWM_CH_B = 1;
const int PWM_FREQ = 19000;     // ~19 kHz, silent
const int PWM_RES  = 10;        // 10-bit (0..1023)

// ---- Helpers
void motorA(int spd) { // spd: -1023..1023
  digitalWrite(AIN1, spd >= 0 ? HIGH : LOW);
  digitalWrite(AIN2, spd >= 0 ? LOW  : HIGH);
  ledcWrite(PWM_CH_A, abs(spd));
}
void motorB(int spd) {
  digitalWrite(BIN1, spd >= 0 ? HIGH : LOW);
  digitalWrite(BIN2, spd >= 0 ? LOW  : HIGH);
  ledcWrite(PWM_CH_B, abs(spd));
}

// ---- PCNT (quadrature: pulse=CHA, ctrl=CHB)
void pcnt_init_unit(pcnt_unit_t unit, gpio_num_t pulse, gpio_num_t ctrl) {
  pcnt_config_t c{};
  c.pulse_gpio_num = pulse;
  c.ctrl_gpio_num  = ctrl;
  c.lctrl_mode = PCNT_MODE_REVERSE; // count direction when ctrl=LOW
  c.hctrl_mode = PCNT_MODE_KEEP;    // direction when ctrl=HIGH
  c.pos_mode   = PCNT_COUNT_INC;    // count on rising edges
  c.neg_mode   = PCNT_COUNT_DEC;    // and on falling edges (x2)
  c.counter_h_lim =  32767;
  c.counter_l_lim = -32768;
  c.unit = unit;
  c.channel = PCNT_CHANNEL_0;       // we only need one channel
  pcnt_unit_config(&c);

  pcnt_set_filter_value(unit, 1000); // ~1us filter @80MHz APB (debounce)
  pcnt_filter_enable(unit);
  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}
int16_t pcnt_read(pcnt_unit_t unit) {
  int16_t v; pcnt_get_counter_value(unit, &v); return v;
}

// ---- Simple ADC helper (EMA filter)
int readADC_ema(int pin) {
  static int emaF=0, emaL=0, emaR=0;
  int raw = analogRead(pin);
  int &ema = (pin==IR_FRONT)?emaF:(pin==IR_LEFT)?emaL:emaR;
  ema = (ema==0) ? raw : (ema*7 + raw)/8; // alpha=1/8
  return ema;
}

// ---- I2C scan
void i2cScan() {
  Serial.println("I2C scan:");
  for (uint8_t addr=1; addr<127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission()==0) {
      Serial.printf("  - 0x%02X\n", addr);
      delay(2);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // Motor pins
  pinMode(STBY, OUTPUT); digitalWrite(STBY, LOW); // keep driver off initially
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

  // PWM
  ledcSetup(PWM_CH_A, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_A, PWM_CH_A);
  ledcAttachPin(PWM_B, PWM_CH_B);
  ledcWrite(PWM_CH_A, 0); ledcWrite(PWM_CH_B, 0);

  // Encoders
  pinMode(ENC1_A, INPUT_PULLUP); pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP); pinMode(ENC2_B, INPUT_PULLUP);
  pcnt_init_unit(PCNT_UNIT_0, (gpio_num_t)ENC1_A, (gpio_num_t)ENC1_B);
  pcnt_init_unit(PCNT_UNIT_1, (gpio_num_t)ENC2_A, (gpio_num_t)ENC2_B);

  // ADC (ADC1 only)
  analogReadResolution(12);
  analogSetPinAttenuation(IR_FRONT, ADC_11db);
  analogSetPinAttenuation(IR_LEFT,  ADC_11db);
  analogSetPinAttenuation(IR_RIGHT, ADC_11db);

  // I2C (MPU-6500/9250)
  Wire.begin(21, 22);
  i2cScan(); // you should see 0x68 (MPU); 0x0C for AK8963 on 9250 via passthrough

  Serial.println("Bring-up ready. Encoders/IR will print; motors idle.");
}

void loop() {
  static uint32_t t0 = 0;
  if (millis() - t0 > 200) {
    t0 = millis();
    int irF = readADC_ema(IR_FRONT);
    int irL = readADC_ema(IR_LEFT);
    int irR = readADC_ema(IR_RIGHT);
    int16_t eL = pcnt_read(PCNT_UNIT_0);
    int16_t eR = pcnt_read(PCNT_UNIT_1);
    Serial.printf("IR F/L/R: %4d %4d %4d | Enc L/R: %6d %6d\n", irF, irL, irR, eL, eR);
  }

  // --- OPTIONAL quick motor tick (uncomment to test) ---
   digitalWrite(STBY, HIGH);             // enable driver
   motorA(200); motorB(200); delay(500); // forward slow
   motorA(0);   motorB(0);   delay(300); // stop
  // motorA(-200);motorB(-200);delay(500); // reverse slow
  // digitalWrite(STBY, LOW);              // disable driver
}
