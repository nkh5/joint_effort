// Pico_Controller.ino - AMW controller on a Raspberry Pi Pico, built on
// touchgadget/flight_stick_tinyusb (MIT). Board: Raspberry Pi Pico
// (earlephilhower core), Tools > USB Stack > "Adafruit TinyUSB".
//
// What the Pi sees (check order with: jstest /dev/input/js0):
//   X, Y      : stick                     (10-bit)
//   twist     : stick twist, HID Rz       (8-bit)
//   slider    : encoder speed level 1-20, sent as 0-255 (level 1 = 0, level 20 = 255)
//   buttons 0-5 : stick button, panel buttons 1-5
//   button 6    : encoder push switch
//   hat         : unused, always centred

#include "tinyusb.h"

// ---------------- Pins ----------------
const int PIN_X = 26, PIN_Y = 27, PIN_TWIST = 28; // ADC0-2
const int BUTTON_PINS[] = {2, 3, 4, 5, 6, 7}; // to GND when pressed
const int NUM_BUTTONS = sizeof(BUTTON_PINS) / sizeof(BUTTON_PINS[0]);
const int ENC_A = 10, ENC_B = 11, ENC_SW = 12; // KY-040 CLK, DT, SW; module + to 3V3

// ---------------- Calibration ----------------
// Raw 12-bit ADC endpoints (0-4095). Set DEBUG 1, push each axis to both
// ends, copy the extremes here. Centre is measured at power-up.
struct AxisCal { int lo, center, hi; bool invert; };
AxisCal calX = {100, 2048, 3995, false};
AxisCal calY = {100, 2048, 3995, false};
AxisCal calT = {100, 2048, 3995, false};

const int NOISE_DEADBAND = 40; // raw counts; real deadband lives on the Pi
const int DEBOUNCE_MS = 15;
const int ENC_STEPS_PER_DETENT = 4; // KY-040: 4 pin changes per click
const bool ENC_INVERT = false;
const int LEVEL_MIN = 1, LEVEL_MAX = 20; // speed levels; Pi maps level -> speed
const int LEVEL_START = 10; // level at power-up
#define DEBUG 0 // 1 = print raw ADC values over USB serial

// ---------------- Globals ----------------
Adafruit_USBD_HID G_usb_hid;
FSJoystick FSJoy(&G_usb_hid);

volatile int32_t encCount = 0;
volatile uint8_t encState = 0;
const int8_t ENC_TABLE[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

bool btnState[NUM_BUTTONS + 1], btnLast[NUM_BUTTONS + 1];
uint32_t btnSince[NUM_BUTTONS + 1];
int level = LEVEL_START;

// ---------------- Helpers ----------------
void encISR() { // quadrature state table
  uint8_t s = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  encCount += ENC_TABLE[(encState << 2) | s];
  encState = s;
}

int readRaw(uint8_t pin) { // average 8 reads against ADC noise
  int total = 0;
  for (int i = 0; i < 8; i++) total += analogRead(pin);
  return total / 8;
}

void calibrateCenter(uint8_t pin, AxisCal &c) { // stick must be released at power-up
  long total = 0;
  for (int i = 0; i < 64; i++) { total += readRaw(pin); delay(1); }
  int m = total / 64;
  if (abs(m - (c.lo + c.hi) / 2) < (c.hi - c.lo) * 15 / 100) c.center = m;
}

// Raw ADC -> 0..outMax, each half scaled separately so both ends reach full scale.
// Output rises smoothly from 0 at the edge of the noise deadband.
int mapAxis(int raw, AxisCal &c, int outMax) {
  if (raw < c.lo) c.lo = raw; // auto-expand range
  if (raw > c.hi) c.hi = raw;
  float v = 0.0f;
  if (abs(raw - c.center) >= NOISE_DEADBAND)
    v = (raw > c.center) ? float(raw - c.center - NOISE_DEADBAND) / (c.hi - c.center - NOISE_DEADBAND)
                         : float(raw - c.center + NOISE_DEADBAND) / (c.center - c.lo - NOISE_DEADBAND);
  v = constrain(v, -1.0f, 1.0f);
  if (c.invert) v = -v;
  return lroundf((v + 1.0f) * 0.5f * outMax);
}

bool debounce(uint8_t i, uint8_t pin, uint32_t now) {
  bool raw = digitalRead(pin) == LOW; // active low with pull-up
  if (raw != btnLast[i]) { btnLast[i] = raw; btnSince[i] = now; }
  else if (now - btnSince[i] >= (uint32_t)DEBOUNCE_MS) btnState[i] = raw;
  return btnState[i];
}

// ---------------- Setup ----------------
void setup() {
  if (!TinyUSBDevice.isInitialized()) TinyUSBDevice.begin(0);
  TinyUSBDevice.setManufacturerDescriptor("Joint Effort");
  TinyUSBDevice.setProductDescriptor("AMW Controller");
  FSJoy.begin();
  if (TinyUSBDevice.mounted()) { // re-enumerate so the host sees the joystick
    TinyUSBDevice.detach(); delay(10); TinyUSBDevice.attach();
  }

  analogReadResolution(12);
  for (int p : BUTTON_PINS) pinMode(p, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  encState = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), encISR, CHANGE);

  calibrateCenter(PIN_X, calX);
  calibrateCenter(PIN_Y, calY);
  calibrateCenter(PIN_TWIST, calT);

  pinMode(LED_BUILTIN, OUTPUT);
  while (!TinyUSBDevice.mounted()) { // blink until the host accepts us
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
  digitalWrite(LED_BUILTIN, HIGH);
}

// ---------------- Loop ----------------
void loop() {
  uint32_t now = millis();

  FSJoy.xAxis(mapAxis(readRaw(PIN_X), calX, 1023));
  FSJoy.yAxis(mapAxis(readRaw(PIN_Y), calY, 1023));
  FSJoy.twist(mapAxis(readRaw(PIN_TWIST), calT, 255));

  uint16_t bits = 0;
  for (int i = 0; i < NUM_BUTTONS; i++)
    if (debounce(i, BUTTON_PINS[i], now)) bits |= 1 << i;
  if (debounce(NUM_BUTTONS, ENC_SW, now)) bits |= 1 << 6;
  FSJoy.buttons(bits);

  noInterrupts(); // take whole clicks, keep the remainder
  int32_t c = encCount;
  int32_t clicks = c / ENC_STEPS_PER_DETENT;
  encCount = c - clicks * ENC_STEPS_PER_DETENT;
  interrupts();
  if (ENC_INVERT) clicks = -clicks;
  level = constrain(level + clicks, LEVEL_MIN, LEVEL_MAX); // stop at the ends, no wrap
  FSJoy.slider(lroundf(float(level - LEVEL_MIN) * 255.0f / (LEVEL_MAX - LEVEL_MIN))); // level 1..20 -> 0..255

  if (FSJoy.ready()) FSJoy.loop(); // sends at most once per ms

#if DEBUG
  static uint32_t lastPrint = 0;
  if (now - lastPrint >= 100) {
    lastPrint = now;
    Serial.printf("raw x=%d y=%d t=%d  level=%d  buttons=0x%02x\n",
                  readRaw(PIN_X), readRaw(PIN_Y), readRaw(PIN_TWIST), level, bits);
  }
#endif
}
