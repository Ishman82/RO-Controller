// === Combined Sketch: Flow Control + Fan Control + FS300A + DS18B20 ===
// Board: Arduino UNO R4 WiFi
// Display: Duinotech 16x2 LCD Keypad Shield (LiquidCrystal(8,9,4,5,6,7))
// Pins mapping (conflicts resolved):
//   LCD: RS=8, E=9, D4=4, D5=5, D6=6, D7=7; Buttons=A0
//   Relay Pump: D13   <-- moved from D11
//   Relay Solenoid: D12
//   Float switch: A2 (to GND, INPUT_PULLUP)
//   FS300A Flow Sensor (interrupt): D3  [F(Hz)=6*Q(L/min) => L/h=10*F]
//   Fan PWM: D11      <-- moved from D10 (backlight pin)
//   Fan Tach (interrupt): D2
//   KS0033 Thermistor analog: A1
//   DS18B20 "Tank Temp": A3
//   LCD Backlight control (shield): D10 (held HIGH)

#include <Arduino.h>
#include <LiquidCrystal.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>

// ---- Option A fix: make IRAM_ATTR a no-op on UNO R4 ----
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif
// --------------------------------------------------------

// -------- LCD Shield --------
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
enum Button { BTN_NONE, BTN_RIGHT, BTN_UP, BTN_DOWN, BTN_LEFT, BTN_SELECT };
static const uint8_t PIN_BTN = A0;
const uint8_t PIN_LCD_BACKLIGHT = 10; // keep HIGH for backlight ON

// Button windows (Duinotech typical)
struct BtnWin { int minv, maxv; Button b; };
BtnWin BTN_WINS[] = {
  {   0,  40, BTN_RIGHT },
  {  70, 135, BTN_UP    },
  { 220, 300, BTN_DOWN  },
  { 380, 450, BTN_LEFT  },
  { 600, 690, BTN_SELECT }
};

Button readButtonRaw() {
  int v = analogRead(PIN_BTN);
  for (auto &w : BTN_WINS) if (v >= w.minv && v <= w.maxv) return w.b;
  return BTN_NONE;
}

// Debounce for edge detection on SELECT and RIGHT
Button lastBtn = BTN_NONE;
unsigned long lastBtnMs = 0;
const unsigned long BTN_DEBOUNCE_MS = 200;
bool selectEdge = false, rightEdge = false;

void scanButtonsForEdges() {
  Button b = readButtonRaw();
  unsigned long now = millis();
  selectEdge = false;
  rightEdge  = false;
  if (b != lastBtn && (now - lastBtnMs) >= BTN_DEBOUNCE_MS) {
    if (b == BTN_SELECT && lastBtn != BTN_SELECT) selectEdge = true;
    if (b == BTN_RIGHT  && lastBtn != BTN_RIGHT ) rightEdge  = true;
    lastBtn = b;
    lastBtnMs = now;
  }
}

// -------- Relays + Float --------
static const uint8_t PIN_RELAY_PUMP = 13;     // moved from 11
static const uint8_t PIN_RELAY_SOL  = 12;
static const uint8_t PIN_FLOAT      = A2;     // frees D3 for FS300A

static const bool RELAY_ACTIVE_LOW  = true;   // LOW-level trigger relays
static const bool FLOAT_CLOSED_MEANS_LOW_LEVEL = true;

// Relay write helper
void writeRelay(uint8_t pin, bool on) {
  if (RELAY_ACTIVE_LOW) digitalWrite(pin, on ? LOW : HIGH);
  else                  digitalWrite(pin, on ? HIGH : LOW);
}

// Control modes
enum Mode { MODE_AUTO = 0, MODE_MAN_ON = 1, MODE_MAN_OFF = 2 };
Mode mode = MODE_AUTO;

// Current relay states (actual outputs)
bool pumpOn = false;
bool solOn  = false;

// Sequencer for 1s staggering
enum SeqPhase { SEQ_IDLE, SEQ_WAIT_PUMP_ON, SEQ_WAIT_SOL_OFF };
SeqPhase seq = SEQ_IDLE;
unsigned long seqDue = 0;

void applyOutputs() {
  writeRelay(PIN_RELAY_PUMP, pumpOn);
  writeRelay(PIN_RELAY_SOL,  solOn);
}

// Request a target (both ON or both OFF) with proper sequencing
void requestTarget(bool wantOn) {
  unsigned long now = millis();
  switch (seq) {
    case SEQ_IDLE:
      if (wantOn) {
        if (!solOn) { solOn = true; applyOutputs(); }
        if (!pumpOn) { seq = SEQ_WAIT_PUMP_ON; seqDue = now + 1000; }
      } else {
        if (pumpOn) { pumpOn = false; applyOutputs(); }
        if (solOn)  { seq = SEQ_WAIT_SOL_OFF; seqDue = now + 1000; }
      }
      break;

    case SEQ_WAIT_PUMP_ON:
      if ((long)(now - seqDue) >= 0) {
        pumpOn = true; applyOutputs();
        seq = SEQ_IDLE;
      }
      if (!wantOn) {
        pumpOn = false; applyOutputs();
        seq = SEQ_WAIT_SOL_OFF; seqDue = now + 1000;
      }
      break;

    case SEQ_WAIT_SOL_OFF:
      if ((long)(now - seqDue) >= 0) {
        solOn = false; applyOutputs();
        seq = SEQ_IDLE;
      }
      if (wantOn) {
        if (!solOn) { solOn = true; applyOutputs(); }
        seq = SEQ_WAIT_PUMP_ON; seqDue = now + 1000;
      }
      break;
  }
}

bool floatDemandsOn() {
  int raw = digitalRead(PIN_FLOAT);
  bool closedToGND = (raw == LOW);
  return FLOAT_CLOSED_MEANS_LOW_LEVEL ? closedToGND : !closedToGND;
}

// -------- FS300A Flow Sensor (D3 interrupt) --------
// F(Hz) = 6 * Q(L/min) => Q = F/6 => L/h = 10 * F
static const uint8_t PIN_FS300A = 3;
volatile uint32_t fs300aPulses = 0;
void IRAM_ATTR flowISR() { fs300aPulses++; }

uint32_t lastFlowMs = 0;
float flowHz = 0.0f;
float flowLph = 0.0f;         // Liters/hour
const float FLOW_ALPHA = 0.30f; // EMA smoothing

void updateFlow() {
  uint32_t now = millis();
  if (now - lastFlowMs >= 1000) {
    noInterrupts();
    uint32_t cnt = fs300aPulses;
    fs300aPulses = 0;
    interrupts();

    float hz = (float)cnt;        // counted over ~1s → Hz ≈ pulses/sec
    if (flowHz <= 0.01f) flowHz = hz;
    else                 flowHz += FLOW_ALPHA * (hz - flowHz);

    float lph = 10.0f * flowHz;   // L/h = 10 * Hz
    if (flowLph <= 0.01f) flowLph = lph;
    else                  flowLph += FLOW_ALPHA * (lph - flowLph);

    lastFlowMs = now;
  }
}

// -------- DS18B20 Tank Temperature (A3) --------
static const uint8_t PIN_DS18B20 = A3;
OneWire oneWire(PIN_DS18B20);
DallasTemperature ds18b20(&oneWire);

uint32_t lastDSMs = 0;
float tankTempC = NAN;

void updateDS18B20() {
  uint32_t now = millis();
  if (now - lastDSMs >= 1000) {
    lastDSMs = now;
    ds18b20.requestTemperatures();
    float t = ds18b20.getTempCByIndex(0);
    if (t > -100.0f && t < 125.0f) tankTempC = t;
  }
}

// -------- Fan Control (moved PWM to D11) --------
const uint8_t FAN_PWM_PIN  = 11;  // was 10
const uint8_t FAN_TACH_PIN = 2;   // tach (interrupt-capable)
const uint8_t KS0033_PIN   = A1;  // KS0033 analog out

// KS0033 Thermistor
const float VREF = 5.0;
const float SERIES_RESISTOR = 4700.0;
const float THERM_NOMINAL   = 10000.0;
const float BETA_COEFF      = 3950.0;
const float TEMP_NOMINAL_C  = 25.0;
const int   ADC_MAX         = 1023;

// Control / Filters
const int   T_MIN = 25;
const int   T_MAX = 85;
const int   MIN_DUTY = 5;
const uint16_t SLEW_INTERVAL_MS = 75;
const int   DUTY_STEP = 3;

const float TEMP_ALPHA = 0.15f;
const float TEMP_BAD   = -1000.0f;

#define  TACH_EDGE_MODE FALLING
uint8_t  PULSES_PER_REV = 2;
const uint16_t MAX_RPM   = 9000;

const uint32_t MIN_PERIOD_US = 60000000UL / ( (uint32_t)MAX_RPM * (uint32_t) (PULSES_PER_REV) );
const uint32_t RPM_TIMEOUT_US = 500000;
const float    RPM_ALPHA_UP   = 0.25f;
const float    RPM_ALPHA_DOWN = 0.60f;
const float    CAL_FACTOR     = 1.00f;

const uint16_t KICK_MS = 300;

// Runtime
volatile uint32_t g_lastEdgeUs = 0;
volatile bool     g_havePrevEdge = false;
volatile uint32_t g_rawPeriodUs = 0;

uint32_t lastThermMs = 0, lastSlewMs = 0, lastPrintMs = 0;
int   currentDuty = 0, targetDuty = 0;
bool  kickActive = false;
uint32_t kickUntilMs = 0;

float tempFilteredC = TEMP_BAD;
float periodEmaUs   = 0.0f;
uint16_t lastRPM    = 0;

void onTachPulse() {
  uint32_t nowUs = micros();
  if (!g_havePrevEdge) {
    g_lastEdgeUs = nowUs;
    g_havePrevEdge = true;
    return;
  }
  uint32_t dt = nowUs - g_lastEdgeUs;
  if (dt < MIN_PERIOD_US) return;
  g_lastEdgeUs = nowUs;
  g_rawPeriodUs = dt;
}

static inline int clampi(int v, int lo, int hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

float readTempC_KS0033() {
  int raw = analogRead(KS0033_PIN);
  if (raw <= 0) return NAN;
  float v = (raw * VREF) / ADC_MAX;
  if (v <= 0.0f || v >= VREF) return NAN;
  float r_th = (VREF - v) * SERIES_RESISTOR / v;
  float invT = (log(r_th / THERM_NOMINAL) / BETA_COEFF) + (1.0f / (TEMP_NOMINAL_C + 273.15f));
  return (1.0f / invT) - 273.15f;
}

int tempToDuty(float tempC) {
  if (!isfinite(tempC)) return 255; // fail-safe
  if (tempC <= T_MIN) return 0;
  if (tempC >= T_MAX) return 255;
  long d = map((int)roundf(tempC), T_MIN, T_MAX, MIN_DUTY, 255);
  return clampi((int)d, 0, 255);
}

void applyPWM(int duty) {
  duty = clampi(duty, 0, 255);
  analogWrite(FAN_PWM_PIN, duty);
  currentDuty = duty;
}

void maybeDoKickStart() {
  if (!kickActive) return;
  if (millis() >= kickUntilMs) { kickActive = false; applyPWM(clampi(targetDuty, MIN_DUTY, 255)); }
  else                          { applyPWM(255); }
}

void updateSlew() {
  if (kickActive) return;
  if (millis() - lastSlewMs < SLEW_INTERVAL_MS) return;
  lastSlewMs = millis();
  if (currentDuty == targetDuty) return;
  int diff = targetDuty - currentDuty;
  int step = (diff > 0) ? clampi(diff, 1, DUTY_STEP) : -clampi(-diff, 1, DUTY_STEP);
  applyPWM(currentDuty + step);
}

void updateRPM() {
  uint32_t edgeUs, rawUs;
  noInterrupts();
  edgeUs = g_lastEdgeUs;
  rawUs  = g_rawPeriodUs;
  interrupts();

  uint32_t ageUs = micros() - edgeUs;
  if (currentDuty < MIN_DUTY || ageUs > RPM_TIMEOUT_US || !g_havePrevEdge) {
    periodEmaUs = 0.0f;
    lastRPM = 0;
    noInterrupts();
    g_havePrevEdge = false;
    interrupts();
    return;
  }

  if (rawUs == 0) return;

  if (periodEmaUs <= 0.0f) periodEmaUs = (float)rawUs;
  else {
    float alpha = (rawUs < periodEmaUs) ? RPM_ALPHA_DOWN : RPM_ALPHA_UP;
    periodEmaUs += alpha * ((float)rawUs - periodEmaUs);
  }

  float rpm = 60000000.0f / ((float)PULSES_PER_REV * periodEmaUs);
  rpm *= CAL_FACTOR;
  if (rpm < 0) rpm = 0;
  if (rpm > 65535.0f) rpm = 65535.0f;
  lastRPM = (uint16_t)roundf(rpm);
}

// -------- LCD Pages --------
uint8_t page = 0; // 0=status/mode, 1=flow+temp
unsigned long lastLcdMs = 0;

void drawLCD() {
  unsigned long now = millis();
  if (now - lastLcdMs < 150) return;
  lastLcdMs = now;

  lcd.setCursor(0,0);
  lcd.print("                ");
  lcd.setCursor(0,1);
  lcd.print("                ");

  if (page == 0) {
    // Original page: Pump/Sol + Mode
    lcd.setCursor(0,0);
    lcd.print("Pump:");
    lcd.print(pumpOn ? "ON " : "OFF");
    lcd.print(" Sol:");
    lcd.print(solOn ? "ON " : "OFF");

    lcd.setCursor(0,1);
    switch (mode) {
      case MODE_AUTO:   lcd.print("Mode:AUTO     "); break;
      case MODE_MAN_ON: lcd.print("Mode:MAN ON   "); break;
      case MODE_MAN_OFF:lcd.print("Mode:MAN OFF  "); break;
    }
  } else {
    // Page 2: Flow + Tank Temp
    lcd.setCursor(0,0);
    lcd.print("Flow:");
    int flowRounded = (int)roundf(flowLph);
    if (flowRounded > 9999) flowRounded = 9999;
    char buf1[17];
    snprintf(buf1, sizeof(buf1), " %4dL/H", flowRounded);
    lcd.print(buf1);

    lcd.setCursor(0,1);
    lcd.print("Tank Temp:");
    if (isnan(tankTempC)) lcd.print(" --C");
    else {
      char buf2[8];
      snprintf(buf2, sizeof(buf2), " %2dC", (int)roundf(tankTempC));
      lcd.print(buf2);
    }
  }
}

// -------- Setup / Loop --------
void setup() {
  Serial.begin(115200);
  delay(200);

  // Keep LCD backlight ON
  pinMode(PIN_LCD_BACKLIGHT, OUTPUT);
  digitalWrite(PIN_LCD_BACKLIGHT, HIGH);

  // IO directions
  pinMode(PIN_RELAY_PUMP, OUTPUT);
  pinMode(PIN_RELAY_SOL,  OUTPUT);
  pumpOn = false; solOn = false; applyOutputs();

  pinMode(PIN_FLOAT, INPUT_PULLUP);

  // FS300A (open-collector → pull-up). External 10k to 5V recommended.
  pinMode(PIN_FS300A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_FS300A), flowISR, RISING);

  // Fan control
  analogReadResolution(10);
  pinMode(FAN_PWM_PIN, OUTPUT);
  applyPWM(0);
  pinMode(FAN_TACH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN), onTachPulse, TACH_EDGE_MODE);

  // DS18B20
  ds18b20.begin();

  // LCD
  lcd.begin(16,2);
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("System Init...");
  lcd.setCursor(0,1); lcd.print("Please wait");
  delay(600);
  lcd.clear();

  Serial.println(F("UNO R4 WiFi — Combined Flow/Pump/Fan + FS300A + DS18B20"));
}

void loop() {
  // Buttons
  scanButtonsForEdges();

  // SELECT cycles control modes
  if (selectEdge) {
    if      (mode == MODE_AUTO)   mode = MODE_MAN_ON;
    else if (mode == MODE_MAN_ON) mode = MODE_MAN_OFF;
    else                          mode = MODE_AUTO;
  }

  // RIGHT toggles LCD page
  if (rightEdge) {
    page ^= 1;
  }

  // Decide desired target
  bool desiredOn = false;
  if (mode == MODE_AUTO)         desiredOn = floatDemandsOn();
  else if (mode == MODE_MAN_ON)  desiredOn = true;
  else                           desiredOn = false;

  // Drive sequencer towards desired target
  requestTarget(desiredOn);

  // FS300A & DS18B20 updates
  updateFlow();
  updateDS18B20();

  // Fan control
  uint32_t now = millis();

  // KS0033 temp every 500ms → target duty
  if (now - lastThermMs >= 500) {
    lastThermMs = now;
    float tC = readTempC_KS0033();
    if (!isfinite(tC)) {
      targetDuty = 255;
      Serial.println(F("KS0033 read error -> fan full speed"));
    } else {
      if (tempFilteredC == TEMP_BAD) tempFilteredC = tC;
      else                           tempFilteredC += TEMP_ALPHA * (tC - tempFilteredC);
      targetDuty = tempToDuty(tempFilteredC);
      if (!kickActive && currentDuty < MIN_DUTY && targetDuty >= MIN_DUTY) {
        kickActive = true;
        kickUntilMs = millis() + KICK_MS;
      }
    }
  }

  updateRPM();
  if (kickActive) maybeDoKickStart();
  else            updateSlew();

  // LCD
  drawLCD();

  // Serial telemetry (1s)
  if (now - lastPrintMs >= 1000) {
    lastPrintMs = now;
    Serial.print(F("[Page=")); Serial.print(page); Serial.print(F("] "));
    Serial.print(F("Mode="));
    switch (mode) {
      case MODE_AUTO:   Serial.print(F("AUTO")); break;
      case MODE_MAN_ON: Serial.print(F("MAN_ON")); break;
      case MODE_MAN_OFF:Serial.print(F("MAN_OFF")); break;
    }
    Serial.print(F("  Flow=")); Serial.print(flowLph, 0); Serial.print(F(" L/h"));
    Serial.print(F("  TankTemp="));
    if (isnan(tankTempC)) Serial.print(F("--"));
    else                  Serial.print(tankTempC, 1);
    Serial.print(F(" C  FanDuty=")); Serial.print(currentDuty);
    Serial.print(F("/255  FanRPM=")); Serial.println(lastRPM);
  }
}
