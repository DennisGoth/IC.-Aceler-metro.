/*
  ESP32 + 2x MMA845X (I2C) + 2x WS2812B (75 LEDs each) - FastLED version
  - Converte a versão otimizada para usar FastLED
  - WS2812B, COLOR_ORDER = GRB
  - MMA845x alimentado em 3.3V
*/

#include <Wire.h>
#include <FastLED.h>
#include <math.h>

// --- Configuração de Pinos ---
#define LED_PIN_1 13
#define LED_PIN_2 14
#define NUM_LEDS_PER_STRIP 75
#define NUM_TOTAL_LEDS (NUM_LEDS_PER_STRIP * 2)

#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB

// --- I2C / MMA845x ---
#define ACCEL1_ADDRESS 0x1C
#define ACCEL2_ADDRESS 0x1D

#define MMA845X_OUT_X_MSB   0x01
#define MMA845X_WHO_AM_I    0x0D
#define MMA845X_CTRL_REG1   0x2A
#define MMA845X_XYZ_DATA_CFG 0x0E

// --- Filtro / Calibração ---
#define USE_EMA true
#define EMA_ALPHA 0.25f
#define FILTER_WINDOW_SIZE 10
const int CALIBRATION_SAMPLES_PER_SENSOR = 150;

// --- FastLED ---
CRGB leds1[NUM_LEDS_PER_STRIP];
CRGB leds2[NUM_LEDS_PER_STRIP];
const uint8_t DEFAULT_BRIGHTNESS = 50;

// --- Estrutura do sensor ---
struct SensorData {
  float bufferX[FILTER_WINDOW_SIZE];
  float bufferY[FILTER_WINDOW_SIZE];
  float bufferZ[FILTER_WINDOW_SIZE];
  int indexX, indexY, indexZ;
  float sumX, sumY, sumZ;

  float emaX, emaY, emaZ;

  float offsetX, offsetY, offsetZ;
  int calibCount;

  float calibratedX, calibratedY, calibratedZ;

  void init() {
    indexX = indexY = indexZ = 0;
    sumX = sumY = sumZ = 0.0f;
    emaX = emaY = emaZ = 0.0f;
    offsetX = offsetY = offsetZ = 0.0f;
    calibratedX = calibratedY = calibratedZ = 0.0f;
    calibCount = 0;
    for (int i=0;i<FILTER_WINDOW_SIZE;i++){
      bufferX[i]=bufferY[i]=bufferZ[i]=0.0f;
    }
  }
};
SensorData accel1, accel2;

// --- Estado global ---
bool calibrating = true;
unsigned long calibrationStart = 0;
DisplayMode currentMode;
unsigned long lastModeChangeLocal = 0;

// --- DisplayMode enum (declared after globals)
enum DisplayMode { RAINBOW_WAVE, IMPACT_PULSE, DIRECTIONAL_FLOW, ENERGY_MAP_SPLIT };

// --- Protótipos ---
void setupMMA845X(int address);
void readMMA845X(int address, float &x, float &y, float &z);
void processSensor(int address, SensorData &s);
float updateMovingAverage(float *buffer, float newValue, float &sum, int &index);
void finishCalibration();
void clearBothStrips();
void i2cScan();
void rainbowWaveEffect(float intensity, float dirX, float dirY);
void impactPulseEffect(float intensity);
void directionalFlowEffect(float dirX, float dirY);
void energyMapSplitEffect(float diffX, float diffY, float diffZ, float totalAccel);

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("=== ESP32 + 2x MMA845X + 2x WS2812B (FastLED) ===");

  // Inicializa sensores
  accel1.init();
  accel2.init();

  // FastLED init
  FastLED.addLeds<LED_TYPE, LED_PIN_1, COLOR_ORDER>(leds1, NUM_LEDS_PER_STRIP).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<LED_TYPE, LED_PIN_2, COLOR_ORDER>(leds2, NUM_LEDS_PER_STRIP).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(DEFAULT_BRIGHTNESS);
  clearBothStrips();

  // I2C
  Wire.begin(); // SDA=21, SCL=22 por padrão
  Wire.setClock(400000);
  i2cScan();

  Serial.println("Configurando acelerometros...");
  setupMMA845X(ACCEL1_ADDRESS);
  setupMMA845X(ACCEL2_ADDRESS);

  // Calibração
  Serial.println("Iniciando calibracao. Mantenha o dispositivo parado e nivelado...");
  calibrationStart = millis();
  calibrating = true;
  accel1.calibCount = 0;
  accel2.calibCount = 0;
}

// -------------------- LOOP --------------------
void loop() {
  unsigned long loopStart = millis();

  processSensor(ACCEL1_ADDRESS, accel1);
  processSensor(ACCEL2_ADDRESS, accel2);

  if (calibrating) {
    if (accel1.calibCount >= CALIBRATION_SAMPLES_PER_SENSOR && accel2.calibCount >= CALIBRATION_SAMPLES_PER_SENSOR) {
      finishCalibration();
    }
    return;
  }

  float diffX = fabs(accel1.calibratedX - accel2.calibratedX);
  float diffY = fabs(accel1.calibratedY - accel2.calibratedY);
  float diffZ = fabs(accel1.calibratedZ - accel2.calibratedZ);
  float totalAccel = sqrt(diffX*diffX + diffY*diffY + diffZ*diffZ);
  float directionX = (accel1.calibratedX + accel2.calibratedX) * 0.5f;
  float directionY = (accel1.calibratedY + accel2.calibratedY) * 0.5f;

  // Inicializa modo se ainda não inicializado
  static bool modeInit = false;
  if (!modeInit) { currentMode = RAINBOW_WAVE; modeInit = true; }

  // Efeitos
  switch(currentMode) {
    case RAINBOW_WAVE:
      rainbowWaveEffect(totalAccel, directionX, directionY);
      break;
    case IMPACT_PULSE:
      impactPulseEffect(totalAccel);
      break;
    case DIRECTIONAL_FLOW:
      directionalFlowEffect(directionX, directionY);
      break;
    case ENERGY_MAP_SPLIT:
      energyMapSplitEffect(diffX, diffY, diffZ, totalAccel);
      break;
  }

  // Troca de modo por impacto (debounced)
  if (totalAccel > 1.0f && millis() - lastModeChangeLocal > 2000) {
    currentMode = (DisplayMode)((currentMode + 1) % 4);
    Serial.print("Modo alterado para: ");
    switch(currentMode) {
      case RAINBOW_WAVE: Serial.println("Rainbow Wave"); break;
      case IMPACT_PULSE: Serial.println("Impact Pulse"); break;
      case DIRECTIONAL_FLOW: Serial.println("Directional Flow"); break;
      case ENERGY_MAP_SPLIT: Serial.println("Energy Map Split"); break;
    }
    lastModeChangeLocal = millis();
    clearBothStrips();
  }

  // Atualiza LEDs (FastLED.show atualiza todas as addLeds)
  FastLED.show();

  // Controle de taxa
  const int TARGET_LOOP_TIME = 25;
  int loopTime = millis() - loopStart;
  if (loopTime < TARGET_LOOP_TIME) delay(TARGET_LOOP_TIME - loopTime);
}

// -------------------- FUNÇÕES --------------------

void i2cScan() {
  Serial.println("Procurando dispositivos I2C...");
  byte count = 0;
  for (byte addr=1; addr<127; ++addr) {
    Wire.beginTransmission(addr);
    byte err = Wire.endTransmission();
    if (err == 0) {
      Serial.print("I2C encontrado em 0x");
      Serial.println(addr, HEX);
      count++;
    }
  }
  if (count == 0) Serial.println("Nenhum dispositivo I2C encontrado.");
  else { Serial.print("Total I2C: "); Serial.println(count); }
}

void setupMMA845X(int address) {
  Wire.beginTransmission(address);
  Wire.write(MMA845X_WHO_AM_I);
  Wire.endTransmission(false);
  Wire.requestFrom(address, 1);
  if (Wire.available() < 1) {
    Serial.print("Erro: Nenhum sensor no endereco 0x");
    Serial.println(address, HEX);
    return;
  }
  uint8_t id = Wire.read();
  Serial.print("WHO_AM_I (0x"); Serial.print(address, HEX); Serial.print(") = 0x"); Serial.println(id, HEX);

  // Standby
  Wire.beginTransmission(address);
  Wire.write(MMA845X_CTRL_REG1);
  Wire.write(0x00);
  Wire.endTransmission(true);
  delay(5);

  // ±4g
  Wire.beginTransmission(address);
  Wire.write(MMA845X_XYZ_DATA_CFG);
  Wire.write(0x01);
  Wire.endTransmission(true);
  delay(5);

  // Active
  Wire.beginTransmission(address);
  Wire.write(MMA845X_CTRL_REG1);
  Wire.write(0x01);
  Wire.endTransmission(true);
  delay(5);
}

void readMMA845X(int address, float &x, float &y, float &z) {
  Wire.beginTransmission(address);
  Wire.write(MMA845X_OUT_X_MSB);
  Wire.endTransmission(false);
  Wire.requestFrom(address, 6);

  if (Wire.available() < 6) { x = y = z = 0.0f; return; }

  int16_t rawX = (Wire.read() << 8) | Wire.read();
  int16_t rawY = (Wire.read() << 8) | Wire.read();
  int16_t rawZ = (Wire.read() << 8) | Wire.read();

  rawX >>= 4; rawY >>= 4; rawZ >>= 4;
  if (rawX & 0x0800) rawX |= 0xF000;
  if (rawY & 0x0800) rawY |= 0xF000;
  if (rawZ & 0x0800) rawZ |= 0xF000;

  const float scale = 8.0f / 4096.0f; // ±4g
  x = rawX * scale;
  y = rawY * scale;
  z = rawZ * scale;
}

float updateMovingAverage(float *buffer, float newValue, float &sum, int &index) {
  sum -= buffer[index];
  buffer[index] = newValue;
  sum += newValue;
  index = (index + 1) % FILTER_WINDOW_SIZE;
  return sum / (float)FILTER_WINDOW_SIZE;
}

void processSensor(int address, SensorData &s) {
  float rawX, rawY, rawZ;
  readMMA845X(address, rawX, rawY, rawZ);

  float fx, fy, fz;
  if (USE_EMA) {
    if (s.calibCount == 0 && s.emaX == 0.0f && s.emaY == 0.0f && s.emaZ == 0.0f) {
      s.emaX = rawX; s.emaY = rawY; s.emaZ = rawZ;
    } else {
      s.emaX = s.emaX * (1.0f - EMA_ALPHA) + rawX * EMA_ALPHA;
      s.emaY = s.emaY * (1.0f - EMA_ALPHA) + rawY * EMA_ALPHA;
      s.emaZ = s.emaZ * (1.0f - EMA_ALPHA) + rawZ * EMA_ALPHA;
    }
    fx = s.emaX; fy = s.emaY; fz = s.emaZ;
  } else {
    fx = updateMovingAverage(s.bufferX, rawX, s.sumX, s.indexX);
    fy = updateMovingAverage(s.bufferY, rawY, s.sumY, s.indexY);
    fz = updateMovingAverage(s.bufferZ, rawZ, s.sumZ, s.indexZ);
  }

  if (calibrating) {
    s.offsetX += fx;
    s.offsetY += fy;
    s.offsetZ += fz;
    s.calibCount++;
    return;
  }

  s.calibratedX = fx - s.offsetX;
  s.calibratedY = fy - s.offsetY;
  s.calibratedZ = fz - s.offsetZ;
}

void finishCalibration() {
  Serial.println("Finalizando calibracao...");
  if (accel1.calibCount > 0) {
    accel1.offsetX /= (float)accel1.calibCount;
    accel1.offsetY /= (float)accel1.calibCount;
    accel1.offsetZ /= (float)accel1.calibCount;
  }
  if (accel2.calibCount > 0) {
    accel2.offsetX /= (float)accel2.calibCount;
    accel2.offsetY /= (float)accel2.calibCount;
    accel2.offsetZ /= (float)accel2.calibCount;
  }

  const bool APPLY_GRAVITY_ADJUST = true;
  if (APPLY_GRAVITY_ADJUST) {
    accel1.offsetZ -= 1.0f;
    accel2.offsetZ -= 1.0f;
  }

  calibrating = false;
  Serial.println("Calibracao concluida.");
  Serial.print("Offsets1: "); Serial.print(accel1.offsetX); Serial.print(", "); Serial.print(accel1.offsetY); Serial.print(", "); Serial.println(accel1.offsetZ);
  Serial.print("Offsets2: "); Serial.print(accel2.offsetX); Serial.print(", "); Serial.print(accel2.offsetY); Serial.print(", "); Serial.println(accel2.offsetZ);
}

void clearBothStrips() {
  for (int i=0;i<NUM_LEDS_PER_STRIP;i++){ leds1[i] = CRGB::Black; leds2[i] = CRGB::Black; }
  FastLED.show();
}

// -------------------- EFEITOS (FastLED) --------------------

void rainbowWaveEffect(float intensity, float dirX, float dirY) {
  // FastLED CHSV hue range 0-255. Mapeia dirY para 0-255.
  int hueBase = map((int)constrain(dirY * 50.0f, -180, 180), -180, 180, 0, 255);
  int brightness = constrain((int)(intensity * 100.0f), 0, 255);

  for (int i=0;i<NUM_LEDS_PER_STRIP;i++) {
    // espalha hue ao longo da fita
    uint8_t ledHue = (uint8_t)((hueBase + (i * 256 / NUM_LEDS_PER_STRIP)) & 0xFF);
    CHSV hsvColor = CHSV(ledHue, 255, brightness);
    leds1[i] = hsvColor;
    leds2[i] = hsvColor;
  }
}

void impactPulseEffect(float intensity) {
  static int pulsePosition = 0;
  static unsigned long lastPulse = 0;

  if (intensity > 0.35f && millis() - lastPulse > 300) {
    pulsePosition = 0;
    lastPulse = millis();
  }

  if (pulsePosition < NUM_LEDS_PER_STRIP * 2) {
    for (int i=0;i<NUM_LEDS_PER_STRIP;i++) {
      float distance = fabs((float)i - (float)pulsePosition);
      if (distance < (NUM_LEDS_PER_STRIP / 2.0f)) {
        int brightness = (int)(255.0f * (1.0f - (distance / (NUM_LEDS_PER_STRIP / 2.0f))));
        uint8_t r = brightness;
        uint8_t g = brightness / 2;
        leds1[i] = CRGB(r,g,0);
        leds2[i] = CRGB(r,g,0);
      } else {
        leds1[i] = CRGB::Black;
        leds2[i] = CRGB::Black;
      }
    }
    pulsePosition++;
  } else {
    for (int i=0;i<NUM_LEDS_PER_STRIP;i++) {
      leds1[i] = CRGB::Black;
      leds2[i] = CRGB::Black;
    }
  }
}

void directionalFlowEffect(float dirX, float dirY) {
  static float flowPosition = 0.0f;
  float flowSpeed = dirX * 0.8f;
  flowPosition += flowSpeed;
  if (flowPosition >= NUM_LEDS_PER_STRIP) flowPosition -= NUM_LEDS_PER_STRIP;
  if (flowPosition < 0) flowPosition += NUM_LEDS_PER_STRIP;

  int hue = map((int)constrain(dirY * 50.0f, -180, 180), -180, 180, 0, 255);

  for (int i=0;i<NUM_LEDS_PER_STRIP;i++) {
    float distance = fabs((float)i - flowPosition);
    if (distance > NUM_LEDS_PER_STRIP/2.0f) distance = NUM_LEDS_PER_STRIP - distance;
    int brightness = (int)(255.0f * (1.0f - distance / (NUM_LEDS_PER_STRIP / 2.0f)));
    leds1[i] = CHSV(hue, 255, brightness);
    leds2[i] = CHSV(hue, 255, brightness);
  }
}

void energyMapSplitEffect(float diffX, float diffY, float diffZ, float totalAccel) {
  int red1   = constrain((int)(diffX * 100.0f), 0, 255);
  int green1 = constrain((int)(diffY * 100.0f), 0, 255);
  int blue1  = constrain((int)(diffZ * 100.0f), 0, 255);

  int red2   = constrain((int)(totalAccel * 150.0f), 0, 255);
  int green2 = constrain((int)(fabs(diffX - diffY) * 100.0f), 0, 255);
  int blue2  = 0;

  for (int i=0;i<NUM_LEDS_PER_STRIP;i++) {
    float pos = (float)i / (float)(NUM_LEDS_PER_STRIP - 1);
    uint8_t r1 = (uint8_t)((int)(red1 * pos));
    uint8_t g1 = (uint8_t)((int)(green1 * pos));
    uint8_t b1 = (uint8_t)((int)(blue1 * (1.0f - pos)));
    uint8_t r2 = (uint8_t)((int)(red2 * pos));
    uint8_t g2 = (uint8_t)((int)(green2 * (1.0f - pos)));
    uint8_t b2 = (uint8_t)blue2;

    leds1[i] = CRGB(r1, g1, b1);
    leds2[i] = CRGB(r2, g2, b2);
  }
}

