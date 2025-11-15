#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>

// --- Configuração de Pinos ---
#define LED_PIN_1 13        // Data pin fita LED 1
#define LED_PIN_2 14        // Data pin fita LED 2
#define NUM_LEDS_PER_STRIP 75
#define NUM_TOTAL_LEDS (NUM_LEDS_PER_STRIP * 2)

// I2C (Wire) usa por padrão SDA=21, SCL=22 no ESP32 quando Wire.begin() sem parâmetros

// --- Endereços MMA845x ---
#define ACCEL1_ADDRESS 0x1C
#define ACCEL2_ADDRESS 0x1D

// --- Registradores MMA845x ---
#define MMA845X_OUT_X_MSB 0x01
#define MMA845X_WHO_AM_I  0x0D
#define MMA845X_CTRL_REG1 0x2A
#define MMA845X_XYZ_DATA_CFG 0x0E

// --- Filtro / Calibração ---
#define USE_EMA true              // true = EMA (mais rápido). false = média móvel (mais estável)
#define EMA_ALPHA 0.25f           // 0.0 - 1.0 (maior = resposta mais rápida)
#define FILTER_WINDOW_SIZE 10     // usado apenas se USE_EMA == false
const int CALIBRATION_SAMPLES_PER_SENSOR = 150; // amostras por sensor durante calibração

// --- LEDs ---
Adafruit_NeoPixel strip1 = Adafruit_NeoPixel(NUM_LEDS_PER_STRIP, LED_PIN_1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2 = Adafruit_NeoPixel(NUM_LEDS_PER_STRIP, LED_PIN_2, NEO_GRB + NEO_KHZ800);
const uint8_t DEFAULT_BRIGHTNESS = 50; // 0-255

// --- Estrutura do sensor ---
struct SensorData {
  // Para média móvel (opcional)
  float bufferX[FILTER_WINDOW_SIZE];
  float bufferY[FILTER_WINDOW_SIZE];
  float bufferZ[FILTER_WINDOW_SIZE];
  int indexX, indexY, indexZ;
  float sumX, sumY, sumZ;

  // Para EMA
  float emaX, emaY, emaZ;

  // Calibração
  float offsetX, offsetY, offsetZ;
  int calibCount;

  // Resultado final
  float calibratedX, calibratedY, calibratedZ;

  // Inicialização
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
bool sensorsPresent = false;
unsigned long lastModeChange = 0;

// --- Modos de exibição ---
enum DisplayMode { RAINBOW_WAVE, IMPACT_PULSE, DIRECTIONAL_FLOW, ENERGY_MAP_SPLIT };
DisplayMode currentMode = RAINBOW_WAVE;

// --- Prototipos locais ---
void setupMMA845X(int address);
void readMMA845X(int address, float &x, float &y, float &z);
void processSensor(int address, SensorData &s);
float updateMovingAverage(float *buffer, float newValue, float &sum, int &index);
void finishCalibration();
void updateBothStrips();
void clearBothStrips();
void rainbowWaveEffect(float intensity, float dirX, float dirY);
void impactPulseEffect(float intensity);
void directionalFlowEffect(float dirX, float dirY);
void energyMapSplitEffect(float diffX, float diffY, float diffZ, float totalAccel);
void i2cScan();

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("=== ESP32 + 2x MMA845X + 2x WS2812B (75 LEDs cada) ===");

  // Inicializa sensores na struct
  accel1.init();
  accel2.init();

  // Inicializa I2C
  Wire.begin();           // SDA=21, SCL=22 por padrão no ESP32
  Wire.setClock(400000);  // 400kHz - ok, pode reduzir se instável

  // Scan I2C
  i2cScan();

  Serial.println("Configurando acelerometros...");
  setupMMA845X(ACCEL1_ADDRESS);
  setupMMA845X(ACCEL2_ADDRESS);

  // Inicializa fitas LED
  strip1.begin();
  strip2.begin();
  strip1.setBrightness(DEFAULT_BRIGHTNESS);
  strip2.setBrightness(DEFAULT_BRIGHTNESS);
  clearBothStrips();

  // Inicia calibração
  Serial.println("Iniciando calibracao. Mantenha o dispositivo parado e nivelado...");
  calibrationStart = millis();
  calibrating = true;
  accel1.calibCount = 0;
  accel2.calibCount = 0;

  // opcional: preenche buffers com zeros (para média móvel)
  if (!USE_EMA) {
    for (int i=0;i<FILTER_WINDOW_SIZE;i++) {
      updateMovingAverage(accel1.bufferX, 0.0f, accel1.sumX, accel1.indexX);
      updateMovingAverage(accel1.bufferY, 0.0f, accel1.sumX, accel1.indexY); // note: sum arg corrected below in update calls
      // however we'll re-init indices properly:
    }
    // reinicia indices
    accel1.indexX = accel1.indexY = accel1.indexZ = 0;
    accel2.indexX = accel2.indexY = accel2.indexZ = 0;
  }
}

// -------------------- LOOP --------------------
void loop() {
  unsigned long loopStart = millis();

  // Processa sensores (lê, filtra, acumula calibração ou produz valores calibrados)
  processSensor(ACCEL1_ADDRESS, accel1);
  processSensor(ACCEL2_ADDRESS, accel2);

  // Finaliza a calibração quando ambos sensores acumularam as amostras necessárias
  if (calibrating) {
    if (accel1.calibCount >= CALIBRATION_SAMPLES_PER_SENSOR && accel2.calibCount >= CALIBRATION_SAMPLES_PER_SENSOR) {
      finishCalibration();
    }
    // evita executar efeitos durante calibração
    return;
  }

  // Calcula diferenças/direções
  float diffX = fabs(accel1.calibratedX - accel2.calibratedX);
  float diffY = fabs(accel1.calibratedY - accel2.calibratedY);
  float diffZ = fabs(accel1.calibratedZ - accel2.calibratedZ);
  float totalAccel = sqrt(diffX*diffX + diffY*diffY + diffZ*diffZ);
  float directionX = (accel1.calibratedX + accel2.calibratedX) * 0.5f;
  float directionY = (accel1.calibratedY + accel2.calibratedY) * 0.5f;

  // Escolhe efeito
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
  static unsigned long lastModeChangeLocal = 0;
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

  // Chama show() uma vez por fita (após setar todos os pixels nos efeitos acima)
  strip1.show();
  strip2.show();

  // Controla taxa de atualização alvo (ajuste conforme necessário)
  const int TARGET_LOOP_TIME = 25; // ms (≈40Hz)
  int loopTime = millis() - loopStart;
  if (loopTime < TARGET_LOOP_TIME) delay(TARGET_LOOP_TIME - loopTime);
}

// -------------------- FUNÇÕES AUXILIARES --------------------

// I2C scanner simples
void i2cScan() {
  Serial.println("Procurando dispositivos I2C...");
  byte count = 0;
  for (byte addr=1; addr<127; ++addr) {
    Wire.beginTransmission(addr);
    byte err = Wire.endTransmission();
    if (err == 0) {
      Serial.print("Dispositivo I2C encontrado no endereco 0x");
      Serial.println(addr, HEX);
      count++;
    }
  }
  if (count == 0) Serial.println("Nenhum dispositivo I2C encontrado.");
  else Serial.print("Total encontrado: "), Serial.println(count);
}

// Inicializa MMA845X (modo standby -> config range -> ativa)
void setupMMA845X(int address) {
  Wire.beginTransmission(address);
  Wire.write(MMA845X_WHO_AM_I);
  Wire.endTransmission(false);
  Wire.requestFrom(address, 1);
  if (Wire.available() < 1) {
    Serial.print("Erro: Nenhum sensor encontrado no endereco I2C 0x");
    Serial.println(address, HEX);
    return;
  }
  uint8_t id = Wire.read();
  Serial.print("WHO_AM_I do sensor 0x");
  Serial.print(address, HEX);
  Serial.print(": 0x");
  Serial.println(id, HEX);

  // Coloca em standby (clear bit ACTIVE)
  Wire.beginTransmission(address);
  Wire.write(MMA845X_CTRL_REG1);
  Wire.write(0x00); // standby (simplificado)
  Wire.endTransmission(true);
  delay(5);

  // Configura ±4g (XYZ_DATA_CFG)
  Wire.beginTransmission(address);
  Wire.write(MMA845X_XYZ_DATA_CFG);
  Wire.write(0x01); // 0x01 = ±4g
  Wire.endTransmission(true);
  delay(5);

  // Ativa com taxa padrão (escreve ACTIVE bit = 1)
  Wire.beginTransmission(address);
  Wire.write(MMA845X_CTRL_REG1);
  Wire.write(0x01); // ACTIVE = 1 (mantendo demais bits padrão)
  Wire.endTransmission(true);
  delay(5);
}

// Leitura correta do bloco de 6 bytes e conversão para g (float)
void readMMA845X(int address, float &x, float &y, float &z) {
  Wire.beginTransmission(address);
  Wire.write(MMA845X_OUT_X_MSB);
  Wire.endTransmission(false);
  Wire.requestFrom(address, 6);

  if (Wire.available() < 6) {
    // Em caso de falha, retorna zeros
    x = y = z = 0.0f;
    return;
  }

  // Lê bytes em 16-bit, os 12 MSBs contêm os dados (right-shifted 4)
  int16_t rawX = (Wire.read() << 8) | Wire.read();
  int16_t rawY = (Wire.read() << 8) | Wire.read();
  int16_t rawZ = (Wire.read() << 8) | Wire.read();

  // Ajusta para 12-bit (mantendo sinal)
  rawX >>= 4;
  rawY >>= 4;
  rawZ >>= 4;

  // Força sign-extension para 12 bits (caso a plataforma não faça arithmetic shift como esperado)
  if (rawX & 0x0800) rawX |= 0xF000;
  if (rawY & 0x0800) rawY |= 0xF000;
  if (rawZ & 0x0800) rawZ |= 0xF000;

  // Escala correta para ±4g: LSB = 8g / 4096 = 0.001953125 g/LSB
  const float scale = 8.0f / 4096.0f; // = 0.001953125
  x = rawX * scale;
  y = rawY * scale;
  z = rawZ * scale;
}

// Média móvel (usada apenas se USE_EMA == false)
float updateMovingAverage(float *buffer, float newValue, float &sum, int &index) {
  sum -= buffer[index];
  buffer[index] = newValue;
  sum += newValue;
  int retIndex = index;
  index = (index + 1) % FILTER_WINDOW_SIZE;
  return sum / (float)FILTER_WINDOW_SIZE;
}

// Processa um sensor: lê, filtra (EMA ou média móvel), e acumula calibração
void processSensor(int address, SensorData &s) {
  float rawX, rawY, rawZ;
  readMMA845X(address, rawX, rawY, rawZ);

  float fx, fy, fz;
  if (USE_EMA) {
    // Inicializa EMA com a primeira amostra (se ainda zero)
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

  // Durante calibração, acumula offsets
  if (calibrating) {
    s.offsetX += fx;
    s.offsetY += fy;
    s.offsetZ += fz;
    s.calibCount++;
    return;
  }

  // Valores calibrados (subtrai offset médio)
  s.calibratedX = fx - s.offsetX;
  s.calibratedY = fy - s.offsetY;
  s.calibratedZ = fz - s.offsetZ;
}

// Finaliza calibração: divide offsets por amostras e subtrai gravidade no eixo Z
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

  // Ajusta 1g terrestre (opcional). Se os sensores estiverem alinhados verticalmente,
  // remover ~1.0g do offsetZ centraliza a leitura.
  const bool APPLY_GRAVITY_ADJUST = true;
  if (APPLY_GRAVITY_ADJUST) {
    accel1.offsetZ -= 1.0f;
    accel2.offsetZ -= 1.0f;
  }

  // Marca fim de calibração
  calibrating = false;
  Serial.println("Calibracao concluida.");
  Serial.print("Offsets sensor1: "); Serial.print(accel1.offsetX); Serial.print(", "); Serial.print(accel1.offsetY); Serial.print(", "); Serial.println(accel1.offsetZ);
  Serial.print("Offsets sensor2: "); Serial.print(accel2.offsetX); Serial.print(", "); Serial.print(accel2.offsetY); Serial.print(", "); Serial.println(accel2.offsetZ);
}

// Atualiza ambas as fitas (chamar show fora desta função apenas se necessário)
void updateBothStrips() {
  strip1.show();
  strip2.show();
}

void clearBothStrips() {
  strip1.clear();
  strip2.clear();
  strip1.show();
  strip2.show();
}

// -------------------- EFEITOS --------------------

void rainbowWaveEffect(float intensity, float dirX, float dirY) {
  // hue baseado em dirY, brightness baseado em intensity
  int hueBase = map((int)constrain(dirY * 50.0f, -180, 180), -180, 180, 0, 65535);
  int brightness = constrain((int)(intensity * 100.0f), 0, 255);

  for (int i=0;i<NUM_LEDS_PER_STRIP;i++) {
    int ledHue = (hueBase + (int)((uint32_t)i * (65536UL / NUM_LEDS_PER_STRIP))) % 65536;
    uint32_t color = strip1.gamma32(strip1.ColorHSV(ledHue, 255, brightness));
    strip1.setPixelColor(i, color);
    strip2.setPixelColor(i, color);
  }
  // show será chamado no loop() principal
}

void impactPulseEffect(float intensity) {
  static int pulsePosition = 0;
  static unsigned long lastPulse = 0;

  if (intensity > 0.35f && millis() - lastPulse > 300) {
    pulsePosition = 0;
    lastPulse = millis();
  }

  if (pulsePosition < NUM_LEDS_PER_STRIP * 2) {
    // Monta frame
    for (int i=0;i<NUM_LEDS_PER_STRIP;i++) {
      float distance = fabs((float)i - (float)pulsePosition);
      int r=0,g=0,b=0;
      if (distance < (NUM_LEDS_PER_STRIP / 2.0f)) {
        int brightness = (int)(255.0f * (1.0f - (distance / (NUM_LEDS_PER_STRIP / 2.0f))));
        r = brightness;
        g = brightness/2;
        b = 0;
      }
      strip1.setPixelColor(i, strip1.Color(r,g,b));
      strip2.setPixelColor(i, strip2.Color(r,g,b));
    }
    pulsePosition++;
  } else {
    // decai para apagado
    for (int i=0;i<NUM_LEDS_PER_STRIP;i++) {
      strip1.setPixelColor(i, 0);
      strip2.setPixelColor(i, 0);
    }
  }
}

void directionalFlowEffect(float dirX, float dirY) {
  static float flowPosition = 0.0f;

  float flowSpeed = dirX * 0.8f; // ajuste de sensibilidade
  flowPosition += flowSpeed;
  if (flowPosition >= NUM_LEDS_PER_STRIP) flowPosition -= NUM_LEDS_PER_STRIP;
  if (flowPosition < 0) flowPosition += NUM_LEDS_PER_STRIP;

  int hue = map((int)constrain(dirY * 50.0f, -180, 180), -180, 180, 0, 65535);

  for (int i=0;i<NUM_LEDS_PER_STRIP;i++) {
    float distance = fabs((float)i - flowPosition);
    if (distance > NUM_LEDS_PER_STRIP/2.0f) distance = NUM_LEDS_PER_STRIP - distance;
    int brightness = (int)(255.0f * (1.0f - distance / (NUM_LEDS_PER_STRIP / 2.0f)));
    uint32_t color = strip1.gamma32(strip1.ColorHSV(hue, 255, brightness));
    strip1.setPixelColor(i, color);
    strip2.setPixelColor(i, color);
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
    int r1 = (int)(red1 * pos);
    int g1 = (int)(green1 * pos);
    int b1 = (int)(blue1 * (1.0f - pos));
    int r2 = (int)(red2 * pos);
    int g2 = (int)(green2 * (1.0f - pos));
    int b2 = blue2;

    strip1.setPixelColor(i, strip1.Color(r1, g1, b1));
    strip2.setPixelColor(i, strip2.Color(r2, g2, b2));
  }
}
