#include <Wire.h> // Biblioteca I2C padrão do Arduino/ESP32
#include <Adafruit_NeoPixel.h> // Biblioteca para controlar as fitas LED WS2812B

// --- Configuração de Pinos ---
#define LED_PIN_1 13        // Pino GPIO para o sinal da fita LED 1
#define LED_PIN_2 14        // Pino GPIO para o sinal da fita LED 2
#define NUM_LEDS_PER_STRIP 75 // Número de LEDs em CADA fita (ALTERADO)
#define NUM_TOTAL_LEDS 150  // Número total de LEDs (ALTERADO)

// --- Configuração dos Acelerômetros MMA845X ---
#define ACCEL1_ADDRESS 0x1C // Exemplo: MMA845X com ADDR em GND
#define ACCEL2_ADDRESS 0x1D // Exemplo: MMA845X com ADDR em VCC

// --- Parâmetros do Acelerômetro ---
#define MMA845X_STATUS 0x00
#define MMA845X_OUT_X_MSB 0x01
#define MMA845X_OUT_Y_MSB 0x03
#define MMA845X_OUT_Z_MSB 0x05
#define MMA845X_WHO_AM_I 0x0D
#define MMA845X_CTRL_REG1 0x2A
#define MMA845X_CTRL_REG2 0x2B
#define MMA845X_XYZ_DATA_CFG 0x0E
#define MMA845X_FF_MT_CFG 0x15
#define MMA845X_PL_CFG 0x11
#define MMA845X_PL_STATUS 0x10
#define MMA845X_FF_MT_THS 0x17
#define MMA845X_FF_MT_COUNT 0x18

// --- Configurações de Filtro e Calibração ---
#define FILTER_WINDOW_SIZE 10
const int CALIBRATION_SAMPLES = 150; // Aproximadamente 3 segundos a 50Hz

// --- Estrutura para dados do sensor ---
struct SensorData {
  float bufferX[FILTER_WINDOW_SIZE];
  float bufferY[FILTER_WINDOW_SIZE];
  float bufferZ[FILTER_WINDOW_SIZE];
  int bufferIndex;
  float sumX, sumY, sumZ;
  float calibratedX, calibratedY, calibratedZ;
  float offsetX, offsetY, offsetZ;
};

// --- Instâncias dos sensores ---
SensorData accel1, accel2;

// --- Instâncias das fitas LED (ALTERADO) ---
Adafruit_NeoPixel strip1 = Adafruit_NeoPixel(NUM_LEDS_PER_STRIP, LED_PIN_1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2 = Adafruit_NeoPixel(NUM_LEDS_PER_STRIP, LED_PIN_2, NEO_GRB + NEO_KHZ800);

// --- Variáveis globais ---
bool calibrating = true;
unsigned long calibrationStartTime;
int calibrationCount = 0;

// --- Modos de exibição ---
enum DisplayMode {
  RAINBOW_WAVE,
  IMPACT_PULSE,
  DIRECTIONAL_FLOW,
  ENERGY_MAP_SPLIT
};
DisplayMode currentMode = RAINBOW_WAVE;

// --- Inicialização do Filtro ---
void initFilter(SensorData &sensor) {
  sensor.bufferIndex = 0;
  sensor.sumX = sensor.sumY = sensor.sumZ = 0;
  for (int i = 0; i < FILTER_WINDOW_SIZE; i++) {
    sensor.bufferX[i] = 0;
    sensor.bufferY[i] = 0;
    sensor.bufferZ[i] = 0;
  }
}

// --- Função para adicionar valor ao filtro de média móvel ---
float updateMovingAverage(float *buffer, float newValue, float &sum, int &index, int size) {
  sum -= buffer[index];
  buffer[index] = newValue;
  sum += newValue;
  index = (index + 1) % size; // Atualiza o índice localmente
  return sum / size;
}

// --- Inicialização do Acelerômetro MMA845X ---
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
  Serial.print("ID do sensor em 0x");
  Serial.print(address, HEX);
  Serial.print(": 0x");
  Serial.println(id, HEX);

  Wire.beginTransmission(address);
  Wire.write(MMA845X_CTRL_REG1);
  Wire.write(0x05); // Standby
  Wire.endTransmission(true);

  Wire.beginTransmission(address);
  Wire.write(MMA845X_XYZ_DATA_CFG);
  Wire.write(0x01); // ±4g
  Wire.endTransmission(true);

  Wire.beginTransmission(address);
  Wire.write(MMA845X_CTRL_REG1);
  Wire.write(0x05 | 0x01); // 12.5Hz + Ativo
  Wire.endTransmission(true);

  delay(10);
}

// --- Leitura dos dados do MMA845X ---
void readMMA845X(int address, float &x, float &y, float &z) {
  Wire.beginTransmission(address);
  Wire.write(MMA845X_OUT_X_MSB);
  Wire.endTransmission(false);
  Wire.requestFrom(address, 6);

  if (Wire.available() < 6) {
    Serial.println("Erro: Dados insuficientes do acelerometro.");
    x = y = z = 0;
    return;
  }

  int16_t rawX = (Wire.read() << 8 | Wire.read()) >> 4;
  int16_t rawY = (Wire.read() << 8 | Wire.read()) >> 4;
  int16_t rawZ = (Wire.read() << 8 | Wire.read()) >> 4;

  float scale_factor = 4.0 / 1024.0; // ±4g

  x = rawX * scale_factor;
  y = rawY * scale_factor;
  z = rawZ * scale_factor;
}

// --- Processamento de um sensor ---
// CORREÇÃO APLICADA: Removida a atualização duplicada de sensor.bufferIndex
void processSensor(int address, SensorData &sensor) {
  float rawX, rawY, rawZ;
  readMMA845X(address, rawX, rawY, rawZ);

  // Aplica o filtro de média móvel
  // Cada chamada abaixo atualiza sensor.bufferIndex uma vez (dentro de updateMovingAverage)
  float filteredX = updateMovingAverage(sensor.bufferX, rawX, sensor.sumX, sensor.bufferIndex, FILTER_WINDOW_SIZE);
  // REMOVIDO: sensor.bufferIndex = (sensor.bufferIndex + 1) % FILTER_WINDOW_SIZE;
  float filteredY = updateMovingAverage(sensor.bufferY, rawY, sensor.sumY, sensor.bufferIndex, FILTER_WINDOW_SIZE);
  // REMOVIDO: sensor.bufferIndex = (sensor.bufferIndex + 1) % FILTER_WINDOW_SIZE;
  float filteredZ = updateMovingAverage(sensor.bufferZ, rawZ, sensor.sumZ, sensor.bufferIndex, FILTER_WINDOW_SIZE);
  // REMOVIDO: sensor.bufferIndex = (sensor.bufferIndex + 1) % FILTER_WINDOW_SIZE;

  if (calibrating) {
    sensor.offsetX += filteredX;
    sensor.offsetY += filteredY;
    sensor.offsetZ += filteredZ;
    calibrationCount++;
  } else {
    sensor.calibratedX = filteredX - sensor.offsetX;
    sensor.calibratedY = filteredY - sensor.offsetY;
    sensor.calibratedZ = filteredZ - sensor.offsetZ;
  }
}

// --- Finaliza a calibração ---
void finishCalibration() {
  Serial.println("Calibracao concluida.");
  if (calibrationCount > 0) {
    accel1.offsetX /= calibrationCount;
    accel1.offsetY /= calibrationCount;
    accel1.offsetZ /= calibrationCount;

    accel2.offsetX /= calibrationCount;
    accel2.offsetY /= calibrationCount;
    accel2.offsetZ /= calibrationCount;
  }
  // Ajuste para remover a gravidade (1g) no eixo Z
  accel1.offsetZ -= 1.0;
  accel2.offsetZ -= 1.0;
  Serial.println("Offsets ajustados para remover 1g da gravidade Z.");
  calibrating = false;
}

// --- Função para atualizar ambas as fitas com os mesmos dados ---
void updateBothStrips(Adafruit_NeoPixel &s1, Adafruit_NeoPixel &s2) {
  s1.show();
  s2.show();
}

// --- Função para limpar ambas as fitas ---
void clearBothStrips(Adafruit_NeoPixel &s1, Adafruit_NeoPixel &s2) {
  s1.clear();
  s2.clear();
  updateBothStrips(s1, s2);
}

// --- Configuração Inicial ---
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Iniciando setup do ESP32 com MMA845X e 2 fitas de 75 LEDs...");
  Serial.print("Numero total de LEDs: ");
  Serial.println(NUM_TOTAL_LEDS);

  Wire.begin();
  Wire.setClock(400000);

  Serial.println("Configurando acelerometros...");
  setupMMA845X(ACCEL1_ADDRESS);
  setupMMA845X(ACCEL2_ADDRESS);

  Serial.println("Configurando fitas LED...");
  strip1.begin();
  strip2.begin();
  strip1.show();
  strip2.show();
  // IMPORTANTE: Reduza o brilho para evitar sobrecarga de corrente
  strip1.setBrightness(50); // Ajuste o brilho (0-255)
  strip2.setBrightness(50); // Reduza o brilho se a corrente for alta

  initFilter(accel1);
  initFilter(accel2);

  Serial.println("Iniciando calibracao. Mantenha o dispositivo parado e nivelado...");
  calibrationStartTime = millis();
  calibrating = true;
  calibrationCount = 0;

  // Preenche os buffers com zeros para começar de forma estável
  for(int i = 0; i < FILTER_WINDOW_SIZE; i++) {
      // Chamadas que atualizam os buffers e o índice
      updateMovingAverage(accel1.bufferX, 0, accel1.sumX, accel1.bufferIndex, FILTER_WINDOW_SIZE);
      updateMovingAverage(accel1.bufferY, 0, accel1.sumY, accel1.bufferIndex, FILTER_WINDOW_SIZE);
      updateMovingAverage(accel1.bufferZ, 0, accel1.sumZ, accel1.bufferIndex, FILTER_WINDOW_SIZE);
      updateMovingAverage(accel2.bufferX, 0, accel2.sumX, accel2.bufferIndex, FILTER_WINDOW_SIZE);
      updateMovingAverage(accel2.bufferY, 0, accel2.sumY, accel2.bufferIndex, FILTER_WINDOW_SIZE);
      updateMovingAverage(accel2.bufferZ, 0, accel2.sumZ, accel2.bufferIndex, FILTER_WINDOW_SIZE);
  }
  // Reinicia o índice global após o preenchimento inicial
  accel1.bufferIndex = 0;
  accel2.bufferIndex = 0;
}

// --- Loop Principal ---
void loop() {
  unsigned long loopStartTime = millis();

  processSensor(ACCEL1_ADDRESS, accel1);
  processSensor(ACCEL2_ADDRESS, accel2);

  if (calibrating) {
      if (calibrationCount >= CALIBRATION_SAMPLES) {
          finishCalibration();
      }
      return; // Sai do loop principal durante a calibração
  }

  float diffX = abs(accel1.calibratedX - accel2.calibratedX);
  float diffY = abs(accel1.calibratedY - accel2.calibratedY);
  float diffZ = abs(accel1.calibratedZ - accel2.calibratedZ);
  float totalAccel = sqrt(diffX*diffX + diffY*diffY + diffZ*diffZ);
  float directionX = (accel1.calibratedX + accel2.calibratedX) / 2.0;
  float directionY = (accel1.calibratedY + accel2.calibratedY) / 2.0;

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

  static unsigned long lastModeChange = 0;
  if (totalAccel > 1.0 && millis() - lastModeChange > 2000) {
    currentMode = (DisplayMode)((currentMode + 1) % 4);
    Serial.print("Modo alterado para: ");
    switch(currentMode) {
        case RAINBOW_WAVE: Serial.println("Rainbow Wave"); break;
        case IMPACT_PULSE: Serial.println("Impact Pulse"); break;
        case DIRECTIONAL_FLOW: Serial.println("Directional Flow"); break;
        case ENERGY_MAP_SPLIT: Serial.println("Energy Map Split"); break;
    }
    lastModeChange = millis();
    clearBothStrips(strip1, strip2); // Limpa ambas ao mudar de modo
  }

  // Ajuste a taxa de atualização considerando o tempo de 'show()' com mais LEDs
  int loopTime = millis() - loopStartTime;
  const int TARGET_LOOP_TIME = 25; // Ajuste se necessário (25ms = 40Hz)
  if (loopTime < TARGET_LOOP_TIME) {
    delay(TARGET_LOOP_TIME - loopTime);
  } else {
      // O loop demorou mais que o alvo, pode haver perda de fluidez
      // Serial.print("Loop demorado: "); // Descomente para depurar
      // Serial.print(loopTime);         // Descomente para depurar
      // Serial.println("ms");           // Descomente para depurar
  }
}

// ===== EFEITOS DE LUZ (Adaptados para 75 LEDs por fita) ===== //

void rainbowWaveEffect(float intensity, float dirX, float dirY) {
  int hue = map(constrain(dirY * 50, -180, 180), -180, 180, 0, 65535);
  int brightness = constrain(intensity * 100, 0, 255);

  for(int i=0; i<NUM_LEDS_PER_STRIP; i++) {
    int ledHue = (hue + i * 65536L / NUM_LEDS_PER_STRIP) % 65536;
    uint32_t color = strip1.gamma32(strip1.ColorHSV(ledHue, 255, brightness));
    strip1.setPixelColor(i, color);
    strip2.setPixelColor(i, color);
  }
  updateBothStrips(strip1, strip2);
}

void impactPulseEffect(float intensity) {
  static int pulsePosition = 0;
  static unsigned long lastPulse = 0;

  if (intensity > 0.3 && millis() - lastPulse > 300) {
    pulsePosition = 0;
    lastPulse = millis();
  }

  if (pulsePosition < NUM_LEDS_PER_STRIP * 2) {
    clearBothStrips(strip1, strip2);

    for (int i = 0; i < NUM_LEDS_PER_STRIP; i++) {
      float distance = abs(i - pulsePosition);
      if (distance < NUM_LEDS_PER_STRIP/2) {
        int brightness = (int)(255.0 * (1.0 - distance/(NUM_LEDS_PER_STRIP/2.0)));
        if (brightness > 0) {
            strip1.setPixelColor(i, strip1.Color(brightness, brightness/2, 0));
            strip2.setPixelColor(i, strip2.Color(brightness, brightness/2, 0));
        } else {
            strip1.setPixelColor(i, strip1.Color(0, 0, 0));
            strip2.setPixelColor(i, strip2.Color(0, 0, 0));
        }
      } else {
            strip1.setPixelColor(i, strip1.Color(0, 0, 0));
            strip2.setPixelColor(i, strip2.Color(0, 0, 0));
      }
    }
    updateBothStrips(strip1, strip2);
    pulsePosition++;
  } else {
      if (millis() - lastPulse > 350) {
          clearBothStrips(strip1, strip2);
      }
  }
}

void directionalFlowEffect(float dirX, float dirY) {
  static float flowPosition = 0;

  float flowSpeed = dirX * 0.8;
  flowPosition += flowSpeed;
  if (flowPosition >= NUM_LEDS_PER_STRIP) flowPosition = 0;
  if (flowPosition < 0) flowPosition = NUM_LEDS_PER_STRIP-1;

  for(int i=0; i<NUM_LEDS_PER_STRIP; i++) {
    float distance = abs(i - flowPosition);
    if (distance > NUM_LEDS_PER_STRIP/2) distance = NUM_LEDS_PER_STRIP - distance;

    int brightness = (int)(255.0 * (1.0 - distance/(NUM_LEDS_PER_STRIP/2.0)));
    int hue = map(constrain(dirY * 50, -180, 180), -180, 180, 0, 65535);
    uint32_t color = strip1.gamma32(strip1.ColorHSV(hue, 255, brightness));

    strip1.setPixelColor(i, color);
    strip2.setPixelColor(i, color);
  }
  updateBothStrips(strip1, strip2);
}

void energyMapSplitEffect(float diffX, float diffY, float diffZ, float totalAccel) {
  int red1   = constrain(diffX * 100, 0, 255);
  int green1 = constrain(diffY * 100, 0, 255);
  int blue1  = constrain(diffZ * 100, 0, 255);
  int red2   = constrain(totalAccel * 150, 0, 255);
  int green2 = constrain(abs(diffX - diffY), 0, 255); // Exemplo de combinação
  int blue2  = 0;

  for(int i=0; i<NUM_LEDS_PER_STRIP; i++) {
    float position = (float)i / (NUM_LEDS_PER_STRIP - 1);
    strip1.setPixelColor(i, strip1.Color(
      (int)(red1 * position),
      (int)(green1 * position),
      (int)(blue1 * (1.0 - position))
    ));
    strip2.setPixelColor(i, strip2.Color(
      (int)(red2 * position),
      (int)(green2 * (1.0 - position)),
      blue2
    ));
  }
  updateBothStrips(strip1, strip2);
}
