#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <driver/gpio.h>
#include <esp_bt.h>

// ESP32-C3 Super Mini Pines
#define PIN_BAT_LVL 0
#define PIN_BOTON 2
#define PIN_LED   1
// PIN_SLEEP (GPIO 3) deprecado - ya no se usa
#define PIN_CLOCK 4
#define PIN_TARE 20
#define PIN_DATA 21 

// Debug: cambiar a 1 para activar prints por Serial
#define DEBUG_MODE 1

#if DEBUG_MODE
  #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
#endif

// BLE UUIDs - Nordic UART Service (Standard)
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" 
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer* pServer = NULL;
BLECharacteristic* pTxCharacteristic = NULL;
volatile bool deviceConnected = false;
volatile bool oldDeviceConnected = false;

volatile unsigned int rawData = 0;
volatile unsigned int bitCount = 0;
volatile unsigned long lastClock = 0;
volatile bool frameReady = false;
volatile unsigned long interruptCount = 0;

float batteryVoltage = 0.0f;
int batteryPercent = 0;
unsigned long lastBatteryReadMs = 0;

float lastDecodedValue = 0;
bool hasValidValue = false;

const int TOTAL_BITS = 24;
const unsigned long GAP_THRESHOLD = 30000; // 30 ms en microsegundos
const unsigned long BATTERY_READ_INTERVAL_MS = 60000;
const unsigned long LONG_PRESS_MS = 3000;
const unsigned long INACTIVITY_TIMEOUT_MS = 300000; // 5 minutos
const unsigned long TARE_PULSE_MS = 500;

unsigned long lastActivityMs = 0;

// Ajustes de lectura ADC de bateria.
// Si tu placa usa divisor resistivo para BAT_LVL, ajusta BATTERY_DIVIDER_RATIO.
const float ADC_REFERENCE_VOLTAGE = 3.3f;
const int ADC_MAX_READING = 4095;
const int BATTERY_SAMPLES = 16;
const float BATTERY_DIVIDER_RATIO = 2.0f;

struct BatteryPoint {
  float voltage;
  int percent;
};

// Curva base aproximada para celda Li-Ion 18650 en reposo (sin carga fuerte).
const BatteryPoint BATTERY_CURVE[] = {
  {4.20f, 100},
  {4.10f, 90},
  {4.00f, 80},
  {3.92f, 70},
  {3.85f, 60},
  {3.79f, 50},
  {3.73f, 40},
  {3.68f, 30},
  {3.62f, 20},
  {3.55f, 10},
  {3.30f, 0}
};
const int BATTERY_CURVE_SIZE = sizeof(BATTERY_CURVE) / sizeof(BATTERY_CURVE[0]);

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      DEBUG_PRINTLN("Dispositivo BLE conectado");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      DEBUG_PRINTLN("Dispositivo BLE desconectado");
    }
};

void setAwakeState(bool awake)
{
  // LED encendido mientras el equipo esta despierto.
  digitalWrite(PIN_LED, awake ? HIGH : LOW);
}

int estimateBatteryPercent18650(float voltage)
{
  if (voltage >= BATTERY_CURVE[0].voltage) {
    return BATTERY_CURVE[0].percent;
  }
  if (voltage <= BATTERY_CURVE[BATTERY_CURVE_SIZE - 1].voltage) {
    return BATTERY_CURVE[BATTERY_CURVE_SIZE - 1].percent;
  }

  for (int i = 0; i < BATTERY_CURVE_SIZE - 1; i++) {
    const BatteryPoint &high = BATTERY_CURVE[i];
    const BatteryPoint &low = BATTERY_CURVE[i + 1];

    if (voltage <= high.voltage && voltage >= low.voltage) {
      float ratio = (voltage - low.voltage) / (high.voltage - low.voltage);
      float percent = low.percent + ratio * (high.percent - low.percent);
      return constrain((int)(percent + 0.5f), 0, 100);
    }
  }

  return 0;
}

float readBatteryVoltage()
{
  uint32_t sum = 0;
  for (int i = 0; i < BATTERY_SAMPLES; i++) {
    sum += analogRead(PIN_BAT_LVL);
    delay(2);
  }

  float avgRaw = (float)sum / BATTERY_SAMPLES;
  float adcVoltage = (avgRaw / ADC_MAX_READING) * ADC_REFERENCE_VOLTAGE;
  return adcVoltage * BATTERY_DIVIDER_RATIO;
}

void updateBatteryStatus()
{
  batteryVoltage = readBatteryVoltage();
  batteryPercent = estimateBatteryPercent18650(batteryVoltage);

  DEBUG_PRINT("Bateria: ");
  DEBUG_PRINT(batteryVoltage, 2);
  DEBUG_PRINT(" V (");
  DEBUG_PRINT(batteryPercent);
  DEBUG_PRINTLN("%)");
}

float decodeFrame(uint32_t frame)
{
  // Bits 0-15 = magnitud (16 bits)
  uint32_t magnitude = frame & 0xFFFF;

  // Bit 21 = signo
  bool isNegative = (frame >> 21) & 0x01;

  // Bit 20 = unidades
  bool isInch = (frame >> 20) & 0x01;

  float value;
  if (isInch)
  {
    value = magnitude * 0.0005; // pasos en pulgadas
  }
  else
  {
    value = magnitude * 0.01; // pasos en mm
  }

  if (isNegative)
    value = -value;

  return value;
}

void sendDataOverBluetooth(float value)
{
  if (deviceConnected) {
      char buf[40];
      // Formato: medida,bateria%
      snprintf(buf, sizeof(buf), "%+.2f,%d\n", value, batteryPercent); 
      
      pTxCharacteristic->setValue((uint8_t*)buf, strlen(buf));
      pTxCharacteristic->notify();
      DEBUG_PRINT("Enviado por BLE: ");
      DEBUG_PRINT(buf);
  } else {
      DEBUG_PRINTLN("Bluetooth no conectado.");
  }
}

void executeTare()
{
  DEBUG_PRINTLN("Ejecutando TARE...");
  pinMode(PIN_TARE, OUTPUT);
  digitalWrite(PIN_TARE, LOW);
  delay(TARE_PULSE_MS);
  digitalWrite(PIN_TARE, HIGH);
  pinMode(PIN_TARE, INPUT_PULLUP);
  DEBUG_PRINTLN("TARE completado.");
}

void enterDeepSleep()
{
  DEBUG_PRINTLN("Entrando en Deep Sleep...");
#if DEBUG_MODE
  Serial.flush();
#endif

  setAwakeState(false);

  // Configurar PIN_BOTON para despertar en LOW
  gpio_pullup_en((gpio_num_t)PIN_BOTON);
  esp_deep_sleep_enable_gpio_wakeup(1ULL << PIN_BOTON, ESP_GPIO_WAKEUP_GPIO_LOW);

  esp_deep_sleep_start();
}

void IRAM_ATTR handleClock()
{
  interruptCount++;
  unsigned long now = micros();
  unsigned long delta = now - lastClock;
  lastClock = now;

  // Detectar comienzo de trama por timeout (~30 ms sin pulsos)
  if (delta > GAP_THRESHOLD)
  {
    bitCount = 0;
    rawData = 0;
  }

  // Leer un bit en cada flanco
  int bit = digitalRead(PIN_DATA);
  rawData |= ((uint32_t)bit << bitCount);
  bitCount++;

  if (bitCount >= TOTAL_BITS)
  {
    frameReady = true;
    bitCount = 0;
  }
}

void setup()
{
#if DEBUG_MODE
  Serial.begin(115200);
#endif
  DEBUG_PRINTLN("Iniciando ESP32-C3 Super Mini BLE...");

  // Desactivar JTAG en GPIO 4-7 para poder usarlos como GPIO normales
  gpio_reset_pin((gpio_num_t)PIN_CLOCK);
  gpio_reset_pin((gpio_num_t)PIN_DATA);

  // LED encendido desde el primer momento
  pinMode(PIN_LED, OUTPUT);
  setAwakeState(true);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_BAT_LVL, ADC_11db);

  // Configuración de pines (Pullup interno OK: calibre es 3V, ESP32 es 3.3V)
  pinMode(PIN_CLOCK, INPUT_PULLUP);
  pinMode(PIN_DATA, INPUT_PULLUP);
  pinMode(PIN_TARE, INPUT_PULLUP);
  pinMode(PIN_BOTON, INPUT_PULLUP);

  // Comprobar si despertamos de Deep Sleep
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_GPIO) {
    DEBUG_PRINTLN("¡Sistema despertado por botón!");
  }

  attachInterrupt(digitalPinToInterrupt(PIN_CLOCK), handleClock, RISING);
  lastClock = micros();
  DEBUG_PRINTLN("Interrupt configurada en PIN_CLOCK (GPIO 6) flanco RISING");

  // Configuración BLE
  BLEDevice::init("MPN+ BLE");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
                    
  pTxCharacteristic->addDescriptor(new BLE2902());

  pService->createCharacteristic(
                CHARACTERISTIC_UUID_RX,
                BLECharacteristic::PROPERTY_WRITE
              );

  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  
  // Reducir potencia BLE para ahorrar energía (~-3dBm, suficiente para corta distancia)
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_N3);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_N3);

  BLEDevice::startAdvertising();
  DEBUG_PRINTLN("Esperando conexión...");

  updateBatteryStatus();
  lastBatteryReadMs = millis();
  lastActivityMs = millis();
}

void loop()
{
  if (millis() - lastBatteryReadMs >= BATTERY_READ_INTERVAL_MS) {
    lastBatteryReadMs = millis();
    updateBatteryStatus();
  }

  // Auto deep sleep por inactividad (5 minutos sin presionar botón)
  if (millis() - lastActivityMs >= INACTIVITY_TIMEOUT_MS) {
    DEBUG_PRINTLN("Inactividad detectada. Entrando en deep sleep...");
    enterDeepSleep();
  }

  // Lógica de reconexión/publicidad
  if (!deviceConnected && oldDeviceConnected) {
      delay(500); 
      pServer->startAdvertising(); 
      DEBUG_PRINTLN("Reiniciando publicidad BLE...");
      oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
      oldDeviceConnected = deviceConnected;
  }

  // Procesar trama del calibre cuando esté lista
  if (frameReady)
  {
    noInterrupts();
    uint32_t frame = rawData;
    frameReady = false; 
    interrupts();

#if DEBUG_MODE
    Serial.print("Trama: ");
    for (int i = 0; i < TOTAL_BITS; i++)
    {
      Serial.print((frame >> i) & 1);
    }
    Serial.println();
#endif
    
    lastDecodedValue = decodeFrame(frame);
    hasValidValue = true;
    DEBUG_PRINT("Numero decodificado: ");
    DEBUG_PRINTLN(lastDecodedValue);
  }

  // Verificar botón: toque corto = enviar BLE, toque largo (>=5s) = tare
  if (digitalRead(PIN_BOTON) == LOW)
  {
    delay(50); // Debounce
    if (digitalRead(PIN_BOTON) == LOW) {
      lastActivityMs = millis();
      unsigned long pressStart = millis();

      // Esperar a que se suelte el botón
      while (digitalRead(PIN_BOTON) == LOW) {
        delay(10);
      }

      unsigned long pressDuration = millis() - pressStart;

      if (pressDuration >= LONG_PRESS_MS) {
        // Toque largo: ejecutar tare
        executeTare();
      } else {
        // Toque corto: enviar medida + batería por BLE
        if (hasValidValue) {
          sendDataOverBluetooth(lastDecodedValue);
          DEBUG_PRINTLN("Dato enviado por Bluetooth.");
        } else {
          DEBUG_PRINTLN("Botón presionado, pero no hay datos del calibre aún.");
        }
      }
      delay(200); // Anti-rebote adicional
    }
  }

#if DEBUG_MODE
  // --- DEBUG: Estado cada 2 segundos ---
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 2000) {
    lastDebug = millis();
    Serial.print("[DEBUG] IRQs: ");
    Serial.print(interruptCount);
    Serial.print(" | bits: ");
    Serial.print(bitCount);
    Serial.print(" | CLK=");
    Serial.print(digitalRead(PIN_CLOCK));
    Serial.print(" DATA=");
    Serial.print(digitalRead(PIN_DATA));
    Serial.print(" BTN=");
    Serial.println(digitalRead(PIN_BOTON));

    // Muestreo rápido: leer clock 1000 veces para detectar si cambia
    int changes = 0;
    int lastVal = digitalRead(PIN_CLOCK);
    for (int i = 0; i < 1000; i++) {
      int val = digitalRead(PIN_CLOCK);
      if (val != lastVal) { changes++; lastVal = val; }
    }
    Serial.print("[DEBUG] Cambios CLK en 1000 muestras: ");
    Serial.println(changes);

    changes = 0;
    lastVal = digitalRead(PIN_DATA);
    for (int i = 0; i < 1000; i++) {
      int val = digitalRead(PIN_DATA);
      if (val != lastVal) { changes++; lastVal = val; }
    }
    Serial.print("[DEBUG] Cambios DATA en 1000 muestras: ");
    Serial.println(changes);
  }
#endif
}