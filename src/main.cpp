#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <driver/gpio.h>

// ESP32-C3 Super Mini Pines
#define PIN_DATA 4 
#define PIN_CLOCK 5
#define PIN_SLEEP 3
#define PIN_BOTON 2
#define PIN_LED   7

// Debug: cambiar a 1 para activar prints por Serial
#define DEBUG_MODE 0

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

float lastDecodedValue = 0;
bool hasValidValue = false;

const int TOTAL_BITS = 24;
const unsigned long GAP_THRESHOLD = 30000; // 30 ms en microsegundos

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
      char buf[20];
      // Convertir a string con signo y 2 decimales, simulando el formato anterior
      snprintf(buf, sizeof(buf), "%+.2f\n", value); 
      
      pTxCharacteristic->setValue((uint8_t*)buf, strlen(buf));
      pTxCharacteristic->notify();
      DEBUG_PRINT("Enviado por BLE: ");
      DEBUG_PRINT(buf);
  } else {
      DEBUG_PRINTLN("Bluetooth no conectado.");
  }
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
  gpio_reset_pin((gpio_num_t)PIN_LED);

  // LED del botón encendido mientras el sistema está despierto
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);

  // Configuración de pines (Pullup interno OK: calibre es 3V, ESP32 es 3.3V)
  pinMode(PIN_CLOCK, INPUT_PULLUP);
  pinMode(PIN_DATA, INPUT_PULLUP);
  pinMode(PIN_SLEEP, INPUT_PULLUP); // Botón con pull-up interno

  // Comprobar si despertamos de Deep Sleep
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_GPIO) {
    DEBUG_PRINTLN("¡Sistema despertado por botón!");
  }

  // Configurar GPIO 3 para despertar al ESP32 cuando esté en LOW
  esp_deep_sleep_enable_gpio_wakeup(1ULL << PIN_SLEEP, ESP_GPIO_WAKEUP_GPIO_LOW);
  pinMode(PIN_BOTON, INPUT_PULLUP);

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
  BLEDevice::startAdvertising();
  DEBUG_PRINTLN("Esperando conexión...");
}

void loop()
{
  // Comprobación de botón Sleep/Wake
  if (digitalRead(PIN_SLEEP) == LOW) {
    delay(50); // Debounce
    if (digitalRead(PIN_SLEEP) == LOW) {
      DEBUG_PRINTLN("Botón Sleep presionado. Preparando para dormir...");
      
      // Esperar a que el usuario suelte el botón para evitar ciclos o despertares falsos
      while(digitalRead(PIN_SLEEP) == LOW) {
        delay(10);
      }
      delay(200); // Pequeño delay de seguridad eléctrica

      // Detener BLE (opcional pero recomendado)
      // BLEDevice::deinit(); 
      
      DEBUG_PRINTLN("Entrando en Deep Sleep. Pulsa el mismo botón para despertar.");
#if DEBUG_MODE
      Serial.flush();
#endif
      
      // Apagar LED antes de dormir
      digitalWrite(PIN_LED, LOW);

      // Asegurar que la resistencia pull-up se mantenga activa para el wakeup
      gpio_pullup_en((gpio_num_t)PIN_SLEEP);
      
      esp_deep_sleep_start();
    }
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

  // Verificar botón de envío (INDEPENDIENTE de frameReady)
  if (digitalRead(PIN_BOTON) == LOW)
  {
    delay(50); // Debounce
    if (digitalRead(PIN_BOTON) == LOW) {
      if (hasValidValue) {
        sendDataOverBluetooth(lastDecodedValue);
        DEBUG_PRINTLN("Dato enviado por Bluetooth.");
      } else {
        DEBUG_PRINTLN("Botón presionado, pero no hay datos del calibre aún.");
      }
      // Esperar a que se suelte el botón
      while (digitalRead(PIN_BOTON) == LOW) {
        delay(10);
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