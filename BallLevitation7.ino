//020426
#include <Wire.h>
#include <VL53L0X.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ====== PIN CONFIG ======
#define I2C_SDA        1
#define I2C_SCL        0
#define BLOWER_PWM_PIN 10

#define PWM_FREQ       500
#define PWM_RESOLUTION 8

// ====== BLE ======
BLECharacteristic *txCharacteristic;
BLECharacteristic *rxCharacteristic;

#define SERVICE_UUID "12345678-1234-1234-1234-123456789ab7"
#define TX_UUID      "abcd1234-5678-1234-5678-abcdef123457"
#define RX_UUID      "dcba4321-8765-4321-8765-fedcba654327"

bool deviceConnected = false;

// ====== SENSOR ======
VL53L0X sensor;

// ====== KALMAN FILTER ======
float x_est     = 0;
float P_kal     = 1;
float Q_kal     = 10;   // process noise
float R_kal     = 25;   // measurement noise
bool  kalmanInit = false;

float kalman(float measurement) {
  if (!kalmanInit) {
    x_est     = measurement;   // seed with first real reading
    kalmanInit = true;
  }
  P_kal     = P_kal + Q_kal;
  float K   = P_kal / (P_kal + R_kal);
  x_est     = x_est + K * (measurement - x_est);
  P_kal     = (1 - K) * P_kal;
  return x_est;
}

// ====== PID ======
struct PIDParams { float kp, ki, kd, setpoint; };
PIDParams pid = {1.2, 0.8, 1.5, 700};

float integral    = 0;
float prevPosition = 0;
float dFiltered   = 0;
float beta        = 0.7;

// ====== MODES ======
bool autoMode = true;
bool autoTune = false;
int  manualPWM = 0;

// ====== SOFT START (global so autotune can read hover PWM) ======
float smoothPWM = 0;

// ====== TIMING ======
unsigned long lastTime          = 0;
const int     SAMPLE_TIME_MS    = 30;
unsigned long lastBLESend       = 0;
const int     BLE_INTERVAL_MS   = 100;
unsigned long lastConnectAttempt = 0;

// ====== AUTOTUNE ======
float relayOffset   = 60;    // 60 ± PWM around hover point
float relayHigh     = 200;  //200
float relayLow      = 40; //40
float relayBias     = 0; //0
bool  relayState    = false;
bool  prevRelayState = false;
bool  autoTuneRunning = false;
float oscillationPeriod = 0;  //0
float maxVal        = 0;
float minVal        = 9999;
int   switchCount   = 0;
unsigned long cycleStart = 0;

// ====== BLE SERVER CALLBACKS ======
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *s) {
    deviceConnected = true;
    Serial.println("BLE connected");
  }
  void onDisconnect(BLEServer *s) {
    deviceConnected = false;
    Serial.println("BLE disconnected — restarting advertising");
    BLEDevice::getAdvertising()->start();
  }
};

// ====== BLE RX CALLBACKS ======
class RXCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) {
    String value = c->getValue().c_str();

    float kp, ki, kd, sp;
    int   mode, pwm, tune;

    int parsed = sscanf(value.c_str(), "%f,%f,%f,%f,%d,%d,%d",
                        &kp, &ki, &kd, &sp, &mode, &pwm, &tune);

    if (parsed == 7) {
      pid.kp       = kp;
      pid.ki       = ki;
      pid.kd       = kd;
      pid.setpoint = sp;
      autoMode     = (bool)mode;
      manualPWM    = pwm;
      autoTune     = (bool)tune;
    } else {
      Serial.printf("Bad BLE packet (%d fields): %s\n", parsed, value.c_str());
    }
  }
};

// ====== SETUP ======
void setup() {
  Serial.begin(115200);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  sensor.init();
  sensor.setMeasurementTimingBudget(30000);
  sensor.startContinuous();

  ledcAttach(BLOWER_PWM_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(BLOWER_PWM_PIN, 0);

  BLEDevice::init("BallLevitation_7");
  BLEServer  *server  = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());
  BLEService *service = server->createService(SERVICE_UUID);

  txCharacteristic = service->createCharacteristic(
    TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  txCharacteristic->addDescriptor(new BLE2902());

  rxCharacteristic = service->createCharacteristic(
    RX_UUID, BLECharacteristic::PROPERTY_WRITE);
  rxCharacteristic->setCallbacks(new RXCallbacks());

  service->start();
  BLEDevice::getAdvertising()->start();
  Serial.println("BLE advertising started");
  Serial.println("Setup voltooid");
}

// ====== LOOP ======
void loop() {
  unsigned long now = millis();
  static float position = 0;  // Maak position statisch en initialiseer
  static float dt = 0;        // Maak dt statisch en initialiseer
  static int finalPWM = 0;    // Maak finalPWM statisch en initialiseer

  // Watchdog: keep advertising if disconnected
  if (!deviceConnected && now - lastConnectAttempt > 10000) {
    lastConnectAttempt = now;
    BLEDevice::getAdvertising()->start();
  }

  if (now - lastTime >= SAMPLE_TIME_MS) {
    dt = (now - lastTime) / 1000.0;
    lastTime = now;

    // ====== SENSOR READ ======
    uint16_t raw = sensor.readRangeContinuousMillimeters();
    if (!sensor.timeoutOccurred() && raw > 30 && raw < 1200) {
      position = kalman(raw);
    } else {
      position = prevPosition;  // Gebruik vorige waarde als meting ongeldig is
    }

      // ====== AUTOTUNE ======
    if (autoTune) {
        // Autotune-logica hier (zie vorige voorbeelden)
        Serial.println("Autotune gestart via BLE!");//if (autoTune) {
        // Eerste iteratie: initialiseer autotune
        if (!autoTuneRunning) {
        autoTuneRunning = true;
        relayBias       = smoothPWM;
        relayHigh       = constrain(relayBias + 180, 0, 255);  // Verhoogde offset 180,0,255
        relayLow        = constrain(relayBias - 180, 0, 255);
        maxVal          = position;
        minVal          = position;
        switchCount     = 0;
        cycleStart      = 0;
        prevRelayState  = (position <= pid.setpoint);
        Serial.printf(
          "Autotune gestart: pos=%.1f, sp=%.1f, bias=%.0f, high=%.0f, low=%.0f\n",
          position, pid.setpoint, relayBias, relayHigh, relayLow
        );
        }

        // Relay-logica
        relayState = (position <= pid.setpoint);
        finalPWM   = relayState ? (int)relayHigh : (int)relayLow;

        // Bijwerk max/min waarden
        maxVal = max(maxVal, position);
        minVal = min(minVal, position);

        // Detecteer schakeling
        if (relayState != prevRelayState) {
          switchCount++;
          Serial.printf(
            "Schakeling %d: pos=%.1f, max=%.1f, min=%.1f\n",
            switchCount, position, maxVal, minVal
          );

        if (switchCount == 1) {
          cycleStart = millis();
          Serial.println("Start meting oscillatieperiode...");
        }

        // Controleer of we genoeg schakelingen en amplitude hebben
        if (switchCount >= 3 && (maxVal - minVal) > 10) {  // Verlaagde drempel
          oscillationPeriod = millis() - cycleStart;
          float amplitude   = maxVal - minVal;
          float Ku          = 4.0 * (relayHigh - relayLow) / (3.1415 * amplitude);
          float Tu          = oscillationPeriod / 1000.0;

          // Pas PID-parameters aan met Ziegler-Nichols
          pid.kp = 0.6  * Ku;
          pid.ki = 1.2  * Ku / Tu;
          pid.kd = 0.075 * Ku * Tu;

          Serial.printf(
            "Autotune voltooid: Ku=%.3f, Tu=%.2fs, Amp=%.1f\n"
            "Nieuwe PID: Kp=%.3f, Ki=%.3f, Kd=%.3f\n",
            Ku, Tu, amplitude, pid.kp, pid.ki, pid.kd
          );

          autoTune        = false;
          autoTuneRunning = false;
          autoMode        = true;
          integral        = 0;
        }
        //}

        prevRelayState = relayState;
        ledcWrite(BLOWER_PWM_PIN, finalPWM);
        smoothPWM = finalPWM;
        }
        // ...
      }
    // ====== AUTO PID ======
    else if (autoMode) {
      float error = pid.setpoint - position;

      if (abs(error) < 10) error = 0;
      if (abs(error) > 200) integral = 0;

      float Pout = pid.kp * error;

      float dRaw = (position - prevPosition) / dt;
      dFiltered = beta * dRaw + (1 - beta) * dFiltered;
      float D = -pid.kd * dFiltered;

      float output = Pout + pid.ki * integral + D;

      // Anti-windup: alleen integreren als output binnen bereik is
      if (output > 0 && output < 255) {
        integral += error * dt;
      }
      integral = constrain(integral, -80, 80);

      finalPWM = constrain((int)(Pout + pid.ki * integral + D), 0, 255);

      // Soft start
      smoothPWM = 0.7 * smoothPWM + 0.3 * finalPWM;
      if (smoothPWM > 0 && smoothPWM < 60) smoothPWM = 60;
      ledcWrite(BLOWER_PWM_PIN, (int)smoothPWM);
    }
    // ====== MANUAL ======
    else {
      finalPWM = manualPWM;
      smoothPWM = 0.7 * smoothPWM + 0.3 * finalPWM;
      if (smoothPWM > 0 && smoothPWM < 60) smoothPWM = 60;
      ledcWrite(BLOWER_PWM_PIN, (int)smoothPWM);
    }

    prevPosition = position;

    // ====== BLE SEND ======
    if (deviceConnected && now - lastBLESend >= BLE_INTERVAL_MS) {
      lastBLESend = now;
      String data = String(millis())       + "," +
                    String(position, 1)    + "," +
                    String(pid.setpoint, 1) + "," +
                    String(smoothPWM, 1)   + "," +
                    String(pid.kp, 3)      + "," +
                    String(pid.ki, 3)      + "," +
                    String(pid.kd, 3);
      Serial.print("BLE data: ");
      Serial.println(data);              
      txCharacteristic->setValue(data.c_str());
      txCharacteristic->notify();
    }
  }
}