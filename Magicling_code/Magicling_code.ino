#include <Wire.h>             // I2C通信を行うための標準ライブラリ
#include <Adafruit_BNO055.h>  // BNO055 9軸センサ用ライブラリ
#include <utility/imumaths.h> // ベクトルやクォータニオン演算用のユーティリティ
#include <BluetoothSerial.h>
#include <Update.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_gap_ble_api.h>
BluetoothSerial SerialBT; // Bluetooth SPP通信用

#if defined(ARDUINO_ARCH_ESP32)
#include <Arduino.h>
                          // ESP32 Arduino コアのメジャーバージョンで分岐
#if ESP_ARDUINO_VERSION_MAJOR >= 3
#define PWM_INIT(pin, freq, res, chan) ledcAttach((pin), (freq), (res))
#define PWM_WRITE(pin, chan, value) ledcWrite((pin), (value))
#else
#define PWM_INIT(pin, freq, res, chan) \
  do                                   \
  {                                    \
    ledcSetup((chan), (freq), (res));  \
    ledcAttachPin((pin), (chan));      \
  } while (0)
#define PWM_WRITE(pin, chan, value) ledcWrite((chan), (value))
#endif
#else
#error "This sketch is for ESP32 only."
#endif

// ==== BNO055センサのインスタンス生成 ====
// 引数: センサID(任意), I2Cアドレス
Adafruit_BNO055 bno(55, 0x29);

const int LED_PIN = 22;

// 1P,2Pの指定
String name = "1P";

// PWM設定
const int motorPin = 25;     // 振動モータの制御ピン（GPIO25））
const int pwmFreq = 200;     // PWM周波数（Hz）※振動モータは低めでOK
const int pwmResolution = 8; // 分解能（8bit -> 0〜255）
const int pwmChannel = 0;

// BLE OTA UUIDs: compatible with RemoteCompilerToMicon WebAppSide.
#define DEBUG_SERVICE_UUID "7f3f0001-6b7c-4f2e-9b8a-1a2b3c4d5e6f"
#define DEBUG_LOG_TX_UUID "7f3f0002-6b7c-4f2e-9b8a-1a2b3c4d5e6f"
#define DEBUG_CMD_RX_UUID "7f3f0003-6b7c-4f2e-9b8a-1a2b3c4d5e6f"
#define DEBUG_STAT_UUID "7f3f0005-6b7c-4f2e-9b8a-1a2b3c4d5e6f"
#define OTA_SERVICE_UUID "9f5f0001-8d9e-6f4e-bd0c-3c4d5e6f7180"
#define OTA_CONTROL_UUID "9f5f0002-8d9e-6f4e-bd0c-3c4d5e6f7180"
#define OTA_DATA_UUID "9f5f0003-8d9e-6f4e-bd0c-3c4d5e6f7180"
#define OTA_STATUS_UUID "9f5f0004-8d9e-6f4e-bd0c-3c4d5e6f7180"
#define GAME_SERVICE_UUID "12345678-1234-1234-1234-123456789abc"
#define GAME_SENSOR_CHAR_UUID "12345678-1234-1234-1234-123456789abd"
#define GAME_HAPTIC_CHAR_UUID "12345678-1234-1234-1234-123456789abe"

BLEServer *otaServer = nullptr;
BLECharacteristic *debugLogCharacteristic = nullptr;
BLECharacteristic *debugStatCharacteristic = nullptr;
BLECharacteristic *otaStatusCharacteristic = nullptr;
BLECharacteristic *gameSensorCharacteristic = nullptr;
enum BlePairingMode
{
  BLE_PAIRING_OTA,
  BLE_PAIRING_GAME,
};
BlePairingMode blePairingMode = BLE_PAIRING_OTA;
bool bleClientConnected = false;
bool bleOtaConnected = false;
bool otaPairingLocked = false;
bool otaModeActive = false;
bool otaInProgress = false;
bool otaFinalizeRequested = false;
bool otaAbortRequested = false;
size_t otaExpectedSize = 0;
size_t otaReceivedSize = 0;
size_t otaLastReportedSize = 0;
unsigned long otaModeStartedMs = 0;
unsigned long otaLastActivityMs = 0;
const unsigned long OTA_MODE_TIMEOUT_MS = 60000;
const unsigned long OTA_TRANSFER_IDLE_TIMEOUT_MS = 30000;
const unsigned long OTA_PAIRING_WINDOW_MS = 10000;
const unsigned long GAME_NOTIFY_INTERVAL_MS = 20;
unsigned long lastGameNotifyMs = 0;
uint8_t directHapticStrength = 0;
unsigned long directHapticUntilMs = 0;
bool pwmReady = false;
uint8_t sensorSequence = 0;

int in = '0', in0 = '0'; // シリアル入力値（前回値と現在値） 初期値は'0'
int l = 0;               // メインループカウンタ

// ==== ローパスフィルタ用変数 ====
// alpha: 過去データの残す割合（0.0〜1.0）
float alpha = 0.7;
float ax_f = 0, ay_f = 0, az_f = 0; // ローパス適用後の加速度

// ==== 手動Yawゼロ設定 ====
// 自動ジャンプ補償は行わず、'5'入力時のゼロ設定だけに使用する。
float yaw_offset = 0;

// roll:ロール角
// pitch:ピッチ角
float roll = 0, pitch = 0, pitch_pre = 0;

// ==== 時間計測用変数 ====
unsigned long prevMicros = 0; // 前回のループ開始時刻（µs）

// ==== センサー取得結果を保持する変数 ====
// linAcc: ローカル座標系の線形加速度（重力成分除去済み）
// euler: オイラー角（Yaw, Roll, Pitch）
// mag: 磁気ベクトル
// quat: クォータニオン姿勢
imu::Vector<3> linAcc, euler, mag;
imu::Quaternion quat;
imu::Quaternion quat_corrected;

// ==== ユーティリティ関数 ====
// ±180°の範囲に角度を正規化する（何度でも繰り返して収める）
float normalize180(float angle)
{
  while (angle > 180.0f)
    angle -= 360.0f;
  while (angle <= -180.0f)
    angle += 360.0f;
  return angle;
}

float yawFromQuaternion(const imu::Quaternion &q)
{
  float qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();
  float yaw_deg = atan2f(2.0f * (qx * qy + qz * qw),
                         1.0f - 2.0f * (qy * qy + qz * qz)) *
                  180.0f / PI;
  return normalize180(yaw_deg);
}

imu::Quaternion yawCorrectionQuaternion(float yaw_deg)
{
  float yaw_rad = yaw_deg * PI / 180.0f;
  imu::Quaternion correction(cosf(yaw_rad * 0.5f), 0.0f, 0.0f, sinf(yaw_rad * 0.5f));
  correction.normalize();
  return correction;
}

imu::Quaternion normalizedForSend(imu::Quaternion q)
{
  q.normalize();
  if (q.w() < 0.0f)
  {
    q = q * -1.0;
  }
  return q;
}

imu::Quaternion applyYawCorrection(const imu::Quaternion &source, float offset_deg)
{
  imu::Quaternion corrected = yawCorrectionQuaternion(-offset_deg) * source;
  return normalizedForSend(corrected);
}

// ==== センサー値の取得とローパスフィルタ適用 ====
// BNO055から加速度・姿勢情報を取得し、加速度はローパス処理
void readSensors()
{
  // センサーから各種データを取得
  linAcc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  quat = bno.getQuat();
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  // 加速度値をg単位に変換しローパスフィルタ適用
  float ax = linAcc.x() / 9.8;
  float ay = linAcc.y() / 9.8;
  float az = linAcc.z() / 9.8;
  ax_f = alpha * ax_f + (1 - alpha) * ax;
  ay_f = alpha * ay_f + (1 - alpha) * ay;
  az_f = alpha * az_f + (1 - alpha) * az;
}

// ==== クォータニオン→回転行列変換し、加速度をグローバル座標系に変換 ====
// 出力: ax_global, ay_global, az_global
void calcGlobalAcceleration(float &ax_global, float &ay_global, float &az_global)
{
  // クォータニオンから回転行列を生成
  float qw = quat.w(), qx = quat.x(), qy = quat.y(), qz = quat.z();
  float R[3][3] = {
      {1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)},
      {2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)},
      {2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)}};
  // ローカル加速度をグローバル座標に変換
  ax_global = R[0][0] * ax_f + R[0][1] * ay_f + R[0][2] * az_f;
  ay_global = R[1][0] * ax_f + R[1][1] * ay_f + R[1][2] * az_f;
  az_global = R[2][0] * ax_f + R[2][1] * ay_f + R[2][2] * az_f;
}

void updaterollandpitch()
{
  pitch_pre = pitch;

  // クォータニオンから回転行列を生成
  float qw = quat.w(), qx = quat.x(), qy = quat.y(), qz = quat.z();
  float R[3][3] = {
      {1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)},
      {2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)},
      {2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)}};

  // ZYX（yaw-pitch-roll）系の式で roll/pitch を算出
  // roll（X軸回り）は非中間角のため -180..180 を取り得る
  float r21 = R[2][1]; // R[3][2]
  float r22 = R[2][2]; // R[3][3]
  float r20 = R[2][0]; // R[3][1]

  // roll = atan2(R32, R33) [deg]
  pitch = atan2f(r21, r22) * 180.0f / PI;

  // pitch = atan2(-R31, sqrt(R32^2 + R33^2)) [deg]
  // asin よりも数値的に安定（ゼロ割れや誤差増幅を抑制）
  float denom = sqrtf(r21 * r21 + r22 * r22);
  roll = atan2f(-r20, denom) * 180.0f / PI;

  // 表示範囲を整える
  roll = -normalize180(roll);
  pitch = -normalize180(pitch);

  if (abs(roll) >= 90)
  {
    pitch = pitch - 180;
  }
}

// 加速度ベクトルの大きさに基づいて振動モータを制御する関数
void Vibration(float ax_global, float ay_global, float az_global)
{
  if (millis() < directHapticUntilMs)
  {
    PWM_WRITE(motorPin, pwmChannel, directHapticStrength);
    return;
  }
  else if (directHapticStrength != 0)
  {
    directHapticStrength = 0;
    PWM_WRITE(motorPin, pwmChannel, 0);
  }

  in0 = in;
  // 入力チェック: Bluetooth優先、なければUSB Serial
  if (SerialBT.available() > 0)
  {
    in = SerialBT.read();
  }
  else if (Serial.available() > 0)
  {
    in = Serial.read();
  }
  else
  {
    in = in0;
  }

  // '5'へ切り替わった瞬間だけ、その時点のYawを0にする。
  // 押下後に毎ループYawを再計算すると、±90°特異点を再び通るため連続実行しない。
  if (in == '5' && in0 != '5')
  {
    // 手動操作時だけQuaternionからYawを求める。
    float yaw_raw = yawFromQuaternion(quat);
    yaw_offset = yaw_raw;
  }

  // 想定外の文字は無視して直前の値に戻す
  if (in < '0' || in > '5')
    in = in0;

  // '0' が送られてきたらアナログ振動モード
  if (in == '0')
  {
    // グローバル座標系での加速度ベクトルの大きさ（重力込み）を計算
    // √(ax^2 + ay^2 + az^2) → 全方向の加速度の合成値
    float a_pure = sqrt(ax_global * ax_global +
                        ay_global * ay_global +
                        az_global * az_global);

    // 加速度の値を 3 乗して感度を調整し、スケーリング係数300を掛けて PWM 値に変換
    // 3乗することで小さい加速度変化に対して感度を下げ、大きい加速度で急に強くなるカーブになる
    int vib = a_pure * 40;

    // 上限値を 250 に制限（PWM 8bit の最大255に近い値）
    if (vib > 250)
      vib = 250;
    // 下限閾値60未満はモータ停止（物理的に動かない領域をカット）
    else if (vib < 10)
      vib = 0;

    // PWM出力で振動モータを駆動
    PWM_WRITE(motorPin, pwmChannel, vib);
  }
  else
  {
    int flashRate = 0;
    float flashRate_deno = 0;
    if (in == '1')
    {
      flashRate = 20;
      flashRate_deno = 0.4;
    }
    else if (in == '2')
    {
      flashRate = 40;
      flashRate_deno = 0.2;
    }
    else if (in == '3')
    {
      flashRate = 80;
      flashRate_deno = 0.125;
    }
    else if (in == '4')
    {
      flashRate = 100;
      flashRate_deno = 0.9;
    }
    // '5' またはその他は安全に停止（flashRate=0のまま）
    if (flashRate > 0)
    {
      bool motorOn = (l % flashRate < (int)(flashRate * flashRate_deno));
      PWM_WRITE(motorPin, pwmChannel, motorOn ? 255 : 0);
    }
    else
    {
      PWM_WRITE(motorPin, pwmChannel, 0);
    }
  }
}

// ===============================
// データ送信（シリアル通信）
// ===============================
// Send seven values over Bluetooth:
//  - Accel ax, ay, az as fixed-point (2 decimals): value * 100 -> int16
//  - Quaternion qw, qx, qy, qz as fixed-point (4 decimals): value * 10000 -> int16
void buildSensorPayload(uint8_t *payload, float ax_global, float ay_global, float az_global, const imu::Quaternion &quat_val)
{
  int16_t ax_to_send = (int16_t)roundf(ax_global * 100.0f);
  int16_t ay_to_send = (int16_t)roundf(ay_global * 100.0f);
  int16_t az_to_send = (int16_t)roundf(az_global * 100.0f);
  imu::Quaternion q = normalizedForSend(quat_val);
  int16_t qw_to_send = (int16_t)roundf(q.w() * 10000.0f);
  int16_t qx_to_send = (int16_t)roundf(q.x() * 10000.0f);
  int16_t qy_to_send = (int16_t)roundf(q.y() * 10000.0f);
  int16_t qz_to_send = (int16_t)roundf(q.z() * 10000.0f);

  memcpy(payload + 0 * sizeof(int16_t), &ax_to_send, sizeof(int16_t));
  memcpy(payload + 1 * sizeof(int16_t), &ay_to_send, sizeof(int16_t));
  memcpy(payload + 2 * sizeof(int16_t), &az_to_send, sizeof(int16_t));
  memcpy(payload + 3 * sizeof(int16_t), &qw_to_send, sizeof(int16_t));
  memcpy(payload + 4 * sizeof(int16_t), &qx_to_send, sizeof(int16_t));
  memcpy(payload + 5 * sizeof(int16_t), &qy_to_send, sizeof(int16_t));
  memcpy(payload + 6 * sizeof(int16_t), &qz_to_send, sizeof(int16_t));
}

void buildSerialSensorFrame(uint8_t *frame, float ax_global, float ay_global, float az_global, const imu::Quaternion &quat_val)
{
  frame[0] = 'S';
  buildSensorPayload(frame + 1, ax_global, ay_global, az_global, quat_val);
}

void buildGameSensorFrame(uint8_t *frame, float ax_global, float ay_global, float az_global, const imu::Quaternion &quat_val)
{
  frame[0] = 'S';
  frame[1] = sensorSequence++;
  buildSensorPayload(frame + 2, ax_global, ay_global, az_global, quat_val);
  frame[16] = 0;
}

void outputDataAsBytes(float ax_global, float ay_global, float az_global, const imu::Quaternion &quat_val)
{
  uint8_t frame[1 + sizeof(int16_t) * 7];
  buildSerialSensorFrame(frame, ax_global, ay_global, az_global, quat_val);

  SerialBT.write(frame, sizeof(frame));

  unsigned long nowMs = millis();
  if (bleOtaConnected && gameSensorCharacteristic && !otaInProgress &&
      nowMs - lastGameNotifyMs >= GAME_NOTIFY_INTERVAL_MS)
  {
    lastGameNotifyMs = nowMs;
    uint8_t gameFrame[17];
    buildGameSensorFrame(gameFrame, ax_global, ay_global, az_global, quat_val);
    gameSensorCharacteristic->setValue(gameFrame, sizeof(gameFrame));
    gameSensorCharacteristic->notify();
  }
}

// ==== セットアップ処理 ====
// ハードウェア初期化、BNO055設定、M5Stack画面初期化
void notifyOtaStatus(const char *status)
{
  if (!otaStatusCharacteristic)
    return;

  otaStatusCharacteristic->setValue((uint8_t *)status, strlen(status));
  otaStatusCharacteristic->notify();
}

void notifyDebugLog(const char *message)
{
  if (!bleOtaConnected || !debugLogCharacteristic || otaInProgress)
    return;

  debugLogCharacteristic->setValue((uint8_t *)message, strlen(message));
  debugLogCharacteristic->notify();
}

void updateDebugStatus()
{
  if (!debugStatCharacteristic)
    return;

  char status[64];
  snprintf(status, sizeof(status), "STATE:BLE=%d,WIFI=0,OTA_MODE=%d,IP=0.0.0.0",
           bleOtaConnected ? 1 : 0,
           otaModeActive ? 1 : 0);
  debugStatCharacteristic->setValue((uint8_t *)status, strlen(status));
  if (bleOtaConnected)
    debugStatCharacteristic->notify();
}

String getGameBleName()
{
  return "nIpxel_" + name;
}

String getOtaBleName()
{
  return "ESP32-" + getGameBleName();
}

void startBleAdvertising(BlePairingMode mode)
{
  blePairingMode = mode;

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->stop();
  delay(20);

  String bleName = (mode == BLE_PAIRING_OTA) ? getOtaBleName() : getGameBleName();
  esp_ble_gap_set_device_name(bleName.c_str());

  BLEAdvertisementData advertisementData;
  advertisementData.setFlags(0x06);
  advertisementData.setCompleteServices(BLEUUID(mode == BLE_PAIRING_OTA ? OTA_SERVICE_UUID : GAME_SERVICE_UUID));
  if (mode == BLE_PAIRING_GAME)
    advertisementData.setName(bleName.c_str());

  BLEAdvertisementData scanResponseData;
  scanResponseData.setName(bleName.c_str());

  advertising->setAdvertisementData(advertisementData);
  advertising->setScanResponseData(scanResponseData);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMaxPreferred(0x12);
  advertising->start();

  Serial.printf("[BLE] Advertising %s as %s\n", mode == BLE_PAIRING_OTA ? "OTA" : "GAME", bleName.c_str());
}

void updateBlePairingMode()
{
  if (bleClientConnected || otaInProgress || otaFinalizeRequested)
    return;

  unsigned long nowMs = millis();
  bool otaWindowExpired = nowMs - otaModeStartedMs >= OTA_PAIRING_WINDOW_MS;
  bool otaIdleExpired = otaPairingLocked && (nowMs - otaLastActivityMs >= OTA_MODE_TIMEOUT_MS);

  if (blePairingMode == BLE_PAIRING_OTA && otaWindowExpired && (!otaPairingLocked || otaIdleExpired))
  {
    otaModeActive = false;
    otaPairingLocked = false;
    startBleAdvertising(BLE_PAIRING_GAME);
    updateDebugStatus();
  }
}

class MagiclingOtaServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *server)
  {
    bleClientConnected = true;
    bleOtaConnected = true;
    otaLastActivityMs = millis();
    if (blePairingMode == BLE_PAIRING_OTA)
    {
      otaPairingLocked = true;
      otaModeActive = true;
    }
    notifyDebugLog("[BLE] Magicling connected");
    updateDebugStatus();
  }

  void onDisconnect(BLEServer *server)
  {
    bleClientConnected = false;
    bleOtaConnected = false;
    otaLastActivityMs = millis();
    if (otaInProgress || blePairingMode == BLE_PAIRING_OTA)
      startBleAdvertising(BLE_PAIRING_OTA);
    else
      startBleAdvertising(BLE_PAIRING_GAME);
  }
};

class MagiclingDebugCommandCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *characteristic)
  {
    auto rxValue = characteristic->getValue();
    if (rxValue.length() == 0)
      return;

    String command(rxValue.c_str());
    command.trim();

    if (command == "OTA_MODE")
    {
      otaModeActive = true;
      otaPairingLocked = true;
      otaLastActivityMs = millis();
      notifyDebugLog("[OTA] OTA mode enabled");
      updateDebugStatus();
    }
    else if (command == "STATUS")
    {
      updateDebugStatus();
    }
    else if (command == "ABORT")
    {
      otaAbortRequested = true;
    }
  }
};

class GameHapticCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *characteristic)
  {
    size_t len = characteristic->getLength();
    if (len < 2)
      return;

    uint8_t *data = characteristic->getData();
    directHapticStrength = data[0];
    uint16_t durationMs = (uint16_t)data[1] * 10;

    if (directHapticStrength == 0 || durationMs == 0)
    {
      directHapticStrength = 0;
      directHapticUntilMs = 0;
      if (pwmReady)
        PWM_WRITE(motorPin, pwmChannel, 0);
      return;
    }

    directHapticUntilMs = millis() + durationMs;
    if (pwmReady)
      PWM_WRITE(motorPin, pwmChannel, directHapticStrength);
  }
};

class MagiclingOtaControlCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *characteristic)
  {
    auto rxValue = characteristic->getValue();
    if (rxValue.length() == 0)
      return;

    String command(rxValue.c_str());
    command.trim();

    if (command.startsWith("START:"))
    {
      otaLastActivityMs = millis();
      size_t size = command.substring(6).toInt();
      size_t updateSpace = ESP.getFreeSketchSpace() & 0xFFFFF000;
      if (size == 0 || size > updateSpace)
      {
        notifyOtaStatus("ERROR:INVALID_SIZE");
        return;
      }

      otaExpectedSize = size;
      otaReceivedSize = 0;
      otaLastReportedSize = 0;
      otaFinalizeRequested = false;
      otaAbortRequested = false;
      otaModeActive = true;
      otaPairingLocked = true;

      if (!Update.begin(size, U_FLASH))
      {
        Update.printError(Serial);
        otaInProgress = false;
        notifyOtaStatus("ERROR:BEGIN_FAILED");
        return;
      }

      otaInProgress = true;
      PWM_WRITE(motorPin, pwmChannel, 0);
      notifyOtaStatus("READY");
      Serial.printf("[BLE OTA] START:%u\n", size);
    }
    else if (command == "END")
    {
      otaLastActivityMs = millis();
      if (!otaInProgress)
      {
        notifyOtaStatus("ERROR:NOT_STARTED");
        return;
      }

      if (otaReceivedSize != otaExpectedSize)
      {
        notifyOtaStatus("ERROR:INCOMPLETE");
        Serial.printf("[BLE OTA] Incomplete: %u / %u\n", otaReceivedSize, otaExpectedSize);
        return;
      }

      otaFinalizeRequested = true;
    }
    else if (command == "ABORT")
    {
      otaLastActivityMs = millis();
      otaAbortRequested = true;
    }
  }
};

class MagiclingOtaDataCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *characteristic)
  {
    if (!otaInProgress)
      return;

    size_t len = characteristic->getLength();
    if (len == 0)
      return;
    otaLastActivityMs = millis();

    if (otaReceivedSize + len > otaExpectedSize)
    {
      Update.abort();
      otaInProgress = false;
      notifyOtaStatus("ERROR:OVERFLOW");
      return;
    }

    size_t written = Update.write(characteristic->getData(), len);
    if (written != len)
    {
      Update.printError(Serial);
      Update.abort();
      otaInProgress = false;
      notifyOtaStatus("ERROR:WRITE_FAILED");
      return;
    }

    otaReceivedSize += written;
    if (otaReceivedSize - otaLastReportedSize >= 102400 || otaReceivedSize == otaExpectedSize)
    {
      otaLastReportedSize = otaReceivedSize;
      Serial.printf("[BLE OTA] Progress: %u / %u\n", otaReceivedSize, otaExpectedSize);

      if (otaReceivedSize == otaExpectedSize || otaReceivedSize % 204800 == 0)
      {
        char progress[40];
        snprintf(progress, sizeof(progress), "PROGRESS:%u/%u", otaReceivedSize, otaExpectedSize);
        notifyOtaStatus(progress);
      }
    }
  }
};

void setupBleOta()
{
  String bleName = getOtaBleName();
  BLEDevice::init(bleName.c_str());
  BLEDevice::setMTU(517);

  otaServer = BLEDevice::createServer();
  otaServer->setCallbacks(new MagiclingOtaServerCallbacks());

  BLEService *debugService = otaServer->createService(DEBUG_SERVICE_UUID);

  debugLogCharacteristic = debugService->createCharacteristic(
      DEBUG_LOG_TX_UUID,
      BLECharacteristic::PROPERTY_NOTIFY);
  debugLogCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *debugCommandCharacteristic = debugService->createCharacteristic(
      DEBUG_CMD_RX_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  debugCommandCharacteristic->setCallbacks(new MagiclingDebugCommandCallbacks());

  debugStatCharacteristic = debugService->createCharacteristic(
      DEBUG_STAT_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  debugStatCharacteristic->addDescriptor(new BLE2902());
  updateDebugStatus();

  debugService->start();

  BLEService *gameService = otaServer->createService(GAME_SERVICE_UUID);

  gameSensorCharacteristic = gameService->createCharacteristic(
      GAME_SENSOR_CHAR_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  gameSensorCharacteristic->addDescriptor(new BLE2902());
  uint8_t initialSensorFrame[2 + sizeof(int16_t) * 7 + 1] = {'S'};
  initialSensorFrame[16] = 0;
  gameSensorCharacteristic->setValue(initialSensorFrame, sizeof(initialSensorFrame));

  BLECharacteristic *gameHapticCharacteristic = gameService->createCharacteristic(
      GAME_HAPTIC_CHAR_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  gameHapticCharacteristic->setCallbacks(new GameHapticCallbacks());

  gameService->start();

  BLEService *otaService = otaServer->createService(OTA_SERVICE_UUID);

  BLECharacteristic *otaControlCharacteristic = otaService->createCharacteristic(
      OTA_CONTROL_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  otaControlCharacteristic->setCallbacks(new MagiclingOtaControlCallbacks());

  BLECharacteristic *otaDataCharacteristic = otaService->createCharacteristic(
      OTA_DATA_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  otaDataCharacteristic->setCallbacks(new MagiclingOtaDataCallbacks());

  otaStatusCharacteristic = otaService->createCharacteristic(
      OTA_STATUS_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  otaStatusCharacteristic->addDescriptor(new BLE2902());
  otaStatusCharacteristic->setValue("IDLE");

  otaService->start();

  otaModeStartedMs = millis();
  otaLastActivityMs = otaModeStartedMs;
  startBleAdvertising(BLE_PAIRING_OTA);

  Serial.println("BLE OTA pairing start");
}

void handleBleOta()
{
  if (otaAbortRequested)
  {
    otaAbortRequested = false;
    if (otaInProgress)
      Update.abort();
    otaInProgress = false;
    otaModeActive = false;
    notifyOtaStatus("ABORTED");
    updateDebugStatus();
  }

  if (!otaFinalizeRequested)
    return;

  otaFinalizeRequested = false;
  Serial.printf("[BLE OTA] Finalizing: %u bytes\n", otaReceivedSize);

  if (Update.end(true))
  {
    otaInProgress = false;
    otaModeActive = false;
    notifyOtaStatus("SUCCESS");
    Serial.println("[BLE OTA] Update success. Rebooting...");
    delay(1000);
    ESP.restart();
  }
  else
  {
    Update.printError(Serial);
    otaInProgress = false;
    otaModeActive = false;
    notifyOtaStatus("ERROR:END_FAILED");
    updateDebugStatus();
  }
}

void setup()
{
  Serial.begin(115200);
  SerialBT.begin("nIpxel_" + name);
  Serial.println("Bluetooth SPP start");
  setupBleOta();
  Wire.begin(23, 19);
  Wire.setTimeOut(100);

  pinMode(LED_PIN, OUTPUT);

  if (!bno.begin())
  {
    Serial.println("BNO055接続失敗");
    while (1)
      ; // 無限ループで停止
  }
  delay(100);
  bno.setExtCrystalUse(true);       // 外部水晶振動子使用
  bno.setMode(OPERATION_MODE_NDOF); // 9軸融合モード

  // PWM初期化
  PWM_INIT(motorPin, pwmFreq, pwmResolution, pwmChannel);
  pwmReady = true;
  digitalWrite(LED_PIN, LOW); // LED点灯
  prevMicros = micros();      // 時間計測初期化
}

// ==== メインループ ====
// 各処理関数を順番に呼び出して動作
void loop()
{
  handleBleOta();
  updateBlePairingMode();
  if (otaInProgress || otaFinalizeRequested || otaAbortRequested)
  {
    delay(10);
    return;
  }

  // 経過時間（秒）を計算（今回は未使用だが処理間隔確認に使える）
  float dt = (micros() - prevMicros) / 1e6;
  prevMicros = micros();

  readSensors(); // センサー読み込み＆LPF適用
  if (isnan(linAcc.x()) || isnan(quat.w()) || isnan(euler.x()))
  {
    Serial.println("読み取り失敗 → スキップ");
    delay(10);
    // 再取得
    readSensors();

    // まだダメなら加速度0として続行
    if (isnan(linAcc.x()) || isnan(quat.w()) || isnan(euler.x()))
    {
      Serial.println("再試行失敗 → デフォルト値で続行");
      ax_f = 0;
      ay_f = 0;
      az_f = 0;
    }
  }

  float ax_global, ay_global, az_global;
  updaterollandpitch();
  calcGlobalAcceleration(ax_global, ay_global, az_global); // グローバル加速度算出
  Vibration(ax_global, ay_global, az_global);
  // 自動ジャンプ補償は行わない。通常時は生Quaternionを正規化して送信する。
  // '5'で手動ゼロ設定された場合のみ、明示的なYawオフセットを適用する。
  quat_corrected = applyYawCorrection(quat, yaw_offset);
  // Bluetooth通信で情報の送信（ax,ay,az,qw,qx,qy,qz を固定小数で送る）
  outputDataAsBytes(ax_global, ay_global, az_global, quat_corrected);

  // ループカウンタ更新（4000を超えたら1に戻す）
  l++;
  if (l > 4000)
    l = 1;

  // デバッグ用に加速度・Yaw値をシリアル出力
  // Serial.printf("%.3f,%.3f \n", ax_f, euler.x());
  delay(10); // CPU負荷低減
}
