#include <Wire.h>             // I2C通信を行うための標準ライブラリ
#include <Adafruit_BNO055.h>  // BNO055 9軸センサ用ライブラリ
#include <utility/imumaths.h> // ベクトルやクォータニオン演算用のユーティリティ
#include <BLEDevice.h>        // BLE通信用
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

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

// ==== 関数プロトタイプ宣言 ====
void handleHapticCommand(uint8_t strength, uint8_t duration);

// ==== BLE設定 ====
// UUIDは16進数の文字列として定義（実装で確定）
#define SERVICE_UUID "12345678-1234-1234-1234-123456789abc"
#define SENSOR_CHAR_UUID "12345678-1234-1234-1234-123456789abd"
#define HAPTIC_CHAR_UUID "12345678-1234-1234-1234-123456789abe"

BLEServer *pServer = NULL;
BLECharacteristic *pSensorCharacteristic = NULL;
BLECharacteristic *pHapticCharacteristic = NULL;
bool deviceConnected = false;
uint8_t seqNumber = 0; // シーケンス番号（0-255で循環）

// ==== コールバッククラス内で使用するグローバル変数の前置き宣言 ====
// （クラス定義がグローバル変数宣言より前にあるため必要）
extern unsigned long lastNotifyTime;
extern bool haptic_active;
extern const int motorPin;
extern const int pwmChannel;

// BLE接続状態管理クラス
class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
    seqNumber = 0;             // 再接続時にシーケンス番号をリセット
    lastNotifyTime = micros(); // 接続直後のバースト送信を抑制
    Serial.println("BLE接続確立");
  }

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    // B3: 切断時の状態リセット（安全側へ）
    haptic_active = false;
    PWM_WRITE(motorPin, pwmChannel, 0);
    Serial.println("BLE切断");
    // 自動で再アドバタイジング開始
    BLEDevice::startAdvertising();
  }
};

// 触覚コマンド受信時のコールバック
class HapticCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string value = pCharacteristic->getValue();
    if (value.length() == 2)
    {
      uint8_t strength = (uint8_t)value[0];
      uint8_t duration = (uint8_t)value[1];
      handleHapticCommand(strength, duration);
    }
    else
    {
      // A3: 異常長の場合はログ出力
      Serial.printf("触覚コマンド異常: 長さ=%d (期待値:2)\n", value.length());
    }
  }
};

const int LED_PIN = 22;

// 1P,2Pの指定（BLE移行後は不要だが互換性のため残す）
String name = "1P";
// String name = "2P";

// PWM設定
const int motorPin = 25;     // 振動モータの制御ピン（GPIO25））
const int pwmFreq = 200;     // PWM周波数（Hz）※振動モータは低めでOK
const int pwmResolution = 8; // 分解能（8bit -> 0〜255）
const int pwmChannel = 0;

// 触覚フィードバック制御用変数
uint8_t haptic_strength = 0;
uint8_t haptic_duration = 0;
unsigned long haptic_start_time = 0;
bool haptic_active = false;

// ==== 加速度ベースの触覚フィードバック用変数 ====
bool accel_haptic_enabled = true;          // 加速度ベースの振動有効フラグ
const float ACCEL_HAPTIC_THRESHOLD = 0.1f; // 加速度ベースの振動開始閾値
const int ACCEL_HAPTIC_MAX = 250;          // 加速度ベースの振動最大値
const int ACCEL_HAPTIC_MIN = 10;           // 加速度ベースの振動最小値
const float ACCEL_HAPTIC_SCALE = 40.0f;    // 加速度スケーリング係数

// タイマ送信用変数
unsigned long lastNotifyTime = 0;
const unsigned long NOTIFY_INTERVAL_US = 16667; // 16.67ms = 60Hz

int l = 0; // メインループカウンタ

// ==== ローパスフィルタ用変数 ====
// alpha: 過去データの残す割合（0.0〜1.0）
float alpha = 0.7;
float ax_f = 0, ay_f = 0, az_f = 0; // ローパス適用後の加速度

// ==== Yaw（ヨー角）関連変数 ====
// yaw: 表示用Yaw角（補正適用後）
// yaw_prev: 前回のYaw角（生データ）
// yaw_raw: 生のYaw角（補正前）
// saved_yaw: ジャンプ前に安定していたYaw角
// yaw_offset: ジャンプ後の補正値
float yaw = 0, yaw_prev = 0, yaw_raw = 0.0, saved_yaw = 0.0;
float attack_yaw = 0, yaw_offset = 0;

// roll:ロール角
// pitch:ピッチ角
float roll = 0, pitch = 0, pitch_pre = 0;

// ==== ジャンプ（急激なYaw変化）検出用変数 ====
// jump: ジャンプ状態フラグ
// jump_prev: 前回のジャンプ状態
// j: ジャンプ状態の経過カウント
// j_wall: ジャンプ固定表示するループ回数
int jump = 0, jump_prev = 0, j = 0, j_wall = 2;

// ==== 時間計測用変数 ====
unsigned long prevMicros = 0; // 前回のループ開始時刻（µs）

// ==== センサー取得結果を保持する変数 ====
// linAcc: ローカル座標系の線形加速度（重力成分除去済み）
// euler: オイラー角（Yaw, Roll, Pitch）
// mag: 磁気ベクトル
// quat: クォータニオン姿勢
imu::Vector<3> linAcc, euler, mag;
imu::Quaternion quat;

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

// ==== ジャンプ検出と補正 ====
// 急激なYaw変化（ジャンプ）を検出し、一定時間固定表示後に補正適用
void updateJumpCompensation(float axy_pure)
{
  // センサーから生Yaw取得（±180°に正規化）
  yaw_raw = normalize180(euler.x());
  // 前回との差分を±180°で計算
  float diff = normalize180(yaw_raw - yaw_prev);
  jump_prev = jump;

  // ジャンプ検出条件: Yaw差分が大きく、かつジャンプ中でなく、XY加速度が小さいとき
  if (abs(diff) > 45 && abs(diff) < 320 && jump == 0 && abs(axy_pure) < 0.5)
  {
    jump = 1;
    j = 0;
    saved_yaw = yaw_prev; // ジャンプ開始
  }

  if (jump == 1)
  {
    j++;
    yaw = saved_yaw; // ジャンプ中は表示Yaw固定
    if (j >= j_wall)
    { // 待機ループ終了
      float new_diff = normalize180(yaw_raw - saved_yaw);
      yaw_offset += new_diff; // オフセット補正
      jump = 0;               // ジャンプ終了
    }
  }
  else
  {
    yaw = normalize180(yaw_raw - yaw_offset); // 通常はオフセット適用
  }

  // 次回比較用にYaw生値を保存
  yaw_prev = yaw_raw;
}

// ==== 触覚コマンド処理 ====
// BLE経由で受信した2バイトコマンドを処理
void handleHapticCommand(uint8_t strength, uint8_t duration)
{
  haptic_strength = strength;
  haptic_duration = duration;
  haptic_start_time = millis();
  haptic_active = (strength > 0 && duration > 0);

  Serial.printf("触覚コマンド: strength=%d, duration=%dms\n", strength, duration * 10);

  // 即座に振動開始
  if (haptic_active)
  {
    PWM_WRITE(motorPin, pwmChannel, strength);
  }
  else
  {
    PWM_WRITE(motorPin, pwmChannel, 0);
  }
}

// ==== 加速度ベースの触覚フィードバック処理 ====
// 加速度ベクトルの大きさに基づいて振動強度を計算
// 通信側からのコマンド実行中は加速度ベースの振動をスキップ（割り込み）
void updateAccelerationHaptic(float ax_global, float ay_global, float az_global)
{
  // 通信側からの触覚コマンド実行中は加速度ベースの振動をスキップ
  if (haptic_active)
  {
    return;
  }

  if (!accel_haptic_enabled)
  {
    PWM_WRITE(motorPin, pwmChannel, 0);
    return;
  }

  // グローバル座標系での加速度ベクトルの大きさを計算
  float a_magnitude = sqrt(ax_global * ax_global +
                           ay_global * ay_global +
                           az_global * az_global);

  // 閾値以下は振動なし
  if (a_magnitude < ACCEL_HAPTIC_THRESHOLD)
  {
    PWM_WRITE(motorPin, pwmChannel, 0);
    return;
  }

  // 加速度を振動強度に変換（スケーリング係数を適用）
  int vib_strength = (int)(a_magnitude * ACCEL_HAPTIC_SCALE);

  // 上限・下限クリッピング
  if (vib_strength > ACCEL_HAPTIC_MAX)
    vib_strength = ACCEL_HAPTIC_MAX;
  else if (vib_strength < ACCEL_HAPTIC_MIN)
    vib_strength = 0;

  PWM_WRITE(motorPin, pwmChannel, vib_strength);
}

// ==== 触覚フィードバック更新 ====
// duration経過後に自動停止し、加速度ベースの振動に戻す
void updateHapticFeedback()
{
  if (haptic_active)
  {
    unsigned long elapsed = millis() - haptic_start_time;
    unsigned long duration_ms = haptic_duration * 10; // 10ms単位→ms変換

    if (elapsed >= duration_ms)
    {
      // 触覚コマンド終了
      PWM_WRITE(motorPin, pwmChannel, 0);
      haptic_active = false;
      Serial.println("触覚コマンド完了");
    }
  }
}

// ===============================
// データ送信（BLE GATT Notify）
// ===============================
// 15バイト固定長フレームを送信
// [header(1), seq(1), ax(2), ay(2), az(2), pitch(2), yaw(2), roll(2), flags(1)]
void outputDataAsNotify(float ax_global, float ay_global, float az_global, float pitch_val, float yaw_val, float roll_val)
{
  if (!deviceConnected || pSensorCharacteristic == NULL)
  {
    return; // 未接続時は送信しない
  }

  // 固定小数点化
  int16_t ax_to_send = (int16_t)roundf(ax_global * 100.0f);
  int16_t ay_to_send = (int16_t)roundf(ay_global * 100.0f);
  int16_t az_to_send = (int16_t)roundf(az_global * 100.0f);
  int16_t pitch_to_send = (int16_t)roundf(pitch_val * 10.0f);
  int16_t yaw_to_send = (int16_t)roundf(yaw_val * 10.0f);
  int16_t roll_to_send = (int16_t)roundf(roll_val * 10.0f);

  // 15バイトバッファ構築
  uint8_t buffer[15];
  buffer[0] = 0x53; // header 'S'
  buffer[1] = seqNumber++;
  memcpy(&buffer[2], &ax_to_send, 2);
  memcpy(&buffer[4], &ay_to_send, 2);
  memcpy(&buffer[6], &az_to_send, 2);
  memcpy(&buffer[8], &pitch_to_send, 2);
  memcpy(&buffer[10], &yaw_to_send, 2);
  memcpy(&buffer[12], &roll_to_send, 2);
  // C1: flags（予約、MVPでは0）
  // 将来案: bit0=キャリブレーション完了, bit1=センサ読み失敗中, bit2=磁気校正品質低下
  buffer[14] = 0x00;

  // BLE Notify送信
  pSensorCharacteristic->setValue(buffer, 15);
  pSensorCharacteristic->notify();
}

// ==== セットアップ処理 ====
// ハードウェア初期化、BNO055設定、BLE設定
void setup()
{
  Serial.begin(115200);
  Serial.println("AR陰陽師グローブコントローラ起動");

  Wire.begin(23, 19);
  Wire.setTimeOut(100);

  pinMode(LED_PIN, OUTPUT);

  // BNO055初期化
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

  // BLE初期化
  String deviceName = "ARONMYOJI_GLOVE";
  BLEDevice::init(deviceName.c_str());

  // BLE Server作成
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // BLE Service作成
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Sensor Notify Characteristic作成（Notify専用）
  pSensorCharacteristic = pService->createCharacteristic(
      SENSOR_CHAR_UUID,
      BLECharacteristic::PROPERTY_NOTIFY);
  pSensorCharacteristic->addDescriptor(new BLE2902());

  // Haptics Write Characteristic作成（Write Without Response推奨）
  pHapticCharacteristic = pService->createCharacteristic(
      HAPTIC_CHAR_UUID,
      BLECharacteristic::PROPERTY_WRITE_NR | BLECharacteristic::PROPERTY_WRITE);
  pHapticCharacteristic->setCallbacks(new HapticCallbacks());

  // Service開始
  pService->start();

  // アドバタイジング設定
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  // B1: iPhone接続性向上のための既知回避策（ESP32 BLE標準パターン）
  // 動作確認済み：接続間隔min=6*1.25ms、max=18*1.25ms相当
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("BLEアドバタイジング開始: " + deviceName);
  digitalWrite(LED_PIN, LOW); // LED点灯
  prevMicros = micros();      // 時間計測初期化
  lastNotifyTime = micros();
}

// ==== メインループ ====
// センサー読み取りと60Hz Notify送信
void loop()
{
  // 経過時間（秒）を計算
  float dt = (micros() - prevMicros) / 1e6;
  prevMicros = micros();

  // センサー読み込み＆LPF適用
  readSensors();
  if (isnan(linAcc.x()) || isnan(quat.w()) || isnan(euler.x()))
  {
    Serial.println("読み取り失敗 → スキップ");
    delay(10);
    readSensors();

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
  calcGlobalAcceleration(ax_global, ay_global, az_global);
  updateJumpCompensation(sqrt(ax_global * ax_global + ay_global * ay_global));

  // 触覚フィードバック更新（duration経過チェック）
  updateHapticFeedback();

  // 加速度ベースの触覚フィードバック更新（通信側コマンド実行中は割り込まれる）
  updateAccelerationHaptic(ax_global, ay_global, az_global);

  // 60Hz送信（16.67ms周期）
  unsigned long currentTime = micros();
  if (currentTime - lastNotifyTime >= NOTIFY_INTERVAL_US)
  {
    // A2: タイマドリフト抑制（位相ずれの恒常化を防止）
    lastNotifyTime += NOTIFY_INTERVAL_US;
    // 大きく遅れた場合の追いつき処理（次回も即座に送信対象になる）
    if (currentTime - lastNotifyTime >= NOTIFY_INTERVAL_US)
    {
      lastNotifyTime = currentTime;
    }
    outputDataAsNotify(ax_global, ay_global, az_global, pitch, yaw, roll);
  }

  // ループカウンタ更新
  l++;
  if (l > 4000)
    l = 1;

  // デバッグ用シリアル出力（必要に応じて）
  // Serial.printf("%.3f,%.3f,%.3f \n", ax_f, euler.x(), yaw);

  // CPU負荷低減（タイマ駆動のため短縮可能）
  delay(1);
}
