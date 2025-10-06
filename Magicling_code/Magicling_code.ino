#include <Wire.h>             // I2C通信を行うための標準ライブラリ
#include <Adafruit_BNO055.h>  // BNO055 9軸センサ用ライブラリ
#include <utility/imumaths.h> // ベクトルやクォータニオン演算用のユーティリティ
#include <BluetoothSerial.h>
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
// String name = "2P";

// PWM設定
const int motorPin = 25;     // 振動モータの制御ピン（GPIO25））
const int pwmFreq = 200;     // PWM周波数（Hz）※振動モータは低めでOK
const int pwmResolution = 8; // 分解能（8bit -> 0〜255）
const int pwmChannel = 0;

int in = '0', in0 = '0'; // シリアル入力値（前回値と現在値） 初期値は'0'
int l = 0;               // メインループカウンタ

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
  pitch = normalize180(euler.z());
  roll = normalize180(euler.y());
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

// 加速度ベクトルの大きさに基づいて振動モータを制御する関数
void Vibration(float ax_global, float ay_global, float az_global)
{
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

  // '5' が来たらその時点のYawを0にするようにオフセットを更新
  if (in == '5')
  {
    // 最新センサー値からyaw_rawを再計算して確実に最新を使用
    yaw_raw = normalize180(euler.x());
    // 表示角 yaw = normalize180(yaw_raw - yaw_offset) が 0 になるように設定
    yaw_offset = yaw_raw;
    // ジャンプ状態をクリアし、直ちに反映
    jump = 0;
    j = 0;
    saved_yaw = yaw_raw;
    yaw = 0;
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
// Send six values over Bluetooth:
//  - Accel ax, ay, az as fixed-point (2 decimals): value * 100 -> int16
//  - Angles pitch, yaw, roll as fixed-point (1 decimal): value * 10 -> int16
void outputDataAsBytes(float ax_global, float ay_global, float az_global, float pitch_val, float yaw_val, float roll_val)
{
  int16_t ax_to_send = (int16_t)roundf(ax_global * 100.0f);
  int16_t ay_to_send = (int16_t)roundf(ay_global * 100.0f);
  int16_t az_to_send = (int16_t)roundf(az_global * 100.0f);
  int16_t pitch_to_send = (int16_t)roundf(pitch_val * 10.0f);
  int16_t yaw_to_send = (int16_t)roundf(yaw_val * 10.0f);
  int16_t roll_to_send = (int16_t)roundf(roll_val * 10.0f);

  // Buffer for six int16 values
  uint8_t buffer[sizeof(int16_t) * 6];
  memcpy(buffer + 0 * sizeof(int16_t), &ax_to_send, sizeof(int16_t));
  memcpy(buffer + 1 * sizeof(int16_t), &ay_to_send, sizeof(int16_t));
  memcpy(buffer + 2 * sizeof(int16_t), &az_to_send, sizeof(int16_t));
  memcpy(buffer + 3 * sizeof(int16_t), &pitch_to_send, sizeof(int16_t));
  memcpy(buffer + 4 * sizeof(int16_t), &yaw_to_send, sizeof(int16_t));
  memcpy(buffer + 5 * sizeof(int16_t), &roll_to_send, sizeof(int16_t));

  SerialBT.write('S');                    // header
  SerialBT.write(buffer, sizeof(buffer)); // payload
}

// ==== セットアップ処理 ====
// ハードウェア初期化、BNO055設定、M5Stack画面初期化
void setup()
{
  Serial.begin(115200);
  SerialBT.begin("LOLIN32_Lite_" + name);
  Serial.println("Bluetooth SPP start");
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
  digitalWrite(LED_PIN, LOW); // LED点灯
  prevMicros = micros();      // 時間計測初期化
}

// ==== メインループ ====
// 各処理関数を順番に呼び出して動作
void loop()
{
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
  calcGlobalAcceleration(ax_global, ay_global, az_global);                     // グローバル加速度算出
  updateJumpCompensation(sqrt(ax_global * ax_global + ay_global * ay_global)); // ジャンプ補正
  Vibration(ax_global, ay_global, az_global);
  // Bluetooth通信で情報の送信（ax,ay,az,pitch,yaw,roll を1桁固定小数で送る）
  outputDataAsBytes(ax_global, ay_global, az_global, pitch, yaw, roll);

  // ループカウンタ更新（4000を超えたら1に戻す）
  l++;
  if (l > 4000)
    l = 1;

  // デバッグ用に加速度・Yaw値をシリアル出力
  // Serial.printf("%.3f,%.3f,%.3f \n", ax_f, euler.x(), yaw);
  delay(10); // CPU負荷低減
}
