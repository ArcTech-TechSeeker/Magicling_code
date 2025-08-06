#include <Wire.h>                       // I2C通信を行うための標準ライブラリ
#include <Adafruit_BNO055.h>             // BNO055 9軸センサ用ライブラリ
#include <utility/imumaths.h>            // ベクトルやクォータニオン演算用のユーティリティ
#include <M5Unified.h>                   // M5Stack Core2 用の統合ライブラリ
#include <BluetoothSerial.h>
BluetoothSerial SerialBT; // Bluetooth SPP通信用


// ==== BNO055センサのインスタンス生成 ====
// 引数: センサID(任意), I2Cアドレス
Adafruit_BNO055 bno(55, 0x29);

// PWM設定
const int motorPin = 25;     // 振動モータの制御ピン（GPIO25）
const int pwmChannel = 0;    // PWMチャンネル番号（0〜15）
const int pwmFreq = 200;     // PWM周波数（Hz）※振動モータは低めでOK
const int pwmResolution = 8; // 分解能（8bit -> 0〜255）

int in = 0, in0 = 0;           // シリアル入力値（前回値と現在値）
int l = 0;                     // メインループカウンタ

// ==== ローパスフィルタ用変数 ====
// alpha: 過去データの残す割合（0.0〜1.0）
float alpha = 0.7;
float ax_f = 0, ay_f = 0, az_f = 0;      // ローパス適用後の加速度

// ==== 攻撃・防御・詠唱状態フラグ ====
int protect = 0, attack = 0, spell = 0;

// ==== 攻撃・防御の判定閾値 ====
float protect_wall = 1;                  // Z軸加速度で防御と判定する閾値
float attack_wall = 1.2;                  // XY平面の加速度で攻撃と判定する閾値

// ==== Yaw（ヨー角）関連変数 ====
// yaw: 表示用Yaw角（補正適用後）
// yaw_prev: 前回のYaw角（生データ）
// yaw_raw: 生のYaw角（補正前）
// saved_yaw: ジャンプ前に安定していたYaw角
// yaw_offset: ジャンプ後の補正値
float yaw = 0, yaw_prev = 0, yaw_raw = 0.0, saved_yaw = 0.0;
float attack_yaw = 0, yaw_offset = 0;

//roll:ロール角
//pitch:ピッチ角
float roll = 0, pitch = 0;

// ==== ジャンプ（急激なYaw変化）検出用変数 ====
// jump: ジャンプ状態フラグ
// jump_prev: 前回のジャンプ状態
// j: ジャンプ状態の経過カウント
// j_wall: ジャンプ固定表示するループ回数
int jump = 0, jump_prev = 0, j = 0, j_wall = 2;

// ==== 攻撃キー状態 ====
int attack_key = 0, prev_attack = 0;
unsigned long attack_key_start = 0;      // 攻撃キーが押された時刻（ms）

// ==== 時間計測用変数 ====
unsigned long prevMicros = 0;            // 前回のループ開始時刻（µs）
// ピッチが閾値を超えた時刻を記録する変数
unsigned long pitchStartTime = 0;

// ==== センサー取得結果を保持する変数 ====
// linAcc: ローカル座標系の線形加速度（重力成分除去済み）
// euler: オイラー角（Yaw, Roll, Pitch）
// mag: 磁気ベクトル
// quat: クォータニオン姿勢
imu::Vector<3> linAcc, euler, mag;
imu::Quaternion quat;

// ==== ユーティリティ関数 ====
// ±180°の範囲に角度を正規化する（何度でも繰り返して収める）
float normalize180(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle <= -180.0f) angle += 360.0f;
  return angle;
}


// ==== センサー値の取得とローパスフィルタ適用 ====
// BNO055から加速度・姿勢情報を取得し、加速度はローパス処理
void readSensors() {
  // センサーから各種データを取得
  linAcc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  quat   = bno.getQuat();
  euler  = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  mag    = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

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
void calcGlobalAcceleration(float &ax_global, float &ay_global, float &az_global) {
  // クォータニオンから回転行列を生成
  float qw = quat.w(), qx = quat.x(), qy = quat.y(), qz = quat.z();
  float R[3][3] = {
    {1 - 2 * (qy*qy + qz*qz), 2 * (qx*qy - qz*qw), 2 * (qx*qz + qy*qw)},
    {2 * (qx*qy + qz*qw), 1 - 2 * (qx*qx + qz*qz), 2 * (qy*qz - qx*qw)},
    {2 * (qx*qz - qy*qw), 2 * (qy*qz + qx*qw), 1 - 2 * (qx*qx + qy*qy)}
  };
  // ローカル加速度をグローバル座標に変換
  ax_global = R[0][0] * ax_f + R[0][1] * ay_f + R[0][2] * az_f;
  ay_global = R[1][0] * ax_f + R[1][1] * ay_f + R[1][2] * az_f;
  az_global = R[2][0] * ax_f + R[2][1] * ay_f + R[2][2] * az_f;
}

// ==== 攻撃/防御の判定 ====
// グローバル加速度から攻撃・防御フラグを更新
void detectAttackProtect(float ax_global, float ay_global, float az_global) {
  float axy_pure = sqrt(ax_global * ax_global + ay_global * ay_global);
  if (az_global > protect_wall || axy_pure > attack_wall) {
    if (az_global > axy_pure) {
      protect = 1; if (protect + attack == 2) protect = 0;
    } else {
      attack = 1; if (protect + attack == 2) attack = 0;
    }
  } else if (fabs(az_global) < protect_wall - 0.3 && fabs(axy_pure) < attack_wall - 0.3) {
    attack = 0; protect = 0;
  }
}

// ==== 攻撃キー更新 ====
// 攻撃状態が 1→0 に変化した瞬間に attack_key を 1 にセットし、
// 500ms 経過後に自動で 0 に戻す
void updateAttackKey() {
  // 前回攻撃中だったが今回は攻撃解除 → イベント検出
  if (prev_attack - attack == 1) {
    attack_key = 1;
    attack_yaw = yaw;               // 攻撃終了時のYaw角を記録
    attack_key_start = millis();    // 記録時刻
  }
  // attack_keyが立っている状態で500ms経過したら解除
  if (attack_key == 1 && millis() - attack_key_start >= 500) {
    attack_key = 0;
  }
  // 次回比較用に攻撃状態を保存
  prev_attack = attack;
}

void updaterollandpitch(){
  pitch = normalize180(euler.y());
  roll = normalize180(euler.z());
}


// ==== ジャンプ検出と補正 ====
// 急激なYaw変化（ジャンプ）を検出し、一定時間固定表示後に補正適用
void updateJumpCompensation(float axy_pure) {
  // センサーから生Yaw取得（±180°に正規化）
  yaw_raw = normalize180(euler.x());
  // 前回との差分を±180°で計算
  float diff = normalize180(yaw_raw - yaw_prev);
  jump_prev = jump;

  // ジャンプ検出条件: Yaw差分が大きく、かつジャンプ中でなく、XY加速度が小さいとき
  if (abs(diff) > 45 && abs(diff) < 320 && jump == 0 && abs(axy_pure) < 0.5) {
    jump = 1; j = 0; saved_yaw = yaw_prev; // ジャンプ開始
  }

  if (jump == 1) {
    j++;
    yaw = saved_yaw; // ジャンプ中は表示Yaw固定
    if (j >= j_wall) { // 待機ループ終了
      float new_diff = normalize180(yaw_raw - saved_yaw);
      yaw_offset += new_diff; // オフセット補正
      jump = 0; // ジャンプ終了
    }
  } else {
    yaw = normalize180(yaw_raw - yaw_offset); // 通常はオフセット適用
  }

  // 次回比較用にYaw生値を保存
  yaw_prev = yaw_raw;
}

void updatespell() {
  // pitchが80度以上ならタイマー開始
  if (abs(pitch) >= 70) {
    // まだタイマーをスタートしていなければ開始時刻を記録
    if (pitchStartTime == 0) {
      pitchStartTime = millis();
    }
    // 0.1秒（100ms）以上経過していたらspell=1
    if (millis() - pitchStartTime >= 100) {
      spell = 1;
    }
  } else {
    // 条件を満たさなければリセット
    pitchStartTime = 0;
    spell = 0;
  }
}

// ==== 画面表示更新 ====
// 状態に応じた背景色・文字色設定と表示
void updateDisplay() {
  if (attack == 1) {
    M5.Display.setTextColor(WHITE, RED);
    M5.Display.fillScreen(RED);
  } else if (protect == 1) {
    M5.Display.fillScreen(BLUE);
  } else if (attack_key == 1) {
    M5.Display.fillScreen(BLACK);
    M5.Display.setTextSize(7);
    M5.Display.setTextColor(RED, BLACK);
    M5.Display.setCursor(0, M5.Display.height()/2 - 20);
    M5.Display.printf("%d", (int)attack_yaw);
  }else if (spell == 1){
    M5.Display.fillScreen(YELLOW);
  } else {
    M5.Display.fillScreen(BLACK);
  }

  if (jump == 1) {
    M5.Display.fillScreen(GREEN);
    M5.Display.setTextSize(7);
    M5.Display.setTextColor(WHITE, GREEN);
    M5.Display.setCursor(0, M5.Display.height()/2 - 20);
    M5.Display.printf("%d", (int)yaw);
  }
}

// 加速度ベクトルの大きさに基づいて振動モータを制御する関数
void Vibration(float ax_global, float ay_global, float az_global) {
  in0 = in;
  if (SerialBT.available() > 0) {
    in = SerialBT.read();
    if (in < '0' || in > '5') in = in0;
  } else {
    in = in0;
  }

  if(in == 0){
    // グローバル座標系での加速度ベクトルの大きさ（重力込み）を計算
    // √(ax^2 + ay^2 + az^2) → 全方向の加速度の合成値
    float a_pure = sqrt(ax_global * ax_global +
                        ay_global * ay_global +
                        az_global * az_global);

    // 加速度の値を 3 乗して感度を調整し、スケーリング係数300を掛けて PWM 値に変換
    // 3乗することで小さい加速度変化に対して感度を下げ、大きい加速度で急に強くなるカーブになる
    int vib = a_pure * 40;

    // 上限値を 250 に制限（PWM 8bit の最大255に近い値）
    if (vib > 250) vib = 250;
    // 下限閾値60未満はモータ停止（物理的に動かない領域をカット）
    else if (vib < 10) vib = 0;

    // PWM出力で振動モータを駆動
    ledcWrite(pwmChannel, vib);
  }else {
    int flashRate = 0;
    float flashRate_deno = 0; 
    if (in == '1') {flashRate = 20; flashRate_deno = 0.4;} 
    else if (in == '2') {flashRate = 40; flashRate_deno = 0.2;}
    else if (in == '3') {flashRate = 80; flashRate_deno = 0.125;}
    else if (in == '4') {flashRate = 100; flashRate_deno = 0.9;}
    bool motorOn = (l % flashRate < flashRate * flashRate_deno);
    ledcWrite(pwmChannel, motorOn ? 255 : 0);
  }
}

// ===============================
// データ送信（シリアル通信）
// ===============================
void outputDataAsBytes() {
  // 送る値を int16_t に変換（整数化）
  int16_t protect_to_send    = (int16_t)protect;                // 1 or 0
  int16_t spell_to_send      = (int16_t)spell;                  // 1 or 0
  int16_t spell2_to_send      = (int16_t)attack;                  // 1 or 0
  int16_t attack_key_to_send = (int16_t)attack_key;              // 1 or 0
  int16_t attack_yaw_to_send = (int16_t)(attack_yaw * 10.0f);    // 0.1刻み

  // バッファ作成
  uint8_t buffer[sizeof(int16_t) * 5];
  memcpy(buffer,                           &protect_to_send,    sizeof(int16_t));
  memcpy(buffer + 1 * sizeof(int16_t),     &spell_to_send,      sizeof(int16_t));
  memcpy(buffer + 2 * sizeof(int16_t),     &spell2_to_send,      sizeof(int16_t));
  memcpy(buffer + 3 * sizeof(int16_t),     &attack_key_to_send, sizeof(int16_t));
  memcpy(buffer + 4 * sizeof(int16_t),     &attack_yaw_to_send, sizeof(int16_t));

  // 送信
  SerialBT.write('S');                     // ヘッダ
  SerialBT.write(buffer, sizeof(buffer));  // 本文
}


// ==== セットアップ処理 ====
// ハードウェア初期化、BNO055設定、M5Stack画面初期化
void setup() {
  Serial.begin(115200);
  SerialBT.begin("M5Core2_1P");   // Bluetoothデバイス名
  Serial.println("Bluetooth SPP start");
  Wire.begin(32, 33);
  Wire.setTimeOut(100);

  if (!bno.begin()) {
    Serial.println("BNO055接続失敗");
    while (1); // 無限ループで停止
  }
  delay(100);
  bno.setExtCrystalUse(true);            // 外部水晶振動子使用
  bno.setMode(OPERATION_MODE_NDOF);      // 9軸融合モード
  

  // M5Unifiedの設定オブジェクトを取得（デフォルト設定を読み込む）
  auto cfg = M5.config();
  // M5Stack Core2 の初期化（cfg の設定値に基づいて LCD やタッチパネル、I2C 等を初期化）
  M5.begin(cfg);
  // LCDのバックライト輝度を設定（0〜255、ここでは200）
  M5.Display.setBrightness(200);
  // LCD画面を黒色で塗りつぶす（TFT_BLACK は黒を表す定数）
  M5.Display.fillScreen(TFT_BLACK);


  // PWM初期化
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(motorPin, pwmChannel);

  prevMicros = micros(); // 時間計測初期化
}

// ==== メインループ ====
// 各処理関数を順番に呼び出して動作
void loop() {
  // 経過時間（秒）を計算（今回は未使用だが処理間隔確認に使える）
  float dt = (micros() - prevMicros) / 1e6;
  prevMicros = micros();

  readSensors(); // センサー読み込み＆LPF適用
  if (isnan(linAcc.x()) || isnan(quat.w()) || isnan(euler.x())) {
    Serial.println("読み取り失敗 → スキップ");
    delay(10);
    // 再取得
    readSensors();

    // まだダメなら加速度0として続行
    if (isnan(linAcc.x()) || isnan(quat.w()) || isnan(euler.x())) {
        Serial.println("再試行失敗 → デフォルト値で続行");
        ax_f = 0; ay_f = 0; az_f = 0;
    }
  }

  float ax_global, ay_global, az_global;
  updaterollandpitch();
  calcGlobalAcceleration(ax_global, ay_global, az_global); // グローバル加速度算出
  detectAttackProtect(ax_global, ay_global, az_global);     // 攻撃・防御判定
  updateAttackKey();      
  updatespell();                                  // 攻撃キー更新
  updateJumpCompensation(sqrt(ax_global*ax_global + ay_global*ay_global)); // ジャンプ補正
  Vibration(ax_global, ay_global, az_global);
  updateDisplay(); // 状態に応じた画面更新
  // Bluetooth通信で情報の送信
  outputDataAsBytes();

  // ループカウンタ更新（4000を超えたら1に戻す）
  l++;
  if (l > 4000) l = 1;

  // デバッグ用に加速度・Yaw値をシリアル出力
  // Serial.printf("%.3f,%.3f,%.3f \n", ax_f, euler.x(), yaw);
  delay(1); // CPU負荷低減
}
