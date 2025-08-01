#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <M5Unified.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

float alpha = 0.7;
float ax_f = 0, ay_f = 0, az_f = 0;

int protect = 0;
int attack = 0;

float protect_wall = 1;
float attack_wall = 1;

float yaw     = 0;  // 表示用Yaw角
float yaw_prev = 0;
float yaw_raw = 0.0;   // センサー生Yaw
float saved_yaw = 0.0; // ジャンプ前のYaw
float attack_yaw = 0;
float yaw_offset = 0;

int jump = 0;
int jump_prev = 0;
int j = 0;
int j_wall = 2;

int attack_key = 0;
int prev_attack = 0;
unsigned long attack_key_start = 0;

// ジャイロ積分用クォータニオン（単位姿勢）
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

unsigned long lastUpdate = 0;

unsigned long prevMicros = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(32, 33);
  Wire.setTimeOut(100);

  if (!bno.begin()) {
    Serial.println("BNO055接続失敗");
    while (1);
  }

  delay(100);
  bno.setExtCrystalUse(true);
  bno.setMode(OPERATION_MODE_NDOF);

  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setBrightness(200);
  M5.Display.fillScreen(TFT_BLACK);

  // 初期値として期待する更新レートをセット（後で実測に基づき補正）
  lastUpdate = micros();
  prevMicros = micros();
}

void loop() {
  // ---- 時間差計算 ----
  unsigned long nowMicros = micros();
  float dt = (nowMicros - prevMicros) / 1e6; // 秒
  prevMicros = nowMicros;

  // ---- 重力補正済みローカル加速度 ----
  imu::Vector<3> linAcc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  // ---- クォータニオン ----
  imu::Quaternion quat = bno.getQuat();
  // ---- オイラー角 ----
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  // --- 地磁気ベクトル取得 ---
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  float mx = mag.x();
  float my = mag.y();
  float mz = mag.z();

  if (isnan(linAcc.x()) || isnan(quat.w()) || isnan(euler.x())) {
    Serial.println("読み取り失敗 → スキップ");
    delay(10);
    return;
  }

  // ---- ローパスフィルタ（ローカル加速度）----
  float ax = linAcc.x() / 9.8;
  float ay = linAcc.y() / 9.8;
  float az = linAcc.z() / 9.8;
  ax_f = alpha * ax_f + (1 - alpha) * ax;
  ay_f = alpha * ay_f + (1 - alpha) * ay;
  az_f = alpha * az_f + (1 - alpha) * az;

  // ---- クォータニオン → 回転行列 ----
  float qw = quat.w();
  float qx = quat.x();
  float qy = quat.y();
  float qz = quat.z();

  float R[3][3];
  R[0][0] = 1 - 2 * (qy*qy + qz*qz);
  R[0][1] = 2 * (qx*qy - qz*qw);
  R[0][2] = 2 * (qx*qz + qy*qw);

  R[1][0] = 2 * (qx*qy + qz*qw);
  R[1][1] = 1 - 2 * (qx*qx + qz*qz);
  R[1][2] = 2 * (qy*qz - qx*qw);

  R[2][0] = 2 * (qx*qz - qy*qw);
  R[2][1] = 2 * (qy*qz + qx*qw);
  R[2][2] = 1 - 2 * (qx*qx + qy*qy);

  // ---- ローカル → グローバル座標変換 ----
  float ax_global = R[0][0] * ax_f + R[0][1] * ay_f + R[0][2] * az_f;
  float ay_global = R[1][0] * ax_f + R[1][1] * ay_f + R[1][2] * az_f;
  float az_global = R[2][0] * ax_f + R[2][1] * ay_f + R[2][2] * az_f;

  // ---- 判定用変数 ----
  float axy_pure = sqrt(ax_global * ax_global + ay_global * ay_global);
  float a_pure = sqrt(ax_global * ax_global + ay_global * ay_global + az_global*az_global);

  // ---- 攻撃/防御判定 ----
  if (az_global > protect_wall || axy_pure > attack_wall) {
    if (az_global > axy_pure) {
      protect = 1;
      if (protect + attack == 2) protect = 0;
    } else {
      attack = 1;
      if (protect + attack == 2) attack = 0;
    }
  } else if (fabs(az_global) < protect_wall - 0.3 && fabs(axy_pure) < attack_wall - 0.3) {
    attack = 0;
    protect = 0;
  }

  //---- attack の状態変化を検出（1→0）----
  if (prev_attack - attack == 1) {
    attack_key = 1;
    attack_yaw = yaw;
    attack_key_start = millis();
  }

  // attack_key が 1 の場合、0.5秒後に0に戻す
  if (attack_key == 1 && millis() - attack_key_start >= 500) {
    attack_key = 0;
  }

  prev_attack = attack;

  float roll = euler.y() * PI / 180.0;
  float pitch = euler.z() * PI / 180.0; 

  // --- 生Yaw更新 ---
  yaw_raw = euler.x();
  if (yaw_raw > 180) yaw_raw -= 360;
  if (yaw_raw < -180) yaw_raw += 360;

  // diff計算（生Yaw同士で）
  float diff = yaw_raw - yaw_prev;
  if (diff > 180)  diff -= 360;
  if (diff < -180) diff += 360;

  jump_prev = jump;

  // --- ジャンプ検出 ---
  if (abs(diff) > 45 && abs(diff) < 320 && jump == 0 && abs(axy_pure) < 0.5) {
    jump = 1;
    j = 0;
    saved_yaw = yaw_prev; // 安定値保存
  }

  // --- ジャンプ中処理 ---
  if (jump == 1) {
    j++;
    yaw = saved_yaw; // 表示は固定
    if (j >= j_wall) {
      // 安定後の差分は最新の生Yawと比較
      float new_diff = yaw_raw - saved_yaw;
      if (new_diff > 180)  new_diff -= 360;
      if (new_diff < -180) new_diff += 360;

      yaw_offset += new_diff;
      jump = 0;
    }
  } else {
    yaw = yaw_raw - yaw_offset; // 通常時はオフセット適用
  }

  yaw_prev = yaw_raw; // 検出用は常に更新


  // 再度 ±180°に正規化
  if (yaw > 180) {
    int n = abs(yaw / 180);
    yaw -= 180 + 180 * n;
  }else if (yaw < -180){
    int n = abs(yaw / 180);
    yaw += 180 + 180 * n;
  } 

  

  //---- 画面表示処理 ----
  if (attack == 1){
    M5.Display.setTextColor(WHITE, RED);
    M5.Display.fillScreen(RED);
  } else if (protect == 1) {
    M5.Display.fillScreen(BLUE);
  } else if (attack_key == 1) {
    M5.Display.fillScreen(BLACK);
    M5.Display.setTextSize(7);
    M5.Display.setTextColor(WHITE, BLACK);
  } else {
    M5.Display.fillScreen(BLACK);
    M5.Display.setTextSize(7);
    M5.Display.setTextColor(WHITE, BLACK);
  }
  if (jump == 1){
    M5.Display.fillScreen(GREEN);
    M5.Display.setTextSize(7);
    M5.Display.setTextColor(WHITE, GREEN);
  }
  M5.Display.setCursor(0, M5.Display.height()/2 - 20);
  M5.Display.printf("%d", (int)yaw);

  // ---- デバッグ出力 ----
  Serial.printf("%.3f,%.3f,%.3f \n", ax_f, euler.x(), yaw);
  delay(1);
}
