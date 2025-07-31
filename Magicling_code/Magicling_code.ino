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

float yaw_deg = 0;  // 現在のYaw角（-180〜180）
float yaw_g0  = 0;  // 前回のYaw角
float yaw     = 0;  // 積算Yaw角（連続値）
float attack_yaw = 0;

int attack_key = 0;
int prev_attack = 0;
unsigned long attack_key_start = 0;

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
}

void loop() {
  // ---- 重力補正済みローカル加速度 ----
  imu::Vector<3> linAcc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  // ---- クォータニオン ----
  imu::Quaternion quat = bno.getQuat();
  // ---- オイラー角 ----
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

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

  // ---- Yaw積算処理 ----
  yaw_g0 = yaw_deg;
  yaw_deg = euler.x();
  if (yaw_deg > 180.0) yaw_deg -= 360.0;
  float delta_yaw = yaw_deg - yaw_g0;
  if (delta_yaw > 300)  delta_yaw -= 360.0;
  if (delta_yaw < -300) delta_yaw += 360.0;
  yaw += delta_yaw;

  // ---- 判定用変数 ----
  float axy_pure = sqrt(ax_global * ax_global + ay_global * ay_global);

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

  // attack の状態変化を検出（1→0）
    if (prev_attack - attack == 1) {
        attack_key = 1;
        attack_key_start = millis();
    }

    // attack_key が 1 の場合、3秒後に 0 に戻す
    if (attack_key == 1 && millis() - attack_key_start >= 500) {
        attack_key = 0;
    }

    prev_attack = attack;

  // ---- 画面発光処理 ----
  if (attack == 1) {
    M5.Display.fillScreen(RED);
  } else if (protect == 1) {
    M5.Display.fillScreen(BLUE);
  } else if (attack_key == 1) {
    M5.Display.fillScreen(WHITE);
  } else {
    M5.Display.fillScreen(BLACK);
  }

  // ---- デバッグ出力 ----
  Serial.printf("%.3f,%.3f,%.3f\n", axy_pure, az_global, attack_key);
  delay(10);
}
