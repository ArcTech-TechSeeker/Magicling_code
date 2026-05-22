# Magicling_code（nIpxel用コード）

モーション入力＆ハプティクス付き **無線コントローラ nIpxel**（ESP32 + BNO055）を中心としたコード/回路/CADである。
9 軸センサから得た姿勢・加速度を**世界座標へ変換**し、**Bluetooth SPP** でホストへ送出すると同時に、振動モータを **PWM** で制御する。

紹介サイト：https://nipxel.netlify.app/

---

## 特長

* **9軸姿勢融合**：BNO055（NDOF）から **Yaw / Pitch / Roll** と **線形加速度**を取得
* **座標安定化**：クォータニオン→回転行列で**グローバル座標系加速度**を算出
* **ジャンプ補償**：Yaw のスパイク（磁気外乱等）を短時間固定＋オフセットで自然連続化
* **ハプティクス**：振動モータを **200 Hz / 8bit PWM** で駆動（アナログ追従／点滅パターン）
* **軽量通信**：Bluetooth SPP で **固定小数バイナリ**（13 B/フレーム）を周期送信
* **2P対応**：デバイス名末尾を `"1P"` / `"2P"` で切替

---

## アーキテクチャ

```
BNO055 → (I²C) → ESP32 (LoLin32 Lite 等)
         ├─ クォータニオン姿勢 → 回転行列 → 世界座標加速度
         ├─ ジャンプ補償（Yaw offset）
         ├─ ハプティクス制御（PWM）
         └─ Bluetooth SPP（固定小数バイナリ）→ PC / ゲーム
```

---

## ハードウェア

* **MCU**：ESP32（LoLin32 Lite 等）
* **IMU**：Adafruit BNO055（I²C, アドレス `0x29`）
* **モータ**：小型振動モータ（ローサイド NPN 駆動：2SC1815）
* **ベース抵抗**：**1 kΩ（茶・黒・赤・金）**
* **フライバック**：1N400x 等（モータ端子**並列・逆向き**）

### 回路データ
回路データはCircuitフォルダ内のもの．Kicadを用いて作成されている．

PCBを発注したければ，GaberのZIPファイルを直接JLCPCBなどのサービスに発注するとよい

### CADデータ
AutodeskのInventorを用いて製作．プリントしたい場合はfor 3Dprintフォルダ内のSTLを使用

### 既定ピン

| 機能       | ESP32 ピン    | 備考                  |
| -------- | ----------- | ------------------- |
| I²C SDA  | GPIO **23** | `Wire.begin(23,19)` |
| I²C SCL  | GPIO **19** | 〃                   |
| PWM（モータ） | GPIO **25** | 200 Hz / 8bit       |
| LED      | GPIO **22** | 動作表示                |

> 3.3 V／GND 共有。I²C は 4.7–10 kΩ のプルアップ（多くのブレークアウトに実装済み）。

---

## ファームウェア概要

### 主要ライブラリ

`Wire.h`（I²C）、`Adafruit_BNO055.h`（9軸融合）、`utility/imumaths.h`（Quat/行列）、`BluetoothSerial.h`（SPP）

### PWM 互換層

ESP32 Arduino Core v2/v3 の差異をマクロで吸収（`ledcSetup/AttachPin` vs `ledcAttach`）。

### センサ処理パイプライン

1. **取得**：BNO055 から **線形加速度**・**オイラー角**・**クォータニオン**
2. **平滑化**：加速度[g]に一次 IIR（`alpha=0.7`）
3. **座標変換**：Quat→3×3 回転行列でローカル→**世界座標系**へ
4. **Yaw 補償**：差分 > 45°（かつ < 320°）を「ジャンプ」とみなし短時間固定、後段で `yaw_offset` を適用

### ハプティクス（`Vibration()`）

* **モード選択**：SPP で `'0'..'5'` の一文字コマンド

  * `'0'`：アナログ追従（|a| からデューティ算出、下限デッドゾーン・上限クランプ）
  * `'1'..'4'`：点滅パターン（周期×デューティ）
  * `'5'`：停止

---

## Bluetooth SPP プロトコル

**1 フレーム = 13 バイト**

* 先頭ヘッダ：`'S'`（1 B）
* ペイロード：**int16 × 6**（12 B, Little-Endian）

  1. `ax_global × 100`（2桁固定小数）
  2. `ay_global × 100`
  3. `az_global × 100`
  4. `pitch × 10`（1桁固定小数）
  5. `yaw × 10`
  6. `roll × 10`

**送信周期**：およそ **100 Hz**（`delay(10)` 目安）
**デコード（受信側）**：LE の `int16` を実数に復元（加速度÷100、角度÷10）。ヘッダ `'S'` で同期。

---

## ビルド & 書き込み

### Arduino IDE

1. ボード：ESP32 ファミリ（LoLin32 Lite 等）
2. 依存ライブラリ導入（Adafruit BNO055 ほか）
3. スケッチを書き込み

### PlatformIO / ESP-IDF

* `ledc` / `Wire` / `BluetoothSerial` が利用可能な環境設定でビルド。

---

## 起動と使用

1. 電源投入／SPP ペアリング：デバイス名は `"LOLIN32_Lite_1P"`（`name` で `"2P"` に切替可）
2. ホストは SPP で **13B 固定フレーム**を連続受信
3. 必要に応じ `'0'..'5'` を送信して**ハプティクスモード**を切替

---

## 調整可能パラメータ

| 変数              |     既定 | 説明         |
| --------------- | -----: | ---------- |
| `alpha`         |    0.7 | ローパス（過去重み） |
| `pwmFreq`       | 200 Hz | モータ駆動周波数   |
| `pwmResolution` |  8 bit | デューティ分解能   |
| `j_wall`        |  2 ループ | Yaw 固定期間   |
| `name`          | `"1P"` | SPP 名末尾    |

---

## リポジトリ構成（例）

```
Magicling_code/
├── src/                # ESP32 スケッチ/ソース
├── include/            # ヘッダ
├── hardware/           # 回路図・配線図
└── README.md
```

> 実際の構成はリポジトリに従うこと。名称や配置は変更される場合がある。


## ライセンス

リポジトリ同梱の **LICENSE** を参照。
# Magicling_code (Code for nIpxel)

This repository contains the code, circuit data, and CAD resources centered around **nIpxel**, a **wireless controller with motion input and haptics** (ESP32 + BNO055).

It converts orientation/acceleration obtained from a 9-axis sensor into the **world (global) coordinate frame**, transmits the data to a host via **Bluetooth SPP**, and controls a vibration motor using **PWM**.

Project site: https://nipxel.netlify.app/

---

## Features

- **9-axis sensor fusion**: Uses BNO055 (NDOF) to obtain **Yaw / Pitch / Roll** and **linear acceleration**
- **Coordinate stabilization**: Converts quaternion → rotation matrix to compute **global-frame acceleration**
- **Jump compensation**: Detects yaw spikes (e.g., magnetic disturbance), briefly freezes + applies an offset for smooth continuity
- **Haptics**: Drives a vibration motor at **200 Hz / 8-bit PWM** (analog follow / blink patterns)
- **Lightweight transport**: Periodically sends **fixed-point binary** frames over Bluetooth SPP (**13 B/frame**)
- **Two-player support**: Switch the device name suffix to `"1P"` / `"2P"`

---

## Architecture

```text
BNO055 → (I²C) → ESP32 (LoLin32 Lite, etc.)
         ├─ Quaternion orientation → Rotation matrix → World-frame acceleration
         ├─ Jump compensation (yaw offset)
         ├─ Haptics control (PWM)
         └─ Bluetooth SPP (fixed-point binary) → PC / game
```

---

## Hardware

- **MCU**: ESP32 (LoLin32 Lite, etc.)
- **IMU**: Adafruit BNO055 (I²C, address `0x29`)
- **Motor**: Small vibration motor (low-side NPN drive: 2SC1815)
- **Base resistor**: **1 kΩ (brown-black-red-gold)**
- **Flyback diode**: 1N400x, etc. (in parallel across motor terminals, reverse polarity)

### Circuit Data

Circuit data is located in the `Circuit` folder and was created using **KiCad**.

If you want to order a PCB, you can upload the **Gerber ZIP** directly to services such as **JLCPCB**.

### CAD Data

Designed using **Autodesk Inventor**. For 3D printing, use the STL files in the `for 3Dprint` folder.

### Default Pins

| Function        | ESP32 Pin     | Notes               |
|----------------|---------------|---------------------|
| I²C SDA        | GPIO **23**   | `Wire.begin(23,19)` |
| I²C SCL        | GPIO **19**   | same as above       |
| PWM (motor)    | GPIO **25**   | 200 Hz / 8-bit      |
| LED            | GPIO **22**   | status indicator    |

> Share 3.3 V / GND. I²C requires 4.7–10 kΩ pull-ups (often included on breakout boards).

---

## Firmware Overview

### Key Libraries

`Wire.h` (I²C), `Adafruit_BNO055.h` (sensor fusion), `utility/imumaths.h` (Quat/matrix), `BluetoothSerial.h` (SPP)

### PWM Compatibility Layer

Absorbs differences between ESP32 Arduino Core v2/v3 using macros (e.g., `ledcSetup/AttachPin` vs `ledcAttach`).

### Sensor Processing Pipeline

1. **Read**: From BNO055: **linear acceleration**, **Euler angles**, **quaternion**
2. **Smoothing**: First-order IIR on acceleration [g] (`alpha=0.7`)
3. **Transform**: Quat → 3×3 rotation matrix; local → **world frame**
4. **Yaw compensation**: If delta > 45° (and < 320°), treat as a “jump”, freeze briefly; apply `yaw_offset` afterward

### Haptics (`Vibration()`)

- **Mode selection**: Send a single-character command via SPP: `'0'..'5'`

  - `'0'`: Analog follow (compute duty from |a| with deadzone and clamp)
  - `'1'..'4'`: Blink patterns (period × duty)
  - `'5'`: Stop

---

## Bluetooth SPP Protocol

**1 frame = 13 bytes**

- Header: `'S'` (1 B)
- Payload: **int16 × 6** (12 B, Little-Endian)

  1. `ax_global × 100` (fixed-point with 2 decimals)
  2. `ay_global × 100`
  3. `az_global × 100`
  4. `pitch × 10` (fixed-point with 1 decimal)
  5. `yaw × 10`
  6. `roll × 10`

**Transmit rate**: approx. **100 Hz** (target via `delay(10)`)  
**Decode (receiver)**: Convert LE `int16` back to float (accel ÷ 100, angles ÷ 10). Use header `'S'` for synchronization.

---

## Build & Flash

### Arduino IDE

1. Board: ESP32 family (LoLin32 Lite, etc.)
2. Install dependencies (Adafruit BNO055, etc.)
3. Upload the sketch

### PlatformIO / ESP-IDF

- Build with an environment where `ledc`, `Wire`, and `BluetoothSerial` are available.

---

## Startup & Usage

1. Power on / pair via SPP: device name defaults to `"LOLIN32_Lite_1P"` (set `name` to switch to `"2P"`)
2. Host continuously receives **13-byte fixed frames** over SPP
3. Send `'0'..'5'` as needed to switch **haptics modes**

---

## Tunable Parameters

| Variable          | Default  | Description            |
|------------------|---------:|------------------------|
| `alpha`          |    0.7   | Low-pass filter weight |
| `pwmFreq`        | 200 Hz   | Motor drive frequency  |
| `pwmResolution`  |  8 bit   | PWM duty resolution    |
| `j_wall`         | 2 loops  | Yaw freeze duration    |
| `name`           | `"1P"`   | SPP name suffix        |

---

## Repository Layout (Example)

```text
Magicling_code/
├── src/                # ESP32 sketch/source
├── include/            # headers
├── hardware/           # schematics/wiring
└── README.md
```

> Follow the actual repository structure. Names and locations may change.

---

## License

See the **LICENSE** file included in this repository.

---

## BLE OTA

The firmware exposes the same BLE OTA protocol as
`naka6ryo/RemoteCompilerToMicon` WebAppSide, without adding Wi-Fi
provisioning.

* BLE device name during the first 10 seconds after boot:
  `ESP32-nIpxel_1P` or `ESP32-nIpxel_2P`
* OTA service: `9f5f0001-8d9e-6f4e-bd0c-3c4d5e6f7180`
* Control: `START:<size>`, `END`, `ABORT`
* Data: binary firmware chunks
* Status: `READY`, `PROGRESS:x/y`, `SUCCESS`, `ERROR:*`

Use an ESP32 partition scheme with OTA slots. The VS Code/Arduino CLI settings
use `PartitionScheme=min_spiffs`, which gives the OTA app slot 1,966,080 bytes.
The PlatformIO settings use `partitions_ota_2m.csv`, matching the referenced
BLE OTA project.

## Game BLE Service

The device also exposes the game-facing BLE service used by
`BleControllerAdapter.js`.

* BLE device name after the OTA pairing window: `nIpxel_1P` or `nIpxel_2P`
* Service: `12345678-1234-1234-1234-123456789abc`
* Sensor Notify: `12345678-1234-1234-1234-123456789abd`
* Haptic Write: `12345678-1234-1234-1234-123456789abe`
* Sensor frame: 15 bytes, `S` + sequence byte + six little-endian `int16` values + flags byte
* Haptic command: 2 bytes, `strength` and `duration` in 10 ms units

Arduino CLI build outputs are written to the workspace-local `build/` folder.

BLE advertising is mode-switched to keep Web Bluetooth filters reliable:
the first 10 seconds after boot advertise only the OTA service, and then the
device advertises only the game service. If an OTA client connects during that
initial window, OTA mode is kept active while connected.

