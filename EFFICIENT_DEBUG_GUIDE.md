# PC-Raspberry Pi 効率的デバッグワークフロー

## 🎯 概要
PC（VS Code）でコード開発・デバッグを行い、実機（Raspberry Pi）で動作確認する効率的なワークフローです。

## 📋 事前準備

### PC側の準備
1. **VS Codeでプロジェクトを開く**
   ```bash
   code C:\Users\DELL\251007_Python_minicar_waypoint
   ```

2. **必要なライブラリをインストール**
   ```bash
   pip install pyserial numpy matplotlib
   ```

### Raspberry Pi側の準備
1. **SSH接続を有効化**
2. **I2C/UART を有効化**
   ```bash
   sudo raspi-config
   # Interface Options → I2C → Yes
   # Interface Options → Serial Port → Login shell: No, Hardware: Yes
   ```

3. **プロジェクトディレクトリ作成**
   ```bash
   mkdir ~/minicar
   ```

## 🚀 効率的デバッグワークフロー

### Step 1: PC でのコード開発・デバッグ

#### VS Code デバッガーを使用
1. **F5** を押すか、**実行とデバッグ** パネルから選択：
   - `IMU Debug - PC Mock Mode` : 軽量デバッグ
   - `IMU Debug - Ultimate (PC)` : 詳細デバッグ
   - `Main Control Loop - PC Test` : メインループテスト

#### コマンドラインから実行
```bash
# 環境セットアップ
python debug_workflow.py --mode setup

# PCでモックデバッグ
python debug_workflow.py --mode debug --debug-type mock
```

#### バッチスクリプト使用
```bash
# 環境セットアップ
debug_helper.bat setup

# PCデバッグ
debug_helper.bat debug
```

### Step 2: Raspberry Pi へのデプロイ

#### VS Code タスクから
1. **Ctrl+Shift+P** → `Tasks: Run Task`
2. `Deploy to Raspberry Pi` を選択
3. Raspberry Pi の IP アドレスを入力

#### コマンドラインから
```bash
# デプロイ実行
python debug_workflow.py --mode deploy --raspberry-ip 192.168.1.100

# または
debug_helper.bat deploy 192.168.1.100
```

### Step 3: 実機でのリモートデバッグ

#### リモートデバッグセッション開始
```bash
# リモートデバッグ開始
python debug_workflow.py --mode remote --raspberry-ip 192.168.1.100

# または
debug_helper.bat remote 192.168.1.100
```

#### 並行監視
```bash
# Raspberry Pi 接続監視
python debug_workflow.py --mode monitor --raspberry-ip 192.168.1.100

# または
debug_helper.bat monitor 192.168.1.100
```

### Step 4: ログの同期・解析

```bash
# ログファイル同期
python debug_workflow.py --mode sync --raspberry-ip 192.168.1.100

# または
debug_helper.bat sync 192.168.1.100
```

## 🔧 効率化のポイント

### 1. **開発サイクルの高速化**
```
PC開発 → デプロイ → 実機テスト → ログ同期 → 解析 → PC開発...
```

### 2. **自動環境検出**
- スクリプトが自動でPC/Raspberry Piを判定
- 環境に応じた設定を自動適用

### 3. **デバッグレベルの段階分け**
- **Level 1**: `imu_debug_simple.py` - 基本動作確認
- **Level 2**: `imu_debug_bno055.py` - センサー専用デバッグ  
- **Level 3**: `imu_debug_ultimate.py` - 包括的デバッグ

### 4. **VS Code 統合**
- デバッガー設定済み
- タスク設定済み
- ブレークポイント設定可能

## 📁 ファイル構成

```
📁 251007_Python_minicar_waypoint/
├── 🔧 debug_workflow.py          # メイン管理スクリプト
├── 📝 debug_helper.bat           # Windowsバッチヘルパー
├── 📋 EFFICIENT_DEBUG_GUIDE.md   # このガイド
├── 📁 .vscode/
│   ├── launch.json               # デバッガー設定
│   └── tasks.json                # タスク設定
├── 🐛 imu_debug_simple.py        # 軽量デバッグ
├── 🐛 imu_debug_ultimate.py      # 包括的デバッグ
├── 📁 logs/                      # 同期ログ保存先
└── ⚙️ config.json               # システム設定
```

## 🎨 デバッグ画面の説明

### PC Mock Mode 画面例
```
💻 PC detected - Setting up development mode
🔧 Setting up PC development environment
✅ PC development environment ready

🐛 IMU Debug - Mock Mode
================================
Time: 15.2s | Count: 152 | Temp: 25.5°C
================================
📊 CALIBRATION:
   Sys:3/3 (Perfect)  Gyro:3/3  Acc:3/3  Mag:2/3

🧭 ORIENTATION:
   Roll:   -1.2°   Pitch:   +2.3°   Yaw: +045.6°

⚡ ACCELERATION [m/s²]:
   X:  +0.12   Y:  -0.05   Z:  +9.81

🌀 GYROSCOPE [rad/s]:
   X:  +0.001   Y:  -0.002   Z:  +0.000
```

### Raspberry Pi 実機画面例
```
🍓 Raspberry Pi detected - Setting up hardware mode
✅ Raspberry Pi environment ready

🔗 BNO055 connected on /dev/serial0
📡 Real sensor data streaming...

🐛 IMU Debug - Hardware Mode
================================
Time: 23.7s | Count: 237 | Temp: 28.3°C
================================
📊 CALIBRATION:
   Sys:2/3 (Good)  Gyro:3/3  Acc:1/3  Mag:0/3

🧭 ORIENTATION:
   Roll:   +3.4°   Pitch:   -1.8°   Yaw: +123.7°

⚡ ACCELERATION [m/s²]:
   X:  +0.23   Y:  -0.15   Z:  +9.75

🌀 GYROSCOPE [rad/s]:
   X:  +0.003   Y:  +0.001   Z:  -0.001
```

## 🆘 トラブルシューティング

### よくある問題と解決法

#### 1. Raspberry Pi 接続エラー
```bash
❌ Failed to deploy: Permission denied
```
**解決法**: SSH鍵認証を設定するか、パスワード認証を確認

#### 2. センサー接続エラー  
```bash
❌ Failed to connect to BNO055 sensor
```
**解決法**: 配線確認、I2C有効化確認、権限確認

#### 3. ログ同期エラー
```bash
❌ Failed to sync logs
```
**解決法**: ネットワーク接続確認、rsyncインストール確認

## 🎯 使い分けガイド

| 目的 | 推奨スクリプト | 実行環境 |
|------|----------------|----------|
| コード開発・基本テスト | `imu_debug_simple.py` | PC (Mock) |
| アルゴリズム検証 | `imu_debug_ultimate.py` | PC (Mock) |
| センサー動作確認 | `imu_debug_bno055.py` | Raspberry Pi |
| 統合テスト | `main_control_loop.py` | Raspberry Pi |
| 本番デバッグ | `imu_debug_ultimate.py` | Raspberry Pi |

このワークフローにより、PC での快適な開発環境と実機での正確な動作確認を効率的に組み合わせることができます！