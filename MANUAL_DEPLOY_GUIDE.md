# 最簡単デプロイガイド

## 🎯 Raspberry Pi への手動ファイル転送

### 方法1: WinSCP使用（推奨）
1. WinSCPをダウンロード・インストール
2. 接続設定:
   - ホスト: 172.20.10.3
   - ユーザー: pi
   - パスワード: （ラズパイのパスワード）
3. 右側パネルで ~/minicar フォルダ作成
4. 以下ファイルをドラッグ&ドロップ:
   - imu_debug_ultimate.py
   - imu_debug_simple.py
   - config.json
   - raspberry_pi_uart_setup.sh

### 方法2: PuTTY + nano使用
1. PuTTYでSSH接続: pi@172.20.10.3
2. ラズパイ側で:
   ```bash
   mkdir -p ~/minicar
   cd ~/minicar
   nano imu_debug_simple.py
   ```
3. ファイル内容をコピー&ペースト

### 方法3: GitHub経由
1. ファイルをGitHubにコミット&プッシュ
2. ラズパイ側で:
   ```bash
   git clone https://github.com/yourusername/your-repo.git
   cd your-repo
   ```

## 🍓 Raspberry Pi側での実行
```bash
cd ~/minicar
python3 imu_debug_simple.py
```

## 🔧 UART設定（必要に応じて）
```bash
chmod +x raspberry_pi_uart_setup.sh
./raspberry_pi_uart_setup.sh
sudo reboot
```