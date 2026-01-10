# 車両モデルをインポート
from minicar_model import MinicarModel
import os
import sys
import json
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, FancyArrow
from matplotlib.widgets import Button, Slider
import math
import time

# インタラクティブモードは使用しない（ブロッキングで表示する）
# ===== コース設定読み込み =====
from course_config import (
    grid_matrix, obstacles, start_lines, pylons,
    x_min, x_max, y_min, y_max, resolution
)

# ===== シミュレーション設定 =====
SIM_DT = 0.05  # シミュレーション更新周期 [s] (50ms = 20Hz)
SPEED_SCALE = 1.0  # 再生速度倍率

# ===== 制御モデルインターフェース =====
def control_step(sensors, imu, state):
    """Dummy control function."""
    # sensors: dict - distance sensors {'Fr': float, 'FrRH': float, ...} [cm]
    # imu: dict - IMU values {'yaw': float, 'accel_x': float, 'accel_y': float}
    # state: dict - car state {'x': float, 'y': float, 'v': float, ...}
    # Returns: tuple (steer, accel)
    # steer: float [-90 ~ +90], accel: float [-100 ~ +100]
    # デフォルト: 直進
    steer = 0.0
    accel = 50.0
    
    # 簡易な壁回避制御
    if sensors['Fr'] < 30:
        steer = 45.0  # 右に回避
    elif sensors['FrRH'] < 20:
        steer = -30.0  # 左に回避
    elif sensors['FrLH'] < 20:
        steer = 30.0  # 右に回避
    
    return steer, accel

# ===== GUIクラス =====
class MinicarSimulator:
    def __init__(self):
        # メインウィンドウ
        self.fig = plt.figure(figsize=(10, 8))
        self.ax_main = plt.subplot2grid((3, 3), (0, 0), rowspan=3, colspan=2)
        self.ax_main.set_aspect('equal', adjustable='box')
        self.ax_main.set_xlim(x_min - 0.2, x_max + 0.2)
        self.ax_main.set_ylim(y_min - 0.2, y_max + 0.2)
        self.ax_main.set_xlabel('X [m]', fontsize=12, fontweight='bold')
        self.ax_main.set_ylabel('Y [m]', fontsize=12, fontweight='bold')
        self.ax_main.set_title('Minicar Simulator v0.1', fontsize=14, fontweight='bold')

        # 情報表示パネル
        self.ax_info = plt.subplot2grid((3, 3), (0, 2))
        self.ax_info.axis('off')

        # センサー表示パネル
        self.ax_sensors = plt.subplot2grid((3, 3), (1, 2))
        self.ax_sensors.axis('off')

        # 車両モデル
        self.car = MinicarModel(x=0.0, y=0.0, yaw=180.0)
        self.sim_time = 0.0
        self.car_artist = None
        self.sensor_rays = []
        self.running = False
        self.last_update_time = None

        self.selected_model_path = None

        self.setup_controls()
        self.draw_course()
        self.draw_car()

        # 日本語フォントを設定
        from matplotlib import rcParams
        rcParams['font.sans-serif'] = ['MS Gothic']  # Windows用の日本語フォント
        rcParams['axes.unicode_minus'] = False

    def on_select_waypoint(self, event):
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk()
        root.withdraw()
        file_path = filedialog.askopenfilename(
            title="走行パターンwaypointのjsonを選択",
            filetypes=[("JSON Files", "*.json")]
        )
        if file_path:
            try:
                with open(file_path, "r", encoding="utf-8") as f:
                    waypoints = json.load(f)
                self.loaded_waypoints = waypoints
                # WP0の座標・向きで自車を初期化
                wp0 = waypoints[0]
                self.car = MinicarModel(x=wp0["x"], y=wp0["y"], yaw=wp0.get("yaw", 0.0))
                self.sim_time = 0.0
                self.draw_car()
                self.fig.canvas.draw_idle()
                print(f"[SIM] Waypointロード: {file_path} / 初期位置: x={wp0['x']}, y={wp0['y']}, yaw={wp0.get('yaw',0.0)}")
            except Exception as e:
                print(f"[SIM] Waypointロード失敗: {e}")
        root.destroy()

    def update_info_panel(self):
        """情報パネルを更新"""
        self.ax_info.clear()
        self.ax_info.axis('off')
        state = self.car.get_state()
        info_text = (
            f"=== Vehicle State ===\n"
            f"Time: {self.sim_time:.2f} s\n"
            f"X: {state['x']:.3f} m\n"
            f"Y: {state['y']:.3f} m\n"
            f"Yaw: {state['yaw']:.1f}°\n"
            f"Speed: {state['v']:.3f} m/s"
        )
        self.ax_info.text(0.1, 0.5, info_text, fontsize=10, family='monospace', verticalalignment='center')

        # センサー値表示
        self.ax_sensors.clear()
        self.ax_sensors.axis('off')

        sensors = state['sensors']
        sensor_text = (
            "=== Sensors [cm] ===\n"
            f"Fr:    {sensors['Fr']:.1f}\n"
            f"L45:   {sensors['FrLH']:.1f}\n"
            f"L90:   {sensors['RrLH']:.1f}\n"
            f"R45:   {sensors['FrRH']:.1f}\n"
            f"R90:   {sensors['RrRH']:.1f}\n"
        )
        self.ax_sensors.text(0.1, 0.5, sensor_text, fontsize=10, family='monospace', verticalalignment='center')

        # WP情報（制御モデルが選択されている場合のみ）
        wp_info = ""
        steer_val = state.get('steer', 0.0)
        accel_val = state.get('accel', 0.0)
        if hasattr(self, 'loaded_waypoints') and self.loaded_waypoints:
            try:
                # 現在のWPインデックスを取得
                current_wp_index = 0  # 必要に応じて現在のインデックスを計算
                current_wp = self.loaded_waypoints[current_wp_index]
                wp_info = (
                    f"\n--- WP Info ---\n"
                    f"WP: {current_wp_index}\n"
                    f"X: {current_wp['x']:.2f} m\n"
                    f"Y: {current_wp['y']:.2f} m\n"
                    f"Steer: {steer_val:.1f}\n"
                    f"Accel: {accel_val:.1f}"
                )
            except Exception as e:
                wp_info = f"\n[WP情報取得エラー] {e}"

        self.ax_info.text(0.1, 0.5, info_text + wp_info, fontsize=10, family='monospace', verticalalignment='center')

        # センサー値表示
        self.ax_sensors.clear()
        self.ax_sensors.axis('off')

        sensors = state['sensors']
        sensor_text = "=== Sensors [cm] ===\n"
        sensor_text += f"Fr:    {sensors['Fr']:.1f}\n"
        sensor_text += f"L45:   {sensors['FrLH']:.1f}\n"
        sensor_text += f"L90:   {sensors['RrLH']:.1f}\n"
        sensor_text += f"R45:   {sensors['FrRH']:.1f}\n"
        sensor_text += f"R90:   {sensors['RrRH']:.1f}\n"

        self.ax_sensors.text(0.1, 0.5, sensor_text, fontsize=10, family='monospace', verticalalignment='center')
        """コース表示の設定"""
        # 背景グリッド
        self.ax_main.imshow(grid_matrix, cmap="Greys", origin="lower",
                           extent=[x_min, x_max, y_min, y_max], alpha=0.7)
        
        # 軸設定
        self.ax_main.set_aspect('equal', adjustable='box')
        self.ax_main.set_xlim(x_min - 0.2, x_max + 0.2)
        self.ax_main.set_ylim(y_min - 0.2, y_max + 0.2)
        self.ax_main.set_xlabel('X [m]', fontsize=12, fontweight='bold')
        self.ax_main.set_ylabel('Y [m]', fontsize=12, fontweight='bold')
        self.ax_main.set_title('Minicar Simulator v0.1', fontsize=14, fontweight='bold')
        
        # 障害物描画
        for obs in obstacles:
            x0, y0 = obs["start"]
            x1, y1 = obs["end"]
            left = min(x0, x1)
            bottom = min(y0, y1)
            width = abs(x1 - x0)
            height = abs(y1 - y0)
            rect = Rectangle((left, bottom), width, height,
                 facecolor="lightgreen", edgecolor='green', alpha=0.6, linewidth=2)
            self.ax_main.add_patch(rect)
        
        # スタートライン描画
        for line_def in start_lines:
            x0, y0 = line_def["start"]
            x1, y1 = line_def["end"]
            self.ax_main.plot([x0, x1], [y0, y1], "b-", linewidth=4, alpha=0.8)
        
        # パイロン描画
        for p in pylons:
            x, y = p["pos"]
            circ = Circle((x, y), radius=0.1, facecolor="darkorange", edgecolor="black")
            self.ax_main.add_patch(circ)
    
    def setup_controls(self):
        """コントロールボタンの設定"""
        # Start/Stopボタン
        ax_start = plt.axes([0.70, 0.05, 0.08, 0.04])
        self.btn_start = Button(ax_start, "Start")
        self.btn_start.on_clicked(self.on_start)
        
        ax_stop = plt.axes([0.79, 0.05, 0.08, 0.04])
        self.btn_stop = Button(ax_stop, "Stop")
        self.btn_stop.on_clicked(self.on_stop)
        
        # Resetボタン
        ax_reset = plt.axes([0.88, 0.05, 0.08, 0.04])
        self.btn_reset = Button(ax_reset, "Reset")
        self.btn_reset.on_clicked(self.on_reset)
        
        # 速度係数スライダー
        ax_coef = plt.axes([0.70, 0.12, 0.26, 0.02])
        self.slider_coef = Slider(ax_coef, "Speed Coef", 0.001, 0.05,
                                  valinit=0.01, valstep=0.001)
        self.slider_coef.on_changed(self.on_coef_changed)

        # 制御モデル選択ボタン
        ax_model = plt.axes([0.70, 0.18, 0.26, 0.04])
        self.btn_model = Button(ax_model, "制御モデル選択")
        self.btn_model.on_clicked(self.on_select_model)

        # waypoint選択ボタン
        ax_wp = plt.axes([0.70, 0.24, 0.26, 0.04])
        self.btn_wp = Button(ax_wp, "waypoint選択")
        self.btn_wp.on_clicked(self.on_select_waypoint)

        self.selected_model_path = None
        self.loaded_waypoints = None
        self.user_module = None
        self.frame_count = 0
        self.draw_skip = 2  # 描画を何フレームごとに行うか（1で毎フレーム）

    def on_select_model(self, event):
        """制御モデル選択ボタンのコールバック"""
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk()
        root.withdraw()
        file_path = filedialog.askopenfilename(
            title="制御モデルPythonファイルを選択",
            filetypes=[("Python Files", "*.py")]
        )
        root.destroy()
        if file_path:
            self.selected_model_path = file_path
            # 選択時にモジュールをロードしてキャッシュする
            try:
                import importlib.util
                module_name = "user_control_model"
                spec = importlib.util.spec_from_file_location(module_name, file_path)
                user_module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(user_module)
                self.user_module = user_module
                print(f"[SIM] 制御モデル選択・ロード成功: {file_path}")
            except Exception as e:
                self.user_module = None
                print(f"[SIM] 制御モデルロード失敗: {e}")
    
    def draw_car(self):
        """車両を描画 (部分的な再描画を使用)"""
        if self.car_artist:
            self.car_artist.remove()
        state = self.car.get_state()
        self.car_artist = FancyArrow(
            state['x'], state['y'],
            0.5 * math.cos(math.radians(state['yaw'])),
            0.5 * math.sin(math.radians(state['yaw'])),
            width=0.1, color='blue'
        )
        self.ax_main.add_patch(self.car_artist)
        self.fig.canvas.draw_idle()  # 部分的な再描画(blit)は使用しない

        # センサーレイ描画
        sensors = state['sensors']
        for sensor_name, distance in sensors.items():
            config = self.car.sensor_config[sensor_name]
            sensor_angle = (state['yaw'] + config['angle']) % 360
            sensor_angle_rad = math.radians(sensor_angle)
            
            # センサー位置
            offset_x = config['offset_x']
            offset_y = config['offset_y']
            sensor_x = state['x'] + offset_x * math.cos(math.radians(state['yaw'])) - offset_y * math.sin(math.radians(state['yaw']))
            sensor_y = state['y'] + offset_x * math.sin(math.radians(state['yaw'])) + offset_y * math.cos(math.radians(state['yaw']))
            
            # レイの終点
            ray_dist = min(distance / 100.0, 2.0)  # cm→m、最大2m
            ray_end_x = sensor_x + ray_dist * math.cos(sensor_angle_rad)
            ray_end_y = sensor_y + ray_dist * math.sin(sensor_angle_rad)
            
            # レイ描画
            color = 'yellow' if distance < 30 else 'cyan'
            ray, = self.ax_main.plot([sensor_x, ray_end_x], [sensor_y, ray_end_y],
                                     color=color, linewidth=1.5, alpha=0.6)
            self.sensor_rays.append(ray)
        
        # 情報表示更新
        self.update_info_panel()
    
    def draw_sensors(self):
        """センサーを描画"""
        # 既存のセンサー描画を削除
        for ray in self.sensor_rays:
            ray.remove()
        self.sensor_rays = []

        # 新しいセンサーを描画
        state = self.car.get_state()
        sensors = state['sensors']
        # センサー名から相対角度を決めるマッピング（単位: 度）
        sensor_angle_map = {
            'Fr': 0,
            'FrLH': 45,   # front-left 45deg
            'RrLH': 90,   # rear-left ~90deg
            'FrRH': -45,  # front-right 45deg
            'RrRH': -90   # rear-right ~90deg
        }
        for name, distance in sensors.items():
            try:
                if distance is None or distance <= 0:
                    continue
                # センサーは cm 単位のことが多いので m に変換
                dist_m = float(distance) / 100.0
                rel_angle = sensor_angle_map.get(name, 0)
                angle = math.radians(state['yaw'] + rel_angle)
                x_end = state['x'] + dist_m * math.cos(angle)
                y_end = state['y'] + dist_m * math.sin(angle)
                ray, = self.ax_main.plot([state['x'], x_end], [state['y'], y_end], color='red')
                self.sensor_rays.append(ray)
            except Exception:
                # 値が予期せぬ型の場合は無視して続行
                continue

    def update_info_panel(self):
        """情報パネルを更新"""
        self.ax_info.clear()
        self.ax_info.axis('off')
        
        state = self.car.get_state()
        info_text = (
            f"=== Vehicle State ===\n"
            f"Time: {self.sim_time:.2f} s\n"
            f"X: {state['x']:.3f} m\n"
            f"Y: {state['y']:.3f} m\n"
            f"Yaw: {state['yaw']:.1f}°\n"
            f"Speed: {state['v']:.3f} m/s"
        )
        
        self.ax_info.text(0.1, 0.5, info_text, fontsize=10, family='monospace',
                         verticalalignment='center')
        
        # センサー値表示
        self.ax_sensors.clear()
        self.ax_sensors.axis('off')
        
        sensors = state['sensors']
        sensor_text = "=== Sensors [cm] ===\n"
        sensor_text += f"Fr:    {sensors['Fr']:.1f}\n"
        sensor_text += f"L45:   {sensors['FrLH']:.1f}\n"
        sensor_text += f"L90:   {sensors['RrLH']:.1f}\n"
        sensor_text += f"R45:   {sensors['FrRH']:.1f}\n"
        sensor_text += f"R90:   {sensors['RrRH']:.1f}\n"
        
        self.ax_sensors.text(0.1, 0.5, sensor_text, fontsize=10, family='monospace',
                            verticalalignment='center')
    
    def simulation_step(self):
        """シミュレーション1ステップ実行"""
        state = self.car.get_state()
        sensors = state['sensors']
        imu = state['imu']

        steer, accel = None, None
        # 制御モデルが選択されていれば、外部ファイルのcontrol_stepを呼び出す
        if self.user_module:
            try:
                if hasattr(self.user_module, "control_step"):
                    steer, accel = self.user_module.control_step(sensors, imu, state)
                else:
                    print(f"[SIM] 選択モデルにcontrol_step関数がありません。デフォルト制御を使用します。")
            except FileNotFoundError as e:
                print(f"[SIM] 制御モデル内でファイルが見つかりません: {e}")
            except Exception as e:
                print(f"[SIM] 制御モデル実行エラー: {e}")
        # なければデフォルト
        if steer is None or accel is None:
            steer, accel = control_step(sensors, imu, state)

        # 制御入力設定
        self.car.set_control(steer, accel)

        # 物理シミュレーション更新
        self.car.update(SIM_DT, obstacles, grid_matrix, resolution, x_min, y_min)

        # 時間更新
        self.sim_time += SIM_DT

        # 描画更新（負荷軽減のため、描画は数フレームに1回にする）
        self.frame_count += 1
        if self.frame_count % self.draw_skip == 0:
            self.draw_car()
            self.draw_sensors()
            try:
                self.fig.canvas.draw()
                self.fig.canvas.flush_events()
            except Exception:
                # 描画が例外を投げる場合は無視して続行
                pass
    
    def on_start(self, event):
        """Start ボタンクリック"""
        if not self.running:
            self.running = True
            self.last_update_time = time.time()
            self.timer_id = self.fig.canvas.new_timer(interval=int(SIM_DT * 1000))
            self.timer_id.add_callback(self.timer_callback)
            self.timer_id.start()
            print("[SIM] Started")
    
    def on_stop(self, event):
        """Stop ボタンクリック"""
        if self.running:
            self.running = False
            if hasattr(self, 'timer_id'):
                self.timer_id.stop()
            print("[SIM] Stopped")
    
    def on_reset(self, event):
        """Reset ボタンクリック"""
        self.on_stop(None)
        self.car = MinicarModel(x=0.0, y=0.0, yaw=180.0)
        self.sim_time = 0.0
        self.draw_car()
        self.fig.canvas.draw_idle()
        print("[SIM] Reset")
    
    def on_coef_changed(self, val):
        """速度係数スライダー変更"""
        self.car.set_duty_to_speed_coef(val)
        print(f"[SIM] Speed coefficient: {val:.3f} m/s per duty%")
    
    def timer_callback(self):
        """タイマーコールバック"""
        if self.running:
            self.simulation_step()
    
    def run(self):
        """シミュレーション実行"""
        # Ensure interactive mode is off so plt.show() blocks and the GUI mainloop runs
        plt.ioff()
        print("[SIM] GUI start")
        plt.show()

    def draw_course(self):
        """コースを描画するメソッド"""
        # 背景グリッド
        self.ax_main.imshow(grid_matrix, cmap="Greys", origin="lower",
                            extent=[x_min, x_max, y_min, y_max], alpha=0.7)

        # 障害物描画
        for obs in obstacles:
            x0, y0 = obs["start"]
            x1, y1 = obs["end"]
            left = min(x0, x1)
            bottom = min(y0, y1)
            width = abs(x1 - x0)
            height = abs(y1 - y0)
            rect = Rectangle((left, bottom), width, height,
                 facecolor="lightgreen", edgecolor='green', alpha=0.6, linewidth=2)
            self.ax_main.add_patch(rect)

        # スタートライン描画
        for line_def in start_lines:
            x0, y0 = line_def["start"]
            x1, y1 = line_def["end"]
            self.ax_main.plot([x0, x1], [y0, y1], "b-", linewidth=4, alpha=0.8)

        # デフォルトのウェイポイントデータ
        default_waypoints = [
            {"x": 0.0, "y": 0.0, "yaw": 0.0},
            {"x": 1.0, "y": 1.0, "yaw": 45.0},
            {"x": 2.0, "y": 2.0, "yaw": 90.0},
        ]

        # 制御モデル内でファイル存在チェック
        if not os.path.exists('waypoints_qualifying.json'):
            print("[SIM] デフォルトのウェイポイントを使用します。")
            self.loaded_waypoints = default_waypoints
        else:
            print("[SIM] waypoints_qualifying.json を使用します。")
            # ファイルが存在する場合の処理（既存のロジック）
            try:
                with open('waypoints_qualifying.json', "r", encoding="utf-8") as f:
                    waypoints = json.load(f)
                self.loaded_waypoints = waypoints
                # WP0の座標・向きで自車を初期化
                wp0 = waypoints[0]
                self.car = MinicarModel(x=wp0["x"], y=wp0["y"], yaw=wp0.get("yaw", 0.0))
                self.sim_time = 0.0
                self.draw_car()
                self.fig.canvas.draw_idle()
                print(f"[SIM] Waypointロード: waypoints_qualifying.json / 初期位置: x={wp0['x']}, y={wp0['y']}, yaw={wp0.get('yaw',0.0)}")
            except Exception as e:
                print(f"[SIM] Waypointロード失敗: {e}")

        # パイロン描画
        for p in pylons:
            x, y = p["pos"]
            circ = Circle((x, y), radius=0.1, facecolor="darkorange", edgecolor="black")
            self.ax_main.add_patch(circ)
    
# ===== メイン実行 =====
if __name__ == "__main__":
    print("="*60)
    print("  Minicar Simulator v0.1")
    print("  Update Rate: 20 Hz (50ms)")
    print("="*60)
    
    sim = MinicarSimulator()
    sim.run()
