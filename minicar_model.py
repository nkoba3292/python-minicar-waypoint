"""
ミニカーシミュレーションモデル
1/10 ラジコンカー - アッカーマンステアリングモデル
"""
import numpy as np
import math

class MinicarModel:
    """
    1/10スケールラジコンカーのシミュレーションモデル
    アッカーマンステアリング + 超音波センサー5個 + IMU
    """
    
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        """
        初期化
        
        Args:
            x: 初期X座標 [m]
            y: 初期Y座標 [m]
            yaw: 初期ヨー角 [degree]
        """
        # 車両状態
        self.x = x          # X座標 [m]
        self.y = y          # Y座標 [m]
        self.yaw = yaw      # ヨー角 [degree]
        self.v = 0.0        # 速度 [m/s]
        
        # 車両パラメータ（1/10スケール）
        self.wheelbase = 0.257  # ホイールベース [m] (実車2.57mの1/10)
        self.max_steer = 30.0   # 最大舵角 [degree]
        self.max_speed = 3.0    # 最大速度 [m/s] (約10.8km/h)
        self.max_accel = 2.0    # 最大加速度 [m/s²]
        self.max_decel = 3.0    # 最大減速度 [m/s²]
        
        # Duty比→速度変換係数（調整可能）
        self.duty_to_speed_coef = 0.01  # [m/s per duty%] (duty100 = 1.0m/s)
        
        # センサー配置（車体中心からの相対位置）
        self.sensor_config = {
            'Fr': {'angle': 0, 'offset_x': 0.15, 'offset_y': 0.0},      # 前方
            'FrRH': {'angle': 45, 'offset_x': 0.10, 'offset_y': -0.08},  # 右前45度
            'RrRH': {'angle': 90, 'offset_x': 0.0, 'offset_y': -0.10},   # 右90度
            'FrLH': {'angle': -45, 'offset_x': 0.10, 'offset_y': 0.08},  # 左前45度
            'RrLH': {'angle': -90, 'offset_x': 0.0, 'offset_y': 0.10}    # 左90度
        }
        
        # センサー値（距離 [cm]）
        self.sensor_distances = {
            'Fr': 200.0,
            'FrRH': 200.0,
            'RrRH': 200.0,
            'FrLH': 200.0,
            'RrLH': 200.0
        }
        
        # IMUデータ
        self.accel_x = 0.0  # X方向加速度 [m/s²]
        self.accel_y = 0.0  # Y方向加速度 [m/s²]
        
        # 制御入力
        self.steer_angle = 0.0   # ステアリング角度 [-90 ~ +90]
        self.accel_command = 0.0 # アクセル指令値 [-100 ~ +100]
    
    def set_control(self, steer, accel):
        """
        制御入力を設定
        
        Args:
            steer: ステアリング角度 [-90 ~ +90] (負=左、正=右)
            accel: アクセル指令値 [-100 ~ +100] (負=後退、正=前進)
        """
        # ステアリング角度制限
        self.steer_angle = np.clip(steer, -90, 90)
        
        # アクセル指令値制限
        self.accel_command = np.clip(accel, -100, 100)
    
    def update(self, dt, obstacles, grid_matrix, resolution, x_min, y_min):
        """
        車両状態を更新（アッカーマンモデル）
        
        Args:
            dt: 時間ステップ [s]
            obstacles: 障害物リスト
            grid_matrix: グリッドマトリックス
            resolution: 解像度 [m/px]
            x_min, y_min: グリッド原点座標 [m]
        """
        # 目標速度計算（Duty比から速度へ変換）
        target_v = self.accel_command * self.duty_to_speed_coef
        
        # 加速度計算
        if target_v > self.v:
            accel = self.max_accel
        elif target_v < self.v:
            accel = -self.max_decel
        else:
            accel = 0.0
        
        # 速度更新
        self.v += accel * dt
        self.v = np.clip(self.v, -self.max_speed, self.max_speed)
        
        # ステアリング角度をラジアンに変換（制限付き）
        steer_rad = math.radians(np.clip(self.steer_angle, -self.max_steer, self.max_steer))
        
        # アッカーマンステアリングモデル
        if abs(steer_rad) < 0.001:
            # 直進
            dx = self.v * math.cos(math.radians(self.yaw)) * dt
            dy = self.v * math.sin(math.radians(self.yaw)) * dt
            dyaw = 0.0
        else:
            # 旋回半径計算
            turning_radius = self.wheelbase / math.tan(steer_rad)
            
            # 角速度計算
            omega = self.v / turning_radius  # [rad/s]
            
            # 位置・姿勢更新
            dyaw = math.degrees(omega * dt)
            
            # 円弧運動
            arc_length = self.v * dt
            dx = arc_length * math.cos(math.radians(self.yaw + dyaw/2))
            dy = arc_length * math.sin(math.radians(self.yaw + dyaw/2))
        
        # 状態更新（壁衝突前の座標保存）
        prev_x, prev_y = self.x, self.y
        self.x += dx
        self.y += dy
        self.yaw = (self.yaw + dyaw) % 360

        # 壁衝突判定・補正
        grid_x = int((self.x - x_min) / resolution)
        grid_y = int((self.y - y_min) / resolution)
        out_of_bounds = (
            grid_x < 0 or grid_x >= grid_matrix.shape[1] or
            grid_y < 0 or grid_y >= grid_matrix.shape[0]
        )
        hit_wall = False
        if not out_of_bounds:
            if grid_matrix[grid_y, grid_x] == 1:
                hit_wall = True
        if out_of_bounds or hit_wall:
            # 衝突時は直前座標に戻し、速度ゼロ
            self.x = prev_x
            self.y = prev_y
            self.v = 0.0
            # センサー値も壁までの距離=0cmに強制補正
            for k in self.sensor_distances.keys():
                self.sensor_distances[k] = 0.0

        # 加速度計算（車体座標系）
        self.accel_x = accel * math.cos(steer_rad)
        self.accel_y = accel * math.sin(steer_rad)

        # センサー値更新
        self.update_sensors(obstacles, grid_matrix, resolution, x_min, y_min)
    
    def update_sensors(self, obstacles, grid_matrix, resolution, x_min, y_min):
        """
        超音波センサー値を更新
        
        Args:
            obstacles: 障害物リスト
            grid_matrix: グリッドマトリックス
            resolution: 解像度 [m/px]
            x_min, y_min: グリッド原点座標 [m]
        """
        for sensor_name, config in self.sensor_config.items():
            # センサーのグローバル位置計算
            yaw_rad = math.radians(self.yaw)
            offset_x = config['offset_x']
            offset_y = config['offset_y']
            
            sensor_x = self.x + offset_x * math.cos(yaw_rad) - offset_y * math.sin(yaw_rad)
            sensor_y = self.y + offset_x * math.sin(yaw_rad) + offset_y * math.cos(yaw_rad)
            
            # センサーの向き
            sensor_angle = (self.yaw + config['angle']) % 360
            
            # レイキャスト（最大2m）
            distance = self.raycast(sensor_x, sensor_y, sensor_angle, 
                                   obstacles, grid_matrix, resolution, x_min, y_min)
            
            # センサー値更新（cm単位）
            self.sensor_distances[sensor_name] = distance * 100.0
    
    def raycast(self, x, y, angle, obstacles, grid_matrix, resolution, x_min, y_min, max_dist=2.0):
        """
        レイキャスト（障害物までの距離計算）
        
        Args:
            x, y: センサー位置 [m]
            angle: センサー方向 [degree]
            obstacles: 障害物リスト
            grid_matrix: グリッドマトリックス
            resolution: 解像度 [m/px]
            x_min, y_min: グリッド原点座標 [m]
            max_dist: 最大検出距離 [m]
        
        Returns:
            距離 [m]
        """
        angle_rad = math.radians(angle)
        step = resolution / 2  # レイのステップサイズ
        
        for dist in np.arange(0, max_dist, step):
            ray_x = x + dist * math.cos(angle_rad)
            ray_y = y + dist * math.sin(angle_rad)
            
            # グリッド座標に変換
            grid_x = int((ray_x - x_min) / resolution)
            grid_y = int((ray_y - y_min) / resolution)
            
            # グリッド範囲外チェック
            if (grid_x < 0 or grid_x >= grid_matrix.shape[1] or 
                grid_y < 0 or grid_y >= grid_matrix.shape[0]):
                return max_dist
            
            # 障害物チェック
            if grid_matrix[grid_y, grid_x] == 1:
                return dist
        
        return max_dist
    
    def get_state(self):
        """
        現在の状態を取得
        
        Returns:
            dict: 状態辞書
        """
        return {
            'x': self.x,
            'y': self.y,
            'yaw': self.yaw,
            'v': self.v,
            'steer': self.steer_angle,
            'accel': self.accel_command,
            'sensors': self.sensor_distances.copy(),
            'imu': {
                'yaw': self.yaw,
                'accel_x': self.accel_x,
                'accel_y': self.accel_y
            }
        }
    
    def set_duty_to_speed_coef(self, coef):
        """
        Duty比→速度変換係数を設定
        
        Args:
            coef: 変換係数 [m/s per duty%]
        """
        self.duty_to_speed_coef = coef
