# センサー融合の実装例
# main_control_loop.py に組み込む

class VehicleStateEstimator:
    """車両状態推定クラス - 複数センサーを統合"""
    
    def __init__(self, imu_sensor, wheel_odometry):
        self.imu = imu_sensor
        self.odometry = wheel_odometry
        
        # 融合パラメータ
        self.imu_weight = 0.3      # IMUの重み（短期精度）
        self.odometry_weight = 0.7  # オドメトリの重み（長期安定性）
        
        # 統合位置
        self.fused_x = 0.0
        self.fused_y = 0.0
        
        # ウェイポイント到達フラグ
        self.last_waypoint_pos = (0.0, 0.0)
    
    def update(self, current_yaw):
        """センサー融合による位置更新"""
        
        # IMUから移動量取得
        imu_x, imu_y, _, _ = self.imu.get_position()
        
        # オドメトリから移動量取得
        odo_distance = self.odometry.get_distance()
        
        # YAW角度を使ってオドメトリをX,Y成分に分解
        import math
        odo_x = odo_distance * math.cos(math.radians(current_yaw))
        odo_y = odo_distance * math.sin(math.radians(current_yaw))
        
        # ✅ 重み付き融合（シンプルな加重平均）
        self.fused_x = (self.imu_weight * imu_x + 
                       self.odometry_weight * odo_x)
        self.fused_y = (self.imu_weight * imu_y + 
                       self.odometry_weight * odo_y)
        
        return (self.fused_x, self.fused_y)
    
    def on_waypoint_reached(self, waypoint_pos):
        """ウェイポイント到達時の処理"""
        
        # ✅ IMU位置をリセット（ドリフト累積防止）
        self.imu.reset_position()
        
        # ✅ オドメトリもリセット
        self.odometry.reset()
        
        # ウェイポイント位置を記録
        self.last_waypoint_pos = waypoint_pos
        
        print(f"📍 Waypoint reached: {waypoint_pos}, IMU/Odometry reset")
    
    def get_position(self):
        """融合後の位置を取得"""
        return (self.fused_x, self.fused_y)


# main_control_loop.py での使用例
def main_control_loop():
    # センサー初期化
    imu = IMUSensorBNO055()
    odometry = WheelOdometry()
    
    # 状態推定器
    state_estimator = VehicleStateEstimator(imu, odometry)
    
    # Pure Pursuit制御
    pure_pursuit = PurePursuit(lookahead_distance=0.5)
    
    while True:
        # センサーデータ取得
        imu_data, error = imu.get_essential()
        yaw = imu_data['yaw']
        accel = imu_data['accel']
        
        # オドメトリ更新
        odometry.update()
        
        # IMU位置更新
        imu.update_position(accel[0], accel[1])
        
        # ✅ センサー融合
        current_pos = state_estimator.update(yaw)
        
        # Pure Pursuit制御計算
        target_waypoint = get_next_waypoint()
        steering = pure_pursuit.calculate_steering(current_pos, target_waypoint)
        
        # ウェイポイント到達判定
        if distance_to(current_pos, target_waypoint) < THRESHOLD:
            state_estimator.on_waypoint_reached(target_waypoint)
            advance_to_next_waypoint()
        
        # モーター制御
        motor_control(steering)
