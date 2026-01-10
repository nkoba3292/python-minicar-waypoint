import math
import json

# シミュレーション用: waypointsファイルを自動選択（予選）
WAYPOINT_FILE = 'waypoints_qualifying.json'
with open(WAYPOINT_FILE, 'r') as f:
    waypoints = json.load(f)
n_wp = len(waypoints)

# 状態管理用グローバル変数（シミュレーション用）
idx = 1
lap = 1
current_x = waypoints[0]['x']
current_y = waypoints[0]['y']
velocity_x, velocity_y = 0.0, 0.0
initial_yaw_offset = 0.0
position_tracking_enabled = True
narrow_passage_active = False
control_mode = "normal"
avoidance_start_wp = None
avoidance_start_time = None
checkpoint_wp_idx = None
target_checkpoint = None
race_mode = "qualifying"
TIME_LIMIT = None
wp_start_time = None
accumulated_distance = 0.0

# パラメータ
Cshort = 20
short = 70
FORWARD_S = 70
FORWARD_C = 70
REVERSE = -60
LEFT = -90
CENTER = 0
RIGHT = 90
DEBUG_MODE = True
SPEED_SCALE = 0.5

# 予選WP復帰マップ
RECOVERY_MAP = [
    (3, 8, 9), (10, 23, 24), (27, 32, 33), (34, 47, 48), (51, 56, 57), (58, 71, 72)
]

def get_recovery_waypoint_qualifying(current_wp_idx):
    for start, end, recovery in RECOVERY_MAP:
        if start <= current_wp_idx <= end:
            return recovery
    return None

def find_waypoint_by_checkpoint(checkpoint_type):
    for wp_idx, wp in enumerate(waypoints):
        if wp.get('checkpoint') == checkpoint_type:
            return wp_idx
    return None

def control_step(sensors, imu, state):
    global idx, lap, current_x, current_y, velocity_x, velocity_y, initial_yaw_offset
    global position_tracking_enabled, narrow_passage_active, control_mode
    global avoidance_start_wp, avoidance_start_time, checkpoint_wp_idx, target_checkpoint
    global wp_start_time, accumulated_distance

    # IMU値取得
    raw_yaw = imu['yaw']
    current_yaw = (raw_yaw + initial_yaw_offset) % 360
    accel_x = imu['accel_x']
    accel_y = imu['accel_y']

    # 現在waypoint取得
    wp = waypoints[idx]
    wp_x, wp_y = wp['x'], wp['y']
    target_speed = wp.get('v', 50)
    target_yaw = wp.get('yaw', None)

    # デバッグモード時の速度調整
    if DEBUG_MODE:
        target_speed = target_speed * SPEED_SCALE
        if target_speed < 20:
            target_speed = 20

    # 狭路判定・制御
    if wp.get('narrow', False):
        control_mode = "narrow_passage"
        narrow_passage_active = True
        if lap == 1:
            lane = "left"
        elif lap == 2:
            lane = "center"
        elif lap == 3:
            lane = "right"
        elif lap == 4:
            lane = "left"
        else:
            lane = "center"
        base_speed = 30
        # レーンに応じてステアリング補正
        LHdis = sensors['FrLH']
        RHdis = sensors['FrRH']
        if lane == "left":
            steer_correction = (LHdis - RHdis) * 2 - 20
        elif lane == "center":
            steer_correction = (LHdis - RHdis) * 2
        elif lane == "right":
            steer_correction = (LHdis - RHdis) * 2 + 20
        else:
            steer_correction = (LHdis - RHdis) * 2
        return steer_correction, base_speed

    # 狭路通過完了検出
    if narrow_passage_active and not wp.get('narrow', False):
        control_mode = "normal"
        narrow_passage_active = False
        lap += 1

    # 障害物判定
    FRdis = sensors['Fr']
    LHdis = sensors['FrLH']
    RHdis = sensors['FrRH']
    if FRdis < Cshort:
        if control_mode != "obstacle_avoidance":
            control_mode = "obstacle_avoidance"
            avoidance_start_wp = idx
            avoidance_start_time = state.get('time', 0)
            position_tracking_enabled = False
            checkpoint_wp_idx = get_recovery_waypoint_qualifying(idx)
            if checkpoint_wp_idx is not None:
                target_checkpoint = f"WP{checkpoint_wp_idx}"
            else:
                if idx < n_wp // 2:
                    target_checkpoint = "middle_mat"
                    checkpoint_wp_idx = find_waypoint_by_checkpoint("middle_mat")
                else:
                    target_checkpoint = "start_line"
                    checkpoint_wp_idx = find_waypoint_by_checkpoint("start_line")
        return RIGHT, REVERSE

    # 回避モードから復帰チェック
    if control_mode == "obstacle_avoidance" and FRdis >= Cshort + 10:
        control_mode = "recovery"

    # 復帰モード処理
    if control_mode == "recovery":
        if checkpoint_wp_idx is not None:
            idx = checkpoint_wp_idx
            current_x = waypoints[idx]['x']
            current_y = waypoints[idx]['y']
            velocity_x = 0.0
            velocity_y = 0.0
            position_tracking_enabled = True
            control_mode = "normal"
        return CENTER, 0

    # Yaw制御ロジック
    if target_yaw is not None:
        current_yaw_rad = math.radians(current_yaw)
        target_yaw_rad = math.radians(target_yaw)
        yaw_error = target_yaw_rad - current_yaw_rad
        while yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        while yaw_error < -math.pi:
            yaw_error += 2 * math.pi
        steer_gain = 2.0
        steer_angle = steer_gain * math.degrees(yaw_error)
        steer_angle = max(-90, min(90, steer_angle))
        return steer_angle, target_speed

    # target_yawがない場合の元の制御
    if FRdis >= Cshort:
        if LHdis <= short and RHdis >= short:
            return RIGHT, FORWARD_C
        elif LHdis > short and RHdis < short:
            return LEFT, FORWARD_C
        elif LHdis < short and RHdis < short:
            if (LHdis - RHdis) > 10:
                return LEFT, FORWARD_C
            elif (RHdis - LHdis) > 10:
                return RIGHT, FORWARD_C
            else:
                return CENTER, FORWARD_S
        else:
            return CENTER, FORWARD_S
    else:
        return CENTER, REVERSE

    # fallback
    return CENTER, FORWARD_S
