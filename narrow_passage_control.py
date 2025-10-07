# 狭路判定・制御モジュール

def is_in_narrow_passage(LH, RH, threshold=20):
    """
    LH, RH: 左右超音波距離[cm]
    threshold: 狭路判定閾値[cm]
    """
    return LH < threshold and RH < threshold

def narrow_passage_control(motor, LH, RH, lane="left", base_speed=30):
    """
    狭路通過時の制御
    motor: モータ制御インスタンス
    LH, RH: 左右超音波距離[cm]
    lane: "left", "center", "right" いずれか
    base_speed: 狭路時の速度
    """
    motor.accel(base_speed)
    if lane == "left":
        # 左レーン：左壁に寄せる
        steer_correction = (RH - LH) * 2 - 10
    elif lane == "center":
        # 中央レーン：左右均等
        steer_correction = (RH - LH) * 2
    elif lane == "right":
        # 右レーン：右壁に寄せる
        steer_correction = (RH - LH) * 2 + 10
    else:
        steer_correction = (RH - LH) * 2
    motor.steer(steer_correction)