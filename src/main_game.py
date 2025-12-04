import time, math
import board, busio, displayio, terminalio
from adafruit_display_text import label
import i2cdisplaybus, adafruit_displayio_ssd1306
import adafruit_adxl34x

# =============== OLED ===============
displayio.release_displays()
i2c = board.I2C()
display_bus = i2cdisplaybus.I2CDisplayBus(i2c, device_address=0x3C)
display = adafruit_displayio_ssd1306.SSD1306(display_bus, width=128, height=64)

def show4(l1="", l2="", l3="", l4=""):
    g = displayio.Group()
    for i, s in enumerate([l1, l2, l3, l4]):
        t = label.Label(terminalio.FONT, text=s, x=0, y=12 + i*14, color=0xFFFFFF)
        g.append(t)
    display.root_group = g

# =============== IMUs ===============
# 手 + 大腿 两个 ADXL345（接线跟你机器学习动作测试一样）
acc_hand = adafruit_adxl34x.ADXL345(i2c, address=0x53)  # 右手
acc_leg  = adafruit_adxl34x.ADXL345(i2c, address=0x1D)  # 大腿

alpha = 0.2
fx_h = fy_h = fz_h = 0.0
fx_l = fy_l = fz_l = 0.0

def read_filtered_hand():
    global fx_h, fy_h, fz_h
    x, y, z = acc_hand.acceleration
    fx_h = alpha * x + (1 - alpha) * fx_h
    fy_h = alpha * y + (1 - alpha) * fy_h
    fz_h = alpha * z + (1 - alpha) * fz_h
    return fx_h, fy_h, fz_h

def read_filtered_leg():
    global fx_l, fy_l, fz_l
    x, y, z = acc_leg.acceleration
    fx_l = alpha * x + (1 - alpha) * fx_l
    fy_l = alpha * y + (1 - alpha) * fy_l
    fz_l = alpha * z + (1 - alpha) * fz_l
    return fx_l, fy_l, fz_l

def compute_features_hand():
    x, y, z = read_filtered_hand()
    pitchx = math.degrees(math.atan2(-x, math.sqrt(y*y + z*z)))
    pitchy = math.degrees(math.atan2(-y, math.sqrt(x*x + z*z)))
    pitchz = math.degrees(math.atan2(-z, math.sqrt(x*x + y*y)))
    mag = math.sqrt(x*x + y*y + z*z)
    return pitchx, pitchy, pitchz, mag

def compute_features_leg():
    x, y, z = read_filtered_leg()
    pitchx = math.degrees(math.atan2(-x, math.sqrt(y*y + z*z)))
    pitchy = math.degrees(math.atan2(-y, math.sqrt(x*x + z*z)))
    pitchz = math.degrees(math.atan2(-z, math.sqrt(x*x + y*y)))
    mag = math.sqrt(x*x + y*y + z*z)
    return pitchx, pitchy, pitchz, mag

STILL_MAG_EPS_HAND = 0.6
STILL_MAG_EPS_LEG  = 0.6

# =============== 模板（直接从你的训练脚本搬过来） ===============
profiles_hand = {
    '前挥手': {
        'mean': [-58.89, -4.28,  -30.20, 11.41],
        'std' : [ 7.83,   5.01,    7.83,  2.95]
    },
    '右挥手': {
        'mean': [-15.07,  66.14,  -3.57, 10.14],
        'std' : [ 8.16,  12.72,  19.49,  0.87]
    },
    '左挥手': {
        'mean': [-16.73, -65.88,  -9.34, 11.07],
        'std' : [ 9.52,  12.82,  14.99,  1.26]
    },
    '右转圈': {
        'mean': [-12.38,  -6.57,  14.07, 10.18],
        'std' : [ 13.29,  24.53,  63.55,  3.76]
    },
    '左转圈': {
        'mean': [-13.44,  -9.92,  -7.61, 11.27],
        'std' : [ 23.82,  19.47,  60.31,  4.43]
    },
    '出击': {
        'mean': [-0.13,  -6.78, -63.09, 11.38],
        'std' : [27.68,  11.10,  15.99,  2.35]
    }
}

profiles_leg = {
    '左摆腿': {
        'mean': [69.19, -6.61, -16.87, 10.44],
        'std' : [ 8.21,  9.46,   8.36,  1.01]
    },
    '右摆腿': {
        'mean': [76.19,  3.73,   0.26, 10.35],
        'std' : [ 7.76, 12.31,   8.97,  2.13]
    },
    '小跑': {
        'mean': [74.10, -9.33,  -3.00, 10.61],
        'std' : [ 7.56, 12.15,   7.86,  1.65]
    }
}

def classify(feature, profiles, mag, still_eps):
    # 1) 静止：mag 接近 9.8 就直接判静止
    if abs(mag - 9.8) < still_eps:
        return "静止", 0.0
    # 2) 否则直接找离哪个模板最近
    best_name = None
    best_d2 = None
    for name, prof in profiles.items():
        mean = prof['mean']
        std  = prof['std']
        d2 = 0.0
        for i in range(4):
            z = (feature[i] - mean[i]) / std[i]
            d2 += z*z
        if (best_d2 is None) or (d2 < best_d2):
            best_d2 = d2
            best_name = name
    return best_name, best_d2

# =============== 小圆点地图（16×3） ===============
GRID_W = 16
GRID_H = 3
player_x = GRID_W // 2
player_y = GRID_H // 2
player_dir = 0  # 0上 1右 2下 3左

DIR_VECTORS = {
    0: (0, -1),
    1: (1, 0),
    2: (0, 1),
    3: (-1, 0),
}
DIR_TEXT = ["Up", "Right", "Down", "Left"]

def clamp_pos():
    global player_x, player_y
    if player_x < 0: player_x = 0
    if player_x >= GRID_W: player_x = GRID_W - 1
    if player_y < 0: player_y = 0
    if player_y >= GRID_H: player_y = GRID_H - 1

def move_forward(steps=1):
    global player_x, player_y
    dx, dy = DIR_VECTORS[player_dir]
    for _ in range(steps):
        player_x += dx
        player_y += dy
        clamp_pos()

def strafe(dx):
    global player_x
    player_x += dx
    clamp_pos()

def turn_left():
    global player_dir
    player_dir = (player_dir - 1) % 4

def turn_right():
    global player_dir
    player_dir = (player_dir + 1) % 4

def dodge_right():
    # 简单版“右转圈”：横向跳 2 格（避障）
    strafe(2)

def dodge_left():
    # 简单版“左转圈”：横向跳 -2 格（避障）
    strafe(-2)

def render_map(hand_label, leg_label):
    rows = []
    for _ in range(GRID_H):
        rows.append([" "] * GRID_W)
    # 角色
    rows[player_y][player_x] = "O"
    s0 = "".join(rows[0])
    s1 = "".join(rows[1])
    s2 = "".join(rows[2])
    line4 = f"{DIR_TEXT[player_dir]} H:{hand_label} L:{leg_label}"
    show4(s0, s1, s2, line4[:16])  # 防止太长

prev_hand_label = "静止"
prev_leg_label  = "静止"

show4("Move with", "your motions", "", "Ready...")
time.sleep(1.0)

while True:
    # 读特征
    px_h, py_h, pz_h, mag_h = compute_features_hand()
    px_l, py_l, pz_l, mag_l = compute_features_leg()
    hand_feature = [px_h, py_h, pz_h, mag_h]
    leg_feature  = [px_l, py_l, pz_l, mag_l]

    # 分类 —— 不要任何冷却、连续帧，只看当前帧
    hand_label, d2_h = classify(hand_feature, profiles_hand, mag_h, STILL_MAG_EPS_HAND)
    leg_label,  d2_l = classify(leg_feature,  profiles_leg,  mag_l, STILL_MAG_EPS_LEG)

    # 只在从「静止」切换到某动作时，执行一次移动
    if hand_label != prev_hand_label:
        if hand_label != "静止":
            if hand_label == "前挥手":
                move_forward(1)
            elif hand_label == "右挥手":
                strafe(+1)
            elif hand_label == "左挥手":
                strafe(-1)
            elif hand_label == "右转圈":
                dodge_right()
            elif hand_label == "左转圈":
                dodge_left()
            # 出击：先不移动，只文字显示
        prev_hand_label = hand_label

    if leg_label != prev_leg_label:
        if leg_label != "静止":
            if leg_label == "左摆腿":
                turn_left()
            elif leg_label == "右摆腿":
                turn_right()
            elif leg_label == "小跑":
                move_forward(2)
        prev_leg_label = leg_label

    render_map(hand_label or "静止", leg_label or "静止")
    time.sleep(0.05)
