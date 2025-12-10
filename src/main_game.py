import time, math
import board, busio, digitalio
import displayio, terminalio
from adafruit_display_text import label
import i2cdisplaybus
import adafruit_displayio_ssd1306
import adafruit_adxl34x
import level_maps
import neopixel
from rotary_encoder import RotaryEncoder  


print("STEP 0: 程序开始运行（完整版 + 小跑滑动窗口峰谷差 + 游戏 + 小球收集，无 global）")

# =======================================================
# ★ 是否使用预训练模板（不想训练就设 True）
# =======================================================
USE_DEFAULT_PROFILES = True   # ← 一键开关：True=用下面这组默认模板；False=每次自己训练

# =======================================================
# ★ 预训练模板数据（你贴过的 mean/std）
# =======================================================

DEFAULT_HAND_PROFILES = {
    "前挥手": {
        "mean": [-34.6491, -1.46076, -54.1404, 8.84603],
        "std":  [11.7971, 6.73255, 10.8319, 2.01564],
    },
    "右挥手": {
        "mean": [-21.6102, 63.8855, -7.93684, 9.76243],
        "std":  [8.27053, 10.9926, 12.6232, 0.905509],
    },
    "左挥手": {
        "mean": [-33.0268, -53.0387, -6.18296, 11.1966],
        "std":  [8.9192, 15.4748, 10.673, 1.15464],
    },
    "右转圈": {
        "mean": [-28.5648, -9.52275, 4.30972, 7.91655],
        "std":  [16.0947, 28.5154, 48.1189, 3.6548],
    },
    "左转圈": {
        "mean": [-30.5536, -9.15549, 17.0639, 8.04242],
        "std":  [26.3095, 16.1497, 51.209, 3.16192],
    },
    "出击": {
        "mean": [-4.75843, 0.909303, -54.6548, 8.20098],
        "std":  [33.2775, 16.1671, 20.4922, 2.49377],
    },
}

DEFAULT_RLEG_PROFILES = {
    "右摆腿": {
        "mean": [66.6615, 19.7218, -9.90336, 9.67785],
        "std":  [11.5365, 11.9366, 5.07672, 1.80696],
    },
    "小跑": {
        "mean": [70.9695, 7.34304, -16.4684, 9.87104],
        "std":  [5.08886, 6.98959, 2.87266, 0.997999],
    },
}

DEFAULT_LLEG_PROFILES = {
    "左摆腿": {
        "mean": [62.4097, -19.9176, -13.6525, 10.0313],
        "std":  [7.40038, 6.27516, 5.39766, 1.5792],
    },
    "小跑": {
        "mean": [74.6558, -3.17041, -14.567, 10.2409],
        "std":  [4.72588, 3.82086, 4.35937, 1.03341],
    },
}

# =========================================
# 参数（动作判定部分）
# =========================================
FRAME_DT = 0.05  # 采样周期（秒），20Hz

STILL_MAG_EPS_HAND = 0.6
STILL_MAG_EPS_LEG  = 0.6

HAND_NOISE_DIST_THRESH = 60.0
LEG_NOISE_DIST_THRESH  = 60.0

alpha = 0.2  # 低通滤波系数


# —— pitch 持续变化判定参数 —— 
PITCH_DEADBAND_DEG = 1.0  # 小于这个变化视为“没怎么动”

# —— 每个动作需要的“持续增长/减少时间”（秒）——
HAND_TREND_MIN_SEC = {
    "右转圈": 0.3,
    "左转圈": 0.3,
}
LEG_TREND_MIN_SEC = {
    "左摆腿": 0.15,
    "右摆腿": 0.15,
}

# —— 小跑检测（滑动窗口峰谷差）——
WINDOW_FRAMES    = 7
RUN_RANGE_THRESH = 0.6
RUN_MIN_SEC      = 0.5

# —— 每个动作训练时长（秒）——
CALIB_SEC_HAND = 6.0
CALIB_SEC_LEG  = 6.0

# 手 / 腿 动作列表（训练时会依次提示）
HAND_ACTIONS = ["前挥手", "右挥手", "左挥手", "右转圈", "左转圈", "出击"]
LEG_ACTIONS  = ["左摆腿", "右摆腿", "小跑"]

# —— 出击：需要当前 mag 相比上一帧有一个大跳变 —— 
ATTACK_MAG_JUMP = 3.0

# —— 摆腿冷却时间（秒）：触发一次后 1.5 秒内不再转向 —— 
LEG_TURN_COOLDOWN_SEC = 1.0

# —— 小球相关 —— 
BALL_COUNT      = 10
BALL_RADIUS     = 4   # 小球半径（世界坐标里用来判定碰撞）


# =========================================
#   工具函数
# =========================================
def lowpass(prev, now):
    return alpha * now + (1 - alpha) * prev

def compute_feature(x, y, z):
    """xyz → [pitchX, pitchY, pitchZ, mag]"""
    pitchx = math.degrees(math.atan2(-x, math.sqrt(y*y + z*z)))
    pitchy = math.degrees(math.atan2(-y, math.sqrt(x*x + z*z)))
    pitchz = math.degrees(math.atan2(-z, math.sqrt(x*x + y*y)))
    mag    = math.sqrt(x*x + y*y + z*z)
    return [pitchx, pitchy, pitchz, mag]

def classify_once(feature, profiles):
    """返回 (最近动作名, 该动作的 dist2)"""
    best_name = None
    best_dist2 = None

    for name, prof in profiles.items():
        mean = prof["mean"]
        std  = prof["std"]
        d2 = 0.0
        for i in range(4):
            z = (feature[i] - mean[i]) / std[i]
            d2 += z * z
        if best_dist2 is None or d2 < best_dist2:
            best_dist2 = d2
            best_name  = name

    return best_name, best_dist2

def final_decision(mag, best_name, dist2, still_eps, noise_thresh):
    """
    最简单规则：
      1) |mag-9.8| < still_eps → 静止
      2) dist2 > noise_thresh  → 静止
      3) 否则 → best_name
    """
    if abs(mag - 9.8) < still_eps:
        return "静止"
    if dist2 > noise_thresh:
        return "静止"
    return best_name

def update_pitch_trend(theta, state, dt):
    """
    检查某个 pitch 角度是否“持续增大或持续减小”
    state: {"last":.., "dir":.., "time":..}
    返回: (当前趋势已持续时间 trend_time, 更新后的 state)
    """
    last = state["last"]
    direction = state["dir"]
    t = state["time"]

    if last is None:
        state["last"] = theta
        state["dir"] = 0
        state["time"] = 0.0
        return 0.0, state

    delta = theta - last

    # 变化太小，当做没动，趋势清零
    if abs(delta) < PITCH_DEADBAND_DEG:
        state["last"] = theta
        state["dir"] = 0
        state["time"] = 0.0
        return 0.0, state

    step_dir = 1 if delta > 0 else -1

    if direction == 0 or step_dir == direction:
        # 和之前方向一致 → 累加时间
        t += dt
        direction = step_dir
    else:
        # 方向反了 → 重新开始计时
        direction = step_dir
        t = dt

    state["last"] = theta
    state["dir"] = direction
    state["time"] = t

    return t, state

def update_window(win, value, max_len):
    """把 value 放进滑动窗口，只保留最近 max_len 个"""
    win.append(value)
    if len(win) > max_len:
        win.pop(0)
    return win

def range_over_window(win):
    """
    返回窗口里的 max-min，如果帧数太少，返回 0
    """
    if len(win) < 2:
        return 0.0
    return max(win) - min(win)


# =========================================
#   第三颗 ADXL345：SPI 驱动类
# =========================================
class ADXL345_SPI:
    REG_DATA_FORMAT = 0x31
    REG_POWER_CTL   = 0x2D
    REG_DATAX0      = 0x32

    def __init__(self, spi, cs):
        self.spi = spi
        self.cs = cs

        print("STEP 3: 准备配置 SPI")
        while not self.spi.try_lock():
            pass
        try:
            self.spi.configure(baudrate=5_000_000, phase=1, polarity=1)
        finally:
            self.spi.unlock()

        self.cs.direction = digitalio.Direction.OUTPUT
        self.cs.value = False

        print("STEP 4: 写入初始化寄存器")
        self._write_register(self.REG_DATA_FORMAT, 0x08)  # ±2g, full-res
        self._write_register(self.REG_POWER_CTL,   0x08)  # Measure 模式
        print("STEP 5: SPI IMU 初始化完毕")

    def _write_register(self, reg, value):
        while not self.spi.try_lock():
            pass
        try:
            self.cs.value = False
            self.spi.write(bytes([reg, value]))
            self.cs.value = True
        finally:
            self.spi.unlock()

    def _read_registers(self, reg, length):
        """
        ADXL345 SPI 多字节读取：
        bit7 = 1 → 读
        bit6 = 1 → 多字节 auto-increment
        所以用 reg | 0xC0
        """
        while not self.spi.try_lock():
            pass
        try:
            self.cs.value = False
            self.spi.write(bytes([reg | 0xC0]))
            buf = bytearray(length)
            self.spi.readinto(buf)
            self.cs.value = True
        finally:
            self.spi.unlock()
        return buf

    @property
    def acceleration(self):
        data = self._read_registers(self.REG_DATAX0, 6)

        def to_signed(lo, hi):
            raw = (hi << 8) | lo
            if raw & 0x8000:
                raw -= 0x10000
            return raw

        x_raw = to_signed(data[0], data[1])
        y_raw = to_signed(data[2], data[3])
        z_raw = to_signed(data[4], data[5])

        scale = 0.004 * 9.80665  # 0.004 g/LSB
        return (x_raw * scale, y_raw * scale, z_raw * scale)


# =========================================
#   主类：动作判定 + 游戏（无 global）
# =========================================
class GameMotion:

    def __init__(self):
        # -------- I2C / OLED / IMUs 初始化 --------
        print("STEP 1: 创建 I2C 总线")
        self.i2c = board.I2C()

        # OLED
        displayio.release_displays()
        display_bus = i2cdisplaybus.I2CDisplayBus(self.i2c, device_address=0x3C)
        self.display = adafruit_displayio_ssd1306.SSD1306(
            display_bus,
            width=128,
            height=64
        )
        
    
        
        # 关掉自动刷新，避免闪屏
        self.display.auto_refresh = False

        self.SCREEN_W = 128
        self.SCREEN_H = 64
        

        # -------- 难度模式（默认 Easy） --------
        self.difficulty = "Easy"

        # 每个难度 / 每一关的倒计时（秒）——你可以随时改这里
        # 例子：Easy 每关 60 秒，Medium 45 秒，Hard 30 秒
        self.level_time_limits = {
            "Easy":   [360] * level_maps.LEVEL_COUNT,
            "Medium": [180] * level_maps.LEVEL_COUNT,
            "Hard":   [60] * level_maps.LEVEL_COUNT,
        }
        # 当前这一关的总时长 & 剩余时间（秒）
        self.level_time_limit = 0.0
        self.remaining_time   = 0.0

        
                # -------- 关卡信息 --------
        self.current_level = 1
        self.max_level = level_maps.LEVEL_COUNT


        # 大地图尺寸 = 屏幕 2 倍
        self.WORLD_W = self.SCREEN_W * 2   # 256
        self.WORLD_H = self.SCREEN_H * 2   # 128

        # 小地图参数
        self.MINIMAP_W = 32
        self.MINIMAP_H = 16
        self.MINIMAP_X = self.SCREEN_W - self.MINIMAP_W - 1
        self.MINIMAP_Y = 1

        self.bitmap = displayio.Bitmap(self.SCREEN_W, self.SCREEN_H, 2)
        self.palette = displayio.Palette(2)
        self.palette[0] = 0x000000
        self.palette[1] = 0xFFFFFF

        self.tilegrid = displayio.TileGrid(self.bitmap, pixel_shader=self.palette)
        self.root_group = displayio.Group()
        self.root_group.append(self.tilegrid)

        self.status_label = label.Label(
            terminalio.FONT,
            text="",
            x=0,
            y=self.SCREEN_H - 1,
            color=0xFFFFFF
        )
        self.root_group.append(self.status_label)
        self.display.root_group = self.root_group

        # 三颗 IMU
        print("STEP 2: 初始化 I2C 上两颗 IMU（右手 + 右腿）")
        self.acc_hand = adafruit_adxl34x.ADXL345(self.i2c, address=0x53)  # 右手
        self.acc_rleg = adafruit_adxl34x.ADXL345(self.i2c, address=0x1D)  # 右腿

        print("STEP 3: 初始化 左腿 IMU（SPI）")
        spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
        cs_left = digitalio.DigitalInOut(board.D7)
        self.acc_lleg = ADXL345_SPI(spi, cs_left)

        print("STEP 6: 三块 ADXL345 已全部初始化成功")
        
         # -------- 旋钮 + 按钮（选择难度用） --------
        # 旋钮：D10, D9  （和你之前的代码一致）
        self.encoder = RotaryEncoder(board.D2, board.D1,
                                     debounce_ms=3, pulses_per_detent=3)

        # 按钮：D8，上拉输入
        self.btn = digitalio.DigitalInOut(board.D0)
        self.btn.direction = digitalio.Direction.INPUT
        self.btn.pull = digitalio.Pull.UP
        # 记录上一次按钮状态，用来做“边沿检测”
        self._btn_last = self.btn.value  # True=没按，False=按下
        
        
        # -------- NeoPixel 灯条（5 颗，用作时间条） --------
        self.num_pixels = 5
        # DIN 接在 D3 脚
        self.pixels = neopixel.NeoPixel(
            board.D6,
            self.num_pixels,
            brightness=0.3,
            auto_write=False
        )
        # 开机先全灭
        for i in range(self.num_pixels):
            self.pixels[i] = (0, 0, 0)
        self.pixels.show()
        



        # -------- 角色（4×4 小方块） --------
        self.TILE = 4

        # 玩家在大地图里起始位置（不要贴墙）
        self.player_x = self.WORLD_W // 20
        self.player_y = self.WORLD_H // 10
        self.player_angle = -math.pi / 2  # 向上

        self.STEP_CLICK = 5  # 单次挥手移动像素
        self.STEP_RUN   = 2.7  # 小跑每帧移动像素

        # 半圆转向 + 箭头参数
        self.ARC_RADIUS = 6.0
        self.ARC_STEPS  = 7
        self.ARROW_LEN  = 6
        self.ARC_STEP_DELAY = 0.001

        # 地图 & 小球由关卡统一初始化
        self.walls = []
        self.balls = []
        self.total_balls = 0
        self.collected_balls = 0
        self.finished = False

        # ★ 根据关卡号加载：出生点 + 地图 + 小球
        self.load_level(self.current_level)


        # 滤波变量
        self.fx_h = self.fy_h = self.fz_h = 0.0  # 手
        self.fx_r = self.fy_r = self.fz_r = 0.0  # 右腿
        self.fx_l = self.fy_l = self.fz_l = 0.0  # 左腿

        # pitch 趋势状态
        self.trend_hand = {"last": None, "dir": 0, "time": 0.0}
        self.trend_rleg = {"last": None, "dir": 0, "time": 0.0}
        self.trend_lleg = {"last": None, "dir": 0, "time": 0.0}

        # 小跑窗口
        self.window_r = []
        self.window_l = []
        self.run_time = 0.0

        # 模板
        self.profiles_hand = {}
        self.profiles_rleg = {}
        self.profiles_lleg = {}

        # 出击：上一帧 mag
        self.prev_mag_hand = None

        # 摆腿冷却计时（秒）：>0 表示还在冷却
        self.leg_turn_cd = 0.0
        
        
    def start_level_timer(self):
        """
        根据当前 difficulty + current_level 设置这一关的倒计时。
        """
        limits = self.level_time_limits.get(self.difficulty,
                                            self.level_time_limits["Easy"])
        idx = self.current_level - 1
        if idx < 0:
            idx = 0
        if idx >= len(limits):
            idx = len(limits) - 1

        self.level_time_limit = float(limits[idx])
        self.remaining_time   = float(limits[idx])
        print("关卡", self.current_level, "难度", self.difficulty,
              "倒计时 =", self.level_time_limit, "秒")
        
        # 刚开始这一关：灯条按照“满时间”状态刷新（= 5 颗全绿）
        self.update_timer_pixels()

        
        
        
    # ========== NeoPixel 灯条相关函数 ==========
    def _set_all_pixels(self, color):
        """把 5 颗灯全部设成同一个颜色"""
        for i in range(self.num_pixels):
            self.pixels[i] = color
        self.pixels.show()

    def _clear_pixels(self):
        """全部熄灭"""
        self._set_all_pixels((0, 0, 0))

    def update_timer_pixels(self):
        """
        根据当前关卡剩余时间，更新 5 颗灯的状态：
        - 刚开始：5 颗全亮绿
        - 剩 4/5 时：灭掉第 5 颗，其余保持绿
        - 剩 3/5 时：第 5、4 颗灭，剩下的变成橙色
        - 剩 2/5 时：第 5、4、3 颗灭，1、2 颗橙色
        - 剩 1/5 时：只留第 1 颗，变红
        """
        if self.level_time_limit <= 0:
            return

        frac = self.remaining_time / self.level_time_limit
        if frac < 0:
            frac = 0.0
        if frac > 1:
            frac = 1.0

        GREEN  = (0, 255, 0)
        ORANGE = (255, 80, 0)
        RED    = (255, 0, 0)
        OFF    = (0, 0, 0)

        # 先全灭
        for i in range(self.num_pixels):
            self.pixels[i] = OFF

        # frac 是“剩余部分”
        if frac > 4.0 / 5.0:
            # 刚开始：5 颗全绿
            for i in range(self.num_pixels):
                self.pixels[i] = GREEN
        elif frac > 3.0 / 5.0:
            # 剩 4/5：灭掉第 5 颗
            for i in range(4):      # 0,1,2,3
                self.pixels[i] = GREEN
        elif frac > 2.0 / 5.0:
            # 剩 3/5：第 5、4 颗灭，1~3 橙色
            for i in range(3):      # 0,1,2
                self.pixels[i] = ORANGE
        elif frac > 1.0 / 5.0:
            # 剩 2/5：第 5、4、3 颗灭，1、2 橙色
            for i in range(2):      # 0,1
                self.pixels[i] = ORANGE
        elif frac > 0.0:
            # 剩 1/5：只留第 1 颗红色
            self.pixels[0] = RED
        else:
            # frac == 0：时间到，这里不处理，失败时 show_fail_screen 会统一全红
            pass

        self.pixels.show()


        
        # -------- 关卡加载 --------
    def load_level(self, level):
        """根据关卡号加载：玩家起点 + 地图墙体 + 小球"""
        # 限制关卡号在合法范围内
        if level < 1:
            level = 1
        if level > level_maps.LEVEL_COUNT:
            level = level_maps.LEVEL_COUNT
        self.current_level = level
        idx = level - 1

        # 玩家起点：从 level_maps.LEVEL_START 取
        if 0 <= idx < len(level_maps.LEVEL_START):
            start_x, start_y = level_maps.LEVEL_START[idx]
            self.player_x = start_x
            self.player_y = start_y

        # 朝向统一朝上
        self.player_angle = -math.pi / 2

        # 重建地图和小球
        self.build_world_map()
        self.build_balls()

        # 重置计数
        self.collected_balls = 0
        self.finished = False
        # 让开场画面居中到玩家
        self.clamp_player()
        
        # 为这一关设置倒计时
        self.start_level_timer()


    # ========== 地图数据 ==========
    def build_world_map(self):
        """根据 current_level，从 level_maps 取当前关卡的内墙，再加上统一外框"""
        self.walls = []
        t = 4
        W = self.WORLD_W
        H = self.WORLD_H

        # 外框（所有关卡通用）
        self.walls.append((0, 0, W - 1, t))
        self.walls.append((0, H - t, W - 1, H - 1))
        self.walls.append((0, 0, t, H - 1))
        self.walls.append((W - t, 0, W - 1, H - 1))

        # 当前关卡的内墙（在 level_maps.LEVEL_WALLS 中配置）
        idx = self.current_level - 1
        if 0 <= idx < len(level_maps.LEVEL_WALLS):
            for (x1, y1, x2, y2) in level_maps.LEVEL_WALLS[idx]:
                self.walls.append((x1, y1, x2, y2))


    # ========== 小球构建 & 工具 ==========
    def point_in_wall(self, x, y):
        """判断世界坐标 (x,y) 是否在某一堵墙内部"""
        for (x1, y1, x2, y2) in self.walls:
            if x1 <= x <= x2 and y1 <= y <= y2:
                return True
        return False

    def build_balls(self):
        """从 level_maps 读取当前关卡的小球坐标"""
        self.balls = []
        idx = self.current_level - 1
        if 0 <= idx < len(level_maps.LEVEL_BALLS):
            for (bx, by) in level_maps.LEVEL_BALLS[idx]:
                self.balls.append({"x": bx, "y": by, "alive": True})
        # 小球总数 = 当前关卡的小球个数
        self.total_balls = len(self.balls)


    # ========== OLED 绘制 ==========
    def clear_bitmap(self):
        self.bitmap.fill(0)

    def draw_pixel(self, x, y):
        if 0 <= x < self.SCREEN_W and 0 <= y < self.SCREEN_H:
            self.bitmap[x, y] = 1

    def draw_label(self, text):
        self.status_label.text = text[:22]
        
        
    # ========== 开始前：用旋钮选择难度 ==========

    def select_difficulty(self):
        options = ["Easy", "Medium", "Hard"]

        # 像老师示例一样，从 0 开始计数
        self.encoder.reset()
        idx = 0

        # 初始画面
        self.bitmap.fill(0)
        self.status_label.x = 0
        self.status_label.y = self.SCREEN_H // 2
        self.status_label.text = "Mode: " + options[idx]
        self.display.refresh()

        print("请用旋钮选择难度（Easy/Medium/Hard），按下按钮确认。")

        while True:
            # ① 完全照老师写法：先 update()，再看有没有 changed
            changed = self.encoder.update()
            if changed:
                # 老师示范里是 print("Position:", encoder.position)
                # 我们用这个 position 来算当前模式
                pos = self.encoder.position
                # 直接用 position 对 3 取模，在 0,1,2 之间循环
                idx = pos % len(options)

                # 更新屏幕上的文字
                self.bitmap.fill(0)
                self.status_label.text = "Mode: " + options[idx]
                self.display.refresh()

            # ② 按钮：检测“松手 -> 按下”的沿，确认选择
            now_btn = self.btn.value  # True = 松开, False = 按下
            if (self._btn_last is True) and (now_btn is False):
                # 边沿触发：刚刚被按下
                self.difficulty = options[idx]
                print("选择难度：", self.difficulty)
                time.sleep(0.3)  # 防抖
                return

            self._btn_last = now_btn

            # 和老师示例一样，给一点点延时
            time.sleep(0.001)


    def draw_hollow_circle(self, cx, cy, r, cam_x, cam_y):
        """在世界坐标 (cx,cy) 画一个空心小圆"""
        for deg in range(0, 360, 15):
            rad = math.radians(deg)
            wx = cx + int(r * math.cos(rad))
            wy = cy + int(r * math.sin(rad))
            sx = wx - cam_x
            sy = wy - cam_y
            self.draw_pixel(sx, sy)

    # ========== 摄像机 ==========
    def compute_camera(self):
        half_w = self.SCREEN_W // 2
        half_h = (self.SCREEN_H - 10) // 2

        cam_x = self.player_x - half_w
        cam_y = self.player_y - half_h

        if cam_x < 0:
            cam_x = 0
        if cam_y < 0:
            cam_y = 0

        max_cx = self.WORLD_W - self.SCREEN_W
        max_cy = self.WORLD_H - (self.SCREEN_H - 10)

        if cam_x > max_cx:
            cam_x = max_cx
        if cam_y > max_cy:
            cam_y = max_cy

        return cam_x, cam_y

    # ========== 碰撞检测 ==========
    def is_colliding(self, nx, ny, margin=1):
        px1 = nx - margin
        py1 = ny - margin
        px2 = nx + self.TILE - 1 + margin
        py2 = ny + self.TILE - 1 + margin

        for (x1, y1, x2, y2) in self.walls:
            if (px2 >= x1 and px1 <= x2 and
                py2 >= y1 and py1 <= y2):
                return True
        return False

    # ========== 角色移动 ==========
    def clamp_player(self):
        if self.player_x < 0:
            self.player_x = 0
        if self.player_x > self.WORLD_W - self.TILE:
            self.player_x = self.WORLD_W - self.TILE
        if self.player_y < 0:
            self.player_y = 0
        if self.player_y > self.WORLD_H - self.TILE:
            self.player_y = self.WORLD_H - self.TILE

    def move_forward(self, px):
        dx = math.cos(self.player_angle) * px
        dy = math.sin(self.player_angle) * px
        nx = self.player_x + int(round(dx))
        ny = self.player_y + int(round(dy))
        if not self.is_colliding(nx, ny):
            self.player_x = nx
            self.player_y = ny
            self.clamp_player()

    def move_strafe(self, side, px):
        angle_side = self.player_angle + side * math.pi / 2
        dx = math.cos(angle_side) * px
        dy = math.sin(angle_side) * px
        nx = self.player_x + int(round(dx))
        ny = self.player_y + int(round(dy))
        if not self.is_colliding(nx, ny):
            self.player_x = nx
            self.player_y = ny
            self.clamp_player()

    def turn_left_90(self):
        self.player_angle -= math.pi / 2

    def turn_right_90(self):
        self.player_angle += math.pi / 2

    def move_arc(self, dir_sign):
        steps = self.ARC_STEPS
        total_forward = 6.0
        step_fwd = total_forward / steps
        max_offset_angle = math.pi / 3

        for i in range(steps):
            if steps > 1:
                t = i / (steps - 1)
            else:
                t = 0.0

            offset = math.sin(math.pi * t) * max_offset_angle * dir_sign
            ang = self.player_angle + offset

            dx = math.cos(ang) * step_fwd
            dy = math.sin(ang) * step_fwd

            nx = self.player_x + int(round(dx))
            ny = self.player_y + int(round(dy))

            if self.is_colliding(nx, ny):
                break

            self.player_x = nx
            self.player_y = ny
            self.clamp_player()

            self.render()
            time.sleep(self.ARC_STEP_DELAY)

    # ========== 画大地图里的墙 ==========
    def draw_world(self, cam_x, cam_y):
        for (x1, y1, x2, y2) in self.walls:
            sx1 = x1 - cam_x
            sy1 = y1 - cam_y
            sx2 = x2 - cam_x
            sy2 = y2 - cam_y

            if sx2 < 0 or sy2 < 0 or sx1 >= self.SCREEN_W or sy1 >= self.SCREEN_H - 10:
                continue

            draw_x1 = max(0, sx1)
            draw_y1 = max(0, sy1)
            draw_x2 = min(self.SCREEN_W - 1, sx2)
            draw_y2 = min(self.SCREEN_H - 11, sy2)

            for yy in range(draw_y1, draw_y2 + 1):
                for xx in range(draw_x1, draw_x2 + 1):
                    self.draw_pixel(xx, yy)

    # ========== 小地图 ==========
    def draw_minimap(self):
        sx = self.MINIMAP_X
        sy = self.MINIMAP_Y

        for x in range(self.MINIMAP_W):
            self.draw_pixel(sx + x, sy)
            self.draw_pixel(sx + x, sy + self.MINIMAP_H - 1)
        for y in range(self.MINIMAP_H):
            self.draw_pixel(sx, sy + y)
            self.draw_pixel(sx + self.MINIMAP_W - 1, sy + y)

        scale_x = (self.MINIMAP_W - 2) / self.WORLD_W
        scale_y = (self.MINIMAP_H - 2) / self.WORLD_H

        for (x1, y1, x2, y2) in self.walls:
            mx1 = sx + 1 + int(x1 * scale_x)
            my1 = sy + 1 + int(y1 * scale_y)
            mx2 = sx + 1 + int(x2 * scale_x)
            my2 = sy + 1 + int(y2 * scale_y)

            if mx1 > mx2:
                mx1, mx2 = mx2, mx1
            if my1 > my2:
                my1, my2 = my2, my1

            for yy in range(my1, my2 + 1):
                for xx in range(mx1, mx2 + 1):
                    self.draw_pixel(xx, yy)

        px = sx + 1 + int(self.player_x * scale_x)
        py = sy + 1 + int(self.player_y * scale_y)
        self.draw_pixel(px, py)
        self.draw_pixel(px + 1, py)
        self.draw_pixel(px - 1, py)
        self.draw_pixel(px, py + 1)
        self.draw_pixel(px, py - 1)

    # ========== 渲染 ==========
    def render(self):
        self.clear_bitmap()

        cam_x, cam_y = self.compute_camera()
        self.draw_world(cam_x, cam_y)

        # 画活着的小球（空心）
        for ball in self.balls:
            if ball["alive"]:
                self.draw_hollow_circle(ball["x"], ball["y"], BALL_RADIUS, cam_x, cam_y)

        # 画玩家
        for dy in range(self.TILE):
            for dx in range(self.TILE):
                wx = self.player_x + dx
                wy = self.player_y + dy
                sx = wx - cam_x
                sy = wy - cam_y
                self.draw_pixel(sx, sy)

        # 画箭头（当前 ARROW_LEN）
        cx_w = self.player_x + self.TILE // 2
        cy_w = self.player_y + self.TILE // 2
        for k in range(1, self.ARROW_LEN + 1):
            wx = cx_w + int(round(math.cos(self.player_angle) * k))
            wy = cy_w + int(round(math.sin(self.player_angle) * k))
            sx = wx - cam_x
            sy = wy - cam_y
            self.draw_pixel(sx, sy)

        self.draw_minimap()

        # 左下角显示：剩余时间 + 小球进度
        # 例如：T:35s  3/10
        sec_left = int(self.remaining_time + 0.99)  # 向上取整
        if sec_left < 0:
            sec_left = 0
        label_text = "%2ds %d/%d" % (
            sec_left,
            self.collected_balls,
            self.total_balls,
        )
        self.draw_label(label_text)
        
        # 游戏状态文字位置（左下角）
        self.status_label.x = 0
        self.status_label.y = 60


        self.display.refresh()


    def flash_attack_arrow(self):
        if self.finished:
            return

        old_len = self.ARROW_LEN
        # 暂时把箭头拉长一点，方便碰到小球
        self.ARROW_LEN = old_len + 10

        # 先检测有没有打到小球，再闪一下
        self.check_arrow_hit()

        self.render()
        time.sleep(0.3)

        self.ARROW_LEN = old_len
        self.render()

    def check_arrow_hit(self):
        """用当前 ARROW_LEN 沿着玩家朝向扫一条线，碰到小球就让小球消失并 +1"""
        cx_w = self.player_x + self.TILE // 2
        cy_w = self.player_y + self.TILE // 2

        for ball in self.balls:
            if not ball["alive"]:
                continue
            bx = ball["x"]
            by = ball["y"]
            hit = False
            for k in range(1, self.ARROW_LEN + 1):
                wx = cx_w + int(round(math.cos(self.player_angle) * k))
                wy = cy_w + int(round(math.sin(self.player_angle) * k))
                dx = wx - bx
                dy = wy - by
                if dx * dx + dy * dy <= BALL_RADIUS * BALL_RADIUS:
                    hit = True
                    break
            if hit:
                ball["alive"] = False
                self.collected_balls += 1

        # 全部收集完 → 标记 finished
        if self.collected_balls >= self.total_balls:
            self.finished = True

    def show_finish_screen(self):
        """当前关卡全部小球收集完后：画面清空，屏幕中心写英文"""
        self.bitmap.fill(0)
        # 显示 “Level X complete”
        self.status_label.text = "Level %d complete" % self.current_level
        self.status_label.x = 10
        self.status_label.y = self.SCREEN_H // 2
        self.display.refresh()

        # ★ 当前关成功：5 颗灯全部亮绿
        self._set_all_pixels((0, 255, 0))

        
    def handle_level_complete(self):
        """
        处理通关：显示当前关卡完成 → 切到下一关 / 全部结束
        返回 True 表示“所有关卡都结束”，False 表示“只是切换到下一关”
        """
        # 先播一遍“Level X complete”
        self.show_finish_screen()
        time.sleep(1.5)  # 给玩家一点时间看

        # 如果已经是最后一关
        if self.current_level >= self.max_level:
            # 恭喜全通关
            self.show_all_complete_screen()
            return True  # 通知外面的循环：本轮游戏彻底结束了
        else:
            # 还有下一关：关卡号 +1
            next_level = self.current_level + 1
            # 调用你之前写好的 load_level()（里面会顺便重置倒计时）
            self.load_level(next_level)
            # 重新渲染一下画面（新地图 + 出生点 + 倒计时）
            self.render()
            return False
        
    def show_fail_screen(self):
        """时间耗尽但没收完小球：显示失败，然后停一下"""
        self.bitmap.fill(0)
        # 这里可以写中文也可以写英文
        self.status_label.text = "fail"
        # 调整一下位置（你可以自己改）
        self.status_label.x = 10
        self.status_label.y = self.SCREEN_H // 2
        self.display.refresh()
        # ★ 本关失败：5 颗灯全部亮红
        self._set_all_pixels((255, 0, 0))

        time.sleep(2.0)  # 看两秒





    # ========== 读特征 ==========
    def _read_filtered(self, acc, px, py, pz):
        x, y, z = acc.acceleration
        fx = lowpass(px, x)
        fy = lowpass(py, y)
        fz = lowpass(pz, z)
        return fx, fy, fz

    def read_feature_hand(self):
        self.fx_h, self.fy_h, self.fz_h = self._read_filtered(
            self.acc_hand, self.fx_h, self.fy_h, self.fz_h
        )
        return compute_feature(self.fx_h, self.fy_h, self.fz_h)

    def read_feature_rleg(self):
        self.fx_r, self.fy_r, self.fz_r = self._read_filtered(
            self.acc_rleg, self.fx_r, self.fy_r, self.fz_r
        )
        return compute_feature(self.fx_r, self.fy_r, self.fz_r)

    def read_feature_lleg(self):
        self.fx_l, self.fy_l, self.fz_l = self._read_filtered(
            self.acc_lleg, self.fx_l, self.fy_l, self.fz_l
        )
        return compute_feature(self.fx_l, self.fy_l, self.fz_l)

    # ========== 训练 ==========
    def collect_mean_std(self, read_feature_fn, duration_sec):
        sums    = [0.0, 0.0, 0.0, 0.0]
        sums_sq = [0.0, 0.0, 0.0, 0.0]
        count   = 0

        t0 = time.monotonic()
        while True:
            now = time.monotonic()
            if now - t0 >= duration_sec:
                break

            feat = read_feature_fn()
            for i in range(4):
                val = feat[i]
                sums[i]    += val
                sums_sq[i] += val * val
            count += 1

            time.sleep(FRAME_DT)

        if count == 0:
            return [0.0, 0.0, 0.0, 9.8], [1.0, 1.0, 1.0, 1.0]

        mean = [sums[i] / count for i in range(4)]
        std  = []
        for i in range(4):
            ex2 = sums_sq[i] / count
            var = ex2 - mean[i]*mean[i]
            if var < 1e-6:
                var = 1e-6
            std.append(math.sqrt(var))

        return mean, std

    def calibrate_hand_profiles(self):
        self.profiles_hand = {}
        print("\n===== 开始上肢（右手）动作训练 =====")
        for name in HAND_ACTIONS:
            print(f"\n>>> 现在准备做【{name}】{CALIB_SEC_HAND} 秒")
            print("   请把右手摆到起始姿势，3 秒后开始采集...")
            time.sleep(3.0)
            mean, std = self.collect_mean_std(self.read_feature_hand, CALIB_SEC_HAND)
            self.profiles_hand[name] = {"mean": mean, "std": std}
            print(f"【{name}】训练完成，mean = {mean}, std = {std}")
        print("===== 上肢训练完成 =====\n")

    def calibrate_leg_profiles(self):
        self.profiles_rleg = {}
        self.profiles_lleg = {}

        print("\n===== 开始下肢动作训练（右腿 → 左腿） =====")

        right_leg_actions = ["右摆腿", "小跑"]
        print("\n>>> [右腿] 动作训练")
        for name in right_leg_actions:
            print(f"\n>>> 现在准备用【右腿】做【{name}】{CALIB_SEC_LEG} 秒")
            print("   请用右腿做这个动作，3 秒后开始采集...")
            time.sleep(3.0)
            mean, std = self.collect_mean_std(self.read_feature_rleg, CALIB_SEC_LEG)
            self.profiles_rleg[name] = {"mean": mean, "std": std}
            print(f"[右腿][{name}] 训练完成，mean = {mean}, std = {std}")

        left_leg_actions = ["左摆腿", "小跑"]
        print("\n>>> [左腿] 动作训练")
        for name in left_leg_actions:
            print(f"\n>>> 现在准备用【左腿】做【{name}】{CALIB_SEC_LEG} 秒")
            print("   请用左腿做这个动作，3 秒后开始采集...")
            time.sleep(3.0)
            mean, std = self.collect_mean_std(self.read_feature_lleg, CALIB_SEC_LEG)
            self.profiles_lleg[name] = {"mean": mean, "std": std}
            print(f"[左腿][{name}] 训练完成，mean = {mean}, std = {std}")

        print("===== 下肢训练完成 =====\n")

    # ========== 主循环 ==========
    def run(self):
        """
        外层 while True：初始界面 + 选难度 + 配置模板
        内层 while True：真正玩游戏（可以跨多关），
                         直到：时间耗尽失败 或 10 关全部通关，
                         然后回到外层再次出现初始界面。
        """
        while True:
            # ---------- 初始界面：选择难度 ----------
            self.draw_label("Select mode...")
            self.display.refresh()
            self.select_difficulty()          # 在这里卡住，直到你按钮确认
            print("当前难度 =", self.difficulty)

            # 每次回到初始界面，相当于从第 1 关重新开始
            self.current_level = 1
            self.load_level(self.current_level)   # 里面会重置小球 & 倒计时 & finished
            self.finished = False

            # ---------- 模板准备 ----------
            if USE_DEFAULT_PROFILES:
                print("使用默认模板，不需要训练 👍")
                self.profiles_hand = DEFAULT_HAND_PROFILES
                self.profiles_rleg = DEFAULT_RLEG_PROFILES
                self.profiles_lleg = DEFAULT_LLEG_PROFILES
            else:
                print("进入训练模式...")
                self.draw_label("Train hand...")
                self.render()
                self.calibrate_hand_profiles()

                self.draw_label("Train leg...")
                self.render()
                self.calibrate_leg_profiles()

            print("所有动作模板准备完毕，进入实时检测 + 游戏阶段...\n")
            self.render()
            time.sleep(0.5)

            # ---------- 本轮游戏循环（可能跨多个 level） ----------
            frame_id = 0
            prev_upper = "静止"
            prev_lower = "静止"

            SCREEN_INTERVAL = 8
            PRINT_INTERVAL  = 20
            DEBUG_PRINT = False

            while True:
                # 1) 倒计时：只在本关还没结束时减少
                if not self.finished:
                    self.remaining_time -= FRAME_DT
                    
                    
                if not self.finished:
                    self.remaining_time -= FRAME_DT

                    # ★ 每帧根据剩余时间更新灯条 ★
                    self.update_timer_pixels()
                    
                    
                    if self.remaining_time <= 0:
                        # 时间到了且还没收完小球 → 失败，退出到初始界面
                        print("时间耗尽，本关失败")
                        self.show_fail_screen()
                        break  # 跳出“本轮游戏”，回到最外层 while True

                # 2) 检查是否已经通关当前关卡
                if self.finished:
                    all_clear = self.handle_level_complete()
                    if all_clear:
                        # 10 关全部通关 → 恭喜画面已经显示，回到初始界面
                        break  # 跳出“本轮游戏”，回到最外层 while True
                    # 如果还有下一关，则 load_level 已经切换好，继续下一帧
                    continue

                # 3) 正常一帧：动作检测 + 移动 + 刷屏
                frame_id += 1

                # 冷却计时：每帧减少摆腿冷却时间
                if self.leg_turn_cd > 0.0:
                    self.leg_turn_cd -= FRAME_DT
                    if self.leg_turn_cd < 0.0:
                        self.leg_turn_cd = 0.0

                # ---------- 上肢：右手 ----------
                f_hand = self.read_feature_hand()
                mag_hand = f_hand[3]

                raw_h, d2_h = classify_once(f_hand, self.profiles_hand)
                upper_action = final_decision(
                    mag_hand, raw_h, d2_h,
                    STILL_MAG_EPS_HAND, HAND_NOISE_DIST_THRESH
                )

                theta_hand = f_hand[2]
                trend_time_hand, self.trend_hand = update_pitch_trend(
                    theta_hand, self.trend_hand, FRAME_DT
                )

                req_hand = HAND_TREND_MIN_SEC.get(upper_action, 0.0)
                if req_hand > 0.0 and trend_time_hand < req_hand:
                    upper_action = "静止"

                if upper_action == "出击":
                    if (self.prev_mag_hand is None) or \
                       (abs(mag_hand - self.prev_mag_hand) < ATTACK_MAG_JUMP):
                        upper_action = "静止"
                self.prev_mag_hand = mag_hand

                # ---------- 下肢：右腿 ----------
                f_rleg = self.read_feature_rleg()
                raw_r, d2_r = classify_once(f_rleg, self.profiles_rleg)
                right_leg_action = final_decision(
                    f_rleg[3], raw_r, d2_r,
                    STILL_MAG_EPS_LEG, LEG_NOISE_DIST_THRESH
                )

                theta_rleg = f_rleg[1]
                trend_time_r, self.trend_rleg = update_pitch_trend(
                    theta_rleg, self.trend_rleg, FRAME_DT
                )
                req_rleg = LEG_TREND_MIN_SEC.get(right_leg_action, 0.0)
                if req_rleg > 0.0 and trend_time_r < req_rleg:
                    right_leg_action = "静止"

                # ---------- 下肢：左腿 ----------
                f_lleg = self.read_feature_lleg()
                raw_l, d2_l = classify_once(f_lleg, self.profiles_lleg)
                left_leg_action = final_decision(
                    f_lleg[3], raw_l, d2_l,
                    STILL_MAG_EPS_LEG, LEG_NOISE_DIST_THRESH
                )

                theta_lleg = f_lleg[1]
                trend_time_l, self.trend_lleg = update_pitch_trend(
                    theta_lleg, self.trend_lleg, FRAME_DT
                )
                req_lleg = LEG_TREND_MIN_SEC.get(left_leg_action, 0.0)
                if req_lleg > 0.0 and trend_time_l < req_lleg:
                    left_leg_action = "静止"

                # ---------- 小跑：滑动窗口峰谷差 ----------
                mag_r = f_rleg[3]
                mag_l = f_lleg[3]

                self.window_r = update_window(self.window_r, mag_r, WINDOW_FRAMES)
                self.window_l = update_window(self.window_l, mag_l, WINDOW_FRAMES)

                range_r = range_over_window(self.window_r)
                range_l = range_over_window(self.window_l)

                jitter_now = (range_r > RUN_RANGE_THRESH) or (range_l > RUN_RANGE_THRESH)

                if jitter_now:
                    self.run_time += FRAME_DT
                else:
                    self.run_time = 0.0

                running = (self.run_time >= RUN_MIN_SEC)
                
                if running and right_leg_action == "静止":
                    running = False
                
                

                if right_leg_action == "小跑":
                    right_leg_action = "静止"
                if left_leg_action == "小跑":
                    left_leg_action = "静止"


                if running:
                    lower_action = "小跑"
                else:
                    a_r = right_leg_action
                    a_l = left_leg_action

                    if a_l == "静止" and a_r == "静止":
                        lower_action = "静止"
                    elif a_l != "静止" and a_r == "静止":
                        lower_action = a_l
                    elif a_l == "静止" and a_r != "静止":
                        lower_action = a_r
                    else:
                        # 用 pitch 判断哪条腿抬得更明显
                        if abs(theta_lleg) >= abs(theta_rleg):
                            lower_action = a_l
                        else:
                            lower_action = a_r

                # ---------- 游戏动作 ----------
                if upper_action == "前挥手":
                    self.move_forward(self.STEP_CLICK)
                elif upper_action == "右挥手":
                    self.move_strafe(+1, self.STEP_CLICK)
                elif upper_action == "左挥手":
                    self.move_strafe(-1, self.STEP_CLICK)
                elif upper_action == "右转圈":
                    self.move_arc(+1)
                elif upper_action == "左转圈":
                    self.move_arc(-1)
                elif upper_action == "出击":
                    self.flash_attack_arrow()

                if lower_action == "小跑":
                    self.move_forward(self.STEP_RUN)
                elif lower_action in ("左摆腿", "右摆腿"):
                    # ★ 摆腿有冷却：只有在冷却为 0 时才真正转向
                    if self.leg_turn_cd <= 0.0:
                        if lower_action == "左摆腿":
                            self.turn_left_90()
                        else:
                            self.turn_right_90()
                        # 触发一次后，开始冷却
                        self.leg_turn_cd = LEG_TURN_COOLDOWN_SEC

                # ---------- 屏幕刷新 ----------
                if (frame_id % SCREEN_INTERVAL == 0) or \
                   (upper_action != prev_upper) or \
                   (lower_action != prev_lower):
                    self.render()
                    prev_upper = upper_action
                    prev_lower = lower_action

                if DEBUG_PRINT and (frame_id % PRINT_INTERVAL == 0):
                    print("上肢：", upper_action,
                          " | 下肢：", lower_action,
                          " | run_time={:.2f}s, range_r={:.2f}, range_l={:.2f}".format(
                              self.run_time, range_r, range_l
                          ))
                    print("-----")

                time.sleep(FRAME_DT)


# =========================================
#   入口
# =========================================
game = GameMotion()
game.run()
    

