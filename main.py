# =====================================================================
#  XE TỰ HÀNH NÉ VẬT CẢN - ESP32-S3 + MicroPython
#  Phần cứng: HC-SR04 + servo G90 (radar quét), L298N, 4 động cơ DC
#  Điều khiển: WiFi (trang web có radar) và Bluetooth BLE (UART)
#  Chế độ:     MAN (thủ công) / AUTO (tự động né vật cản)
# =====================================================================
import time
from time import ticks_ms, ticks_diff, ticks_add
from machine import Pin, PWM, time_pulse_us

# ============================ CẤU HÌNH ===============================
# --- Chân nối (theo sơ đồ của bạn) ---
ENA, IN1, IN2, IN3, IN4, ENB = 4, 5, 6, 7, 15, 16   # L298N
SERVO_PIN = 17                                      # dây cam servo G90
TRIG_PIN = 18                                       # HC-SR04 Trig
ECHO_PIN = 8                                        # HC-SR04 Echo (qua cầu chia áp!)

# --- Động cơ ---
MOTOR_PWM_FREQ = 1000
MIN_DUTY = 35          # % xung tối thiểu để bánh bắt đầu quay (tăng nếu xe ì)
LEFT_INVERT = False    # đổi True nếu bên trái quay ngược chiều khi tiến
RIGHT_INVERT = False   # đổi True nếu bên phải quay ngược chiều khi tiến
TURN_RATIO = 0.35      # tỉ lệ tốc độ bánh trong khi rẽ cong (nút chéo)
BRAKE_MS = 35          # ms xung hãm ngược chiều khi dừng để triệt tiêu quán tính trượt

# --- Servo / radar ---
# Quy ước góc của servo: 0 độ = THẲNG phía trước, dương = bên TRÁI, âm = bên PHẢI (tối đa +-80 độ).
SERVO_MIN_US = 500     # độ rộng xung ở 0 độ cơ khí của servo
SERVO_MAX_US = 2400    # độ rộng xung ở 180 độ cơ khí của servo
SERVO_TRIM_DEG = 0     # chỉnh cho đúng hướng thẳng: lúc khởi động đầu cảm biến lệch TRÁI -> đặt số ÂM
                       # (ví dụ -5), lệch PHẢI -> đặt số DƯƠNG. Đổi từng 2-3 độ rồi thử lại.
SERVO_INVERT = False   # đổi True nếu radar hiển thị bị lật trái/phải
SERVO_BASE_MS = 30     # thời gian chờ servo ổn định (cố định)
SERVO_MS_PER_DEG = 2   # thời gian chờ thêm theo mỗi độ quay
SCAN_RANGE = 80        # độ: quét tối đa sang mỗi bên
SCAN_STEP = 10         # độ: mỗi lần quay servo
SCAN_DWELL_MS = 60     # ms servo đứng yên ở mỗi góc trước khi đo (chờ servo ổn định)
FRONT_HALF_ANGLE = 15  # vùng "phía trước" = 0 +- giá trị này (độ)
MAX_RANGE = 200        # cm, xa hơn coi như thông thoáng
HOLD_INTERVAL_MS = 60  # ms giữa 2 lần đo khi servo đứng yên nhìn thẳng (chế độ thủ công)

# --- Tự động ---
AUTO_MAX_SPEED = 30    # % tốc độ tối đa cho phép ở chế độ tự động
AUTO_MIN_SPEED = 18    # % tốc độ thấp nhất khi giảm tốc gần vật cản
SLOW_DIST = 90         # cm: gần hơn mức này thì bắt đầu giảm tốc
TURN_SPEED = 70        # % tốc độ khi quay tại chỗ
CLEAR_DIST = 150       # cm: hướng có khoảng trống >= 150 cm (1,5 m) được coi là thông thoáng
OPEN_MOVE_CM = 150     # cm: khi thông thoáng cũng chỉ di chuyển tối đa 1,5 m (150 cm) mỗi lần
SAFE_DIST = 25         # cm: khoảng cách đệm an toàn tối thiểu cách vật cản sau mỗi bước tiến
MIN_STEP_CM = 5        # cm: bước di chuyển ngắn nhất khi đã ở gần vật cản
MIN_MOVE_FRAC = 0.35   # tỉ lệ bước đi khi ở gần vật cản (càng gần đi càng ngắn)
MAX_MOVE_FRAC = 0.65   # tỉ lệ bước đi khi ở xa vật cản
AUTO_EMERGENCY_CM = 20 # cm: đang tiến mà thấy vật cản <= mức này thì phanh dừng ngay lập tức
NOPATH_ROTATIONS = 4   # số lần quét liên tiếp thất bại (mỗi lần cách nhau 90 độ CW, 4 lần = đủ 1 vòng) -> báo không có đường
STEP_MS_PER_CM = 45    # HÃY HIỆU CHỈNH: ms xe chạy được 1 cm ở tốc độ AUTO_MAX_SPEED
UNKNOWN_TRUST_DIST = 130  # cm: mất tiếng vọng mà góc bên cạnh thấy vật gần hơn mức này thì KHÔNG coi là thoáng
AUTO_SETTLE_MS = 150   # ms đứng yên sau khi dừng/quay để xe hết rung rồi mới đo
AUTO_SAMPLES = 3       # số lần đo ở mỗi góc khi quét (lấy trung vị để loại nhiễu)
TURN_MS_PER_DEG = 5    # ms để quay 1 độ (HÃY HIỆU CHỈNH theo xe của bạn)

# --- Thủ công ---
MANUAL_TIMEOUT_MS = 400  # mất lệnh từ web quá thời gian này thì xe tự dừng
GUARD_DIST = 20          # cm: tự phanh khi đi tiến mà vật cản gần hơn

# --- Mạng ---
STA_SSID = ""            # điền WiFi nhà để xe vào mạng đó; để trống = tự phát WiFi
STA_PASS = ""
AP_SSID = "ESP32S3-CAR"
AP_PASS = "12345678"     # tối thiểu 8 ký tự
BLE_NAME = "ESP32S3-CAR"
BLE_SEND_RADAR = True    # gửi từng điểm radar qua BLE dạng "P,góc,cm"

# ===================== SERVO: ĐƯA VỀ THẲNG NGAY ======================
# Tạo PWM ngay với xung "thẳng" (0 độ). Trước đây PWM được tạo mà chưa có xung đúng, nên lúc cấp điện
# servo bị giật lệch một chút rồi mới về vị trí.
class Servo:
    def __init__(self, pin):
        self.pwm = PWM(Pin(pin), freq=50, duty_u16=self._duty(0))

    @staticmethod
    def _duty(rel):
        if SERVO_INVERT:
            rel = -rel
        a = max(0, min(180, 90 + SERVO_TRIM_DEG + rel))
        us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * a / 180
        return int(us * 65535 / 20000)

    def write(self, rel):
        """rel: 0 = thẳng, dương = trái, âm = phải (độ)."""
        self.pwm.duty_u16(self._duty(max(-90, min(90, rel))))


servo = Servo(SERVO_PIN)

import json
import hashlib
import binascii
import network
import bluetooth
import uasyncio as asyncio

# ======================== TRẠNG THÁI CHUNG ===========================
S = {
    "mode": "MAN",     # MAN | AUTO
    "speed": 60,       # tốc độ đặt tay (chế độ thủ công)
    "aspeed": AUTO_MAX_SPEED,  # tốc độ tối đa của chế độ tự động
    "cur": 0,          # tốc độ đang chạy thực tế (%)
    "cmd": "S",        # lệnh thủ công hiện tại
    "last": 0,         # thời điểm nhận lệnh gần nhất
    "timeout": True,   # True: lệnh từ web (cần gửi liên tục), False: từ BLE
    "guard": True,     # tự phanh khi thủ công
    "phase": "MAN",    # MAN BLOCK WAIT DRIVE SCAN LOOK TURN
    "angle": 0,        # góc servo hiện tại (0 = thẳng, dương = trái)
    "radar": {},       # góc -> (khoảng cách cm, thời điểm đo); -1 = thông thoáng
    "scanning": False, # thủ công: đang quét trái/phải (chưa được bấm quét tiếp)
    "scanreq": None,   # thủ công: yêu cầu quét đang chờ ("L" hoặc "R")
    "msg": "",         # thông báo gửi về giao diện (VD: NOPATH = không tìm thấy hướng đi)
    "msg_t": 0,
    "autoslow": True,  # AUTO: True = tự giảm tốc khi gần vật cản, False = chạy đều tốc độ đặt
}
ble = None


# ============================ PHẦN CỨNG ==============================
class Motor:
    def __init__(self, en, a, b, invert):
        self.en = PWM(Pin(en), freq=MOTOR_PWM_FREQ, duty_u16=0)
        self.a = Pin(a, Pin.OUT, value=0)
        self.b = Pin(b, Pin.OUT, value=0)
        self.invert = invert

    def drive(self, speed):  # -100..100
        speed = max(-100, min(100, int(speed)))
        if self.invert:
            speed = -speed
        if speed == 0:
            self.a.value(0)
            self.b.value(0)
            self.en.duty_u16(0)
            return
        fwd = speed > 0
        self.a.value(1 if fwd else 0)
        self.b.value(0 if fwd else 1)
        pct = MIN_DUTY + (100 - MIN_DUTY) * abs(speed) / 100
        self.en.duty_u16(int(pct * 65535 / 100))

    def hold_brake(self):
        """Ghìm điện từ hai đầu mô-tơ (L298N: IN1=1, IN2=1, EN=MAX) để chống trôi."""
        self.a.value(1)
        self.b.value(1)
        self.en.duty_u16(65535)


class Car:
    def __init__(self):
        self.left = Motor(ENA, IN1, IN2, LEFT_INVERT)    # OUT1, OUT2
        self.right = Motor(ENB, IN3, IN4, RIGHT_INVERT)  # OUT3, OUT4
        self.l = 0
        self.r = 0

    def set(self, l, r):
        l, r = int(l), int(r)
        if l == self.l and r == self.r:
            return
        self.l, self.r = l, r
        self.left.drive(l)
        self.right.drive(r)

    def stop(self):
        self.set(0, 0)

    async def brake(self):
        """Phanh chủ động chống quán tính trượt khi đang tiến/quay."""
        prev_l, prev_r = self.l, self.r
        if prev_l > 0 or prev_r > 0:
            rev_l = -35 if prev_l > 0 else 0
            rev_r = -35 if prev_r > 0 else 0
            self.left.drive(rev_l)
            self.right.drive(rev_r)
            await asyncio.sleep_ms(BRAKE_MS)
        self.left.hold_brake()
        self.right.hold_brake()
        await asyncio.sleep_ms(25)
        self.l = self.r = 0
        self.left.drive(0)
        self.right.drive(0)


class Sonar:
    def __init__(self, trig, echo):
        self.trig = Pin(trig, Pin.OUT, value=0)
        self.echo = Pin(echo, Pin.IN)
        self.timeout_us = int(MAX_RANGE * 2 / 0.0343) + 500

    def _once(self):
        self.trig.value(0)
        time.sleep_us(2)
        self.trig.value(1)
        time.sleep_us(10)
        self.trig.value(0)
        t = time_pulse_us(self.echo, 1, self.timeout_us)
        if t < 0:
            return -1
        d = t * 0.0343 / 2
        if d > MAX_RANGE:
            return -1
        return max(2, round(d))

    def read(self, retry=True):
        """Trả về cm, hoặc -1 nếu không có vật trong tầm.
        retry=False: đo 1 lần (chặn tối đa ~12 ms thay vì ~24 ms) để xe phản hồi lệnh nhanh hơn."""
        d = self._once()
        if d < 0 and retry:      # thử lại 1 lần để loại nhiễu mất tiếng vọng
            d = self._once()
        return d


car = Car()
sonar = Sonar(TRIG_PIN, ECHO_PIN)


# ============================ RADAR ==================================
def front_dist(max_age=1200):
    """Khoảng cách nhỏ nhất ở vùng phía trước (0 độ +- FRONT_HALF_ANGLE).
    Trả về MAX_RANGE+1 nếu thông thoáng, None nếu chưa có dữ liệu mới."""
    now = ticks_ms()
    best = None
    for a, (d, t) in S["radar"].items():
        if abs(a) <= FRONT_HALF_ANGLE and ticks_diff(now, t) < max_age:
            v = d if d > 0 else MAX_RANGE + 1
            best = v if best is None else min(best, v)
    return best


_servo_at = 0           # góc servo hiện tại
_last_sent = None
_servo_lock = asyncio.Lock()   # chỉ 1 task được dùng servo + cảm biến tại một thời điểm


async def _visit(a, retry, n=1):
    """Quay servo tới đúng góc a (0 = thẳng), đứng yên chờ ổn định rồi đo.
    n > 1: đo n lần, lấy trung vị (lọc nhiễu). Trả về cm, -1 = không có vật trong tầm."""
    global _servo_at, _last_sent
    async with _servo_lock:
        if a != _servo_at:
            servo.write(a)
            await asyncio.sleep_ms(SERVO_BASE_MS + SERVO_MS_PER_DEG * abs(a - _servo_at) + SCAN_DWELL_MS)
            _servo_at = a
        if n <= 1:
            d = sonar.read(retry=retry)
        else:
            vals = []
            for i in range(n):
                v = sonar.read(retry=True)
                vals.append(v if v > 0 else MAX_RANGE + 1)
                if i < n - 1:
                    await asyncio.sleep_ms(12)
            vals.sort()
            # Nếu đa số mẫu đo được vật cản hợp lệ, ưu tiên lấy khoảng cách nhỏ hơn để an toàn
            valid = [v for v in vals if v <= MAX_RANGE]
            if valid:
                d = valid[len(valid) // 2]
            else:
                d = -1
        S["angle"] = a
        S["radar"][a] = (d, ticks_ms())
        if ble and BLE_SEND_RADAR and (retry or d != _last_sent):
            ble.send("P,%d,%d\n" % (90 + a, d))     # gửi góc kiểu radar: 90 = thẳng, lớn hơn = trái
        _last_sent = d
    return d


# ====================== CHẾ ĐỘ THỦ CÔNG ==============================
async def manual_sweep(side):
    """Quét 1 lượt sang bên đã chọn: SCAN_STEP, 2*SCAN_STEP ... tới SCAN_RANGE độ (mỗi lần 10 độ),
    xong thì quay NGAY về 0 độ. Trong lúc này S["scanning"] = True nên chưa quét tiếp được."""
    sign = 1 if side == "L" else -1
    for k in range(SCAN_STEP, SCAN_RANGE + 1, SCAN_STEP):
        S["radar"].pop(sign * k, None)               # xoá số đo cũ của bên này
    for k in range(SCAN_STEP, SCAN_RANGE + 1, SCAN_STEP):
        if S["mode"] == "AUTO":
            break
        await _visit(sign * k, True)
    await _visit(0, False)                           # về góc 0 độ ban đầu
    S["scanning"] = False


async def scan_task():
    """Servo + cảm biến ở chế độ THỦ CÔNG (chế độ TỰ ĐỘNG do auto_task điều khiển trực tiếp).
       Bình thường servo đứng yên ở 0 độ (thẳng) và chỉ đo phía trước để tự phanh.
       Khi bấm Quét trái / Quét phải: quét 1 lượt rồi về 0 độ."""
    await asyncio.sleep_ms(300)
    while True:
        if S["mode"] == "AUTO":
            await asyncio.sleep_ms(50)
            continue
        side = S["scanreq"]
        if side:
            S["scanreq"] = None
            await manual_sweep(side)
            continue
        await _visit(0, False)
        await asyncio.sleep_ms(HOLD_INTERVAL_MS)


def manual_step():
    """Áp lệnh thủ công xuống động cơ NGAY (gọi trực tiếp khi nhận lệnh, không chờ vòng lặp)."""
    if S["mode"] != "MAN":
        return
    c = S["cmd"]
    if S["timeout"] and c != "S" and ticks_diff(ticks_ms(), S["last"]) > MANUAL_TIMEOUT_MS:
        c = S["cmd"] = "S"             # mất tín hiệu -> dừng cho an toàn
    sp = S["speed"]
    k = TURN_RATIO
    l = r = 0
    if c == "F":
        l = r = sp
    elif c == "B":
        l = r = -sp
    elif c == "L":
        l, r = -sp, sp                 # quay trái tại chỗ
    elif c == "R":
        l, r = sp, -sp                 # quay phải tại chỗ
    elif c == "G":
        l, r = sp * k, sp              # tiến + rẽ trái
    elif c == "I":
        l, r = sp, sp * k              # tiến + rẽ phải
    elif c == "H":
        l, r = -sp * k, -sp            # lùi + rẽ trái
    elif c == "J":
        l, r = -sp, -sp * k            # lùi + rẽ phải
    S["phase"] = "MAN"
    forward = c in ("F", "G", "I")
    if S["guard"] and forward:
        f = front_dist(3000 if S["scanning"] else 1200)
        if f is not None and f <= GUARD_DIST:
            l = r = 0
            S["phase"] = "BLOCK"
    car.set(l, r)
    S["cur"] = int(max(abs(l), abs(r)))


async def manual_task():
    while True:                                # chỉ còn lo timeout + tự phanh
        await asyncio.sleep_ms(20)
        manual_step()


# ====================== CHẾ ĐỘ TỰ ĐỘNG ==============================
async def run_for(l, r, ms):
    """Chạy động cơ trong ms mili-giây, huỷ ngay nếu rời chế độ AUTO."""
    if S["mode"] != "AUTO":
        return
    car.set(l, r)
    end = ticks_add(ticks_ms(), ms)
    while ticks_diff(end, ticks_ms()) > 0 and S["mode"] == "AUTO":
        await asyncio.sleep_ms(15)
    if S["mode"] == "AUTO":
        await car.brake()


async def turn_by(deg):
    """deg > 0: quay trái, deg < 0: quay phải (tại chỗ)."""
    dur = int(abs(deg) * TURN_MS_PER_DEG)
    if dur <= 0:
        return
    if deg >= 0:
        await run_for(-TURN_SPEED, TURN_SPEED, dur)
    else:
        await run_for(TURN_SPEED, -TURN_SPEED, dur)


def _eff(raw, a):
    """Khoảng trống hiệu dụng ở góc a. Mất tiếng vọng (-1) KHÔNG đồng nghĩa với thoáng: ở góc tường hoặc
    bề mặt nghiêng, tiếng siêu âm phản xạ lệch đi nên cảm biến không nghe lại. Nếu góc bên cạnh thấy
    vật gần thì lấy số đo gần đó."""
    d = raw[a]
    if d > 0:
        return d
    near = [raw[x] for x in (a - SCAN_STEP, a + SCAN_STEP)
            if raw.get(x, -1) > 0 and raw[x] <= UNKNOWN_TRUST_DIST]
    return min(near) if near else MAX_RANGE + 1


def _candidates():
    """Thứ tự thử: 0, +10, -10, +20, -20 ... tới +-80 (ưu tiên hướng ít phải quay)."""
    seq = [0]
    for k in range(SCAN_STEP, SCAN_RANGE + 1, SCAN_STEP):
        seq.append(k)
        seq.append(-k)
    return seq


async def search_direction():
    """Dừng quét các hướng sang 2 bên (từ 0 độ ra tới +-80 độ):
    - Ở mỗi hướng a, xét cả hành lang bề ngang (a - 10, a, a + 10 độ) để thân xe không quệt vật cản.
    - Nếu hướng đó thông thoáng (khoảng cách hiệu dụng >= CLEAR_DIST = 150 cm) thì chọn ngay.
    - Nếu không có hướng nào thông thoáng hoàn toàn, quét hết các góc và chọn hướng có khoảng cách
      từ xe tới vật cản xa nhất.
    Trả về (góc_tốt_nhất, khoảng_cách_cm) hoặc None nếu bị ngắt."""
    S["phase"] = "SCAN"
    raw = {}
    best_a, best_d = 0, -1
    for a in _candidates():
        win = []
        for x in (a - SCAN_STEP, a, a + SCAN_STEP):
            if abs(x) > SCAN_RANGE:
                continue
            if S["mode"] != "AUTO":
                return None
            if x not in raw:
                raw[x] = await _visit(x, True, AUTO_SAMPLES)
            win.append(x)
        d = min(_eff(raw, x) for x in win)
        if d >= CLEAR_DIST:
            return a, d
        if d > best_d:                          # hoà thì giữ hướng ít phải quay hơn (thử trước)
            best_a, best_d = a, d
    return best_a, best_d


def calc_step_cm(D, open_ahead=False):
    """Tính khoảng cách 1 lần di chuyển (cm) theo nguyên tắc:
    - Nếu phía trước thông thoáng (open_ahead hoặc D > MAX_RANGE): di chuyển tối đa OPEN_MOVE_CM (150 cm = 1,5 m).
    - Khi có vật cản ở khoảng cách D: xe càng gần vật cản thì khoảng cách 1 lần di chuyển càng ngắn.
      + Phần khoảng trống khả dụng sau khi trừ đệm an toàn SAFE_DIST: avail = D - SAFE_DIST.
      + Tỉ lệ bước đi giảm dần tuyến tính từ MAX_MOVE_FRAC (khi ở xa) xuống MIN_MOVE_FRAC (khi ở gần).
      + Đảm bảo không bao giờ vượt quá OPEN_MOVE_CM (150 cm) và luôn chừa lại ít nhất SAFE_DIST.
    - Nếu D <= SAFE_DIST + MIN_STEP_CM (quá gần vật cản): trả về 0 để xe dừng và đổi hướng."""
    if open_ahead or D > MAX_RANGE:
        return OPEN_MOVE_CM
    avail = D - SAFE_DIST
    if avail < MIN_STEP_CM:
        return 0
    span = max(1, MAX_RANGE - SAFE_DIST)
    ratio = MIN_MOVE_FRAC + (MAX_MOVE_FRAC - MIN_MOVE_FRAC) * min(1.0, max(0.0, avail / span))
    step = int(round(avail * ratio))
    step = max(MIN_STEP_CM, min(OPEN_MOVE_CM, step, int(avail)))
    return step


async def front_probe(wide=True):
    """Đo kiểm tra hành lang phía trước theo hướng mũi xe (0 độ):
    - Kiểm tra các góc lân cận (-20, -10, 0, +10, +20 khi wide=True, hoặc -10, +10, 0) rồi đưa servo về 0 độ.
    - Trả về khoảng cách gần nhất để đảm bảo toàn bộ bề ngang xe không vướng vật cản.
    - Trả về None nếu rời chế độ AUTO."""
    angles = (-2 * SCAN_STEP, -SCAN_STEP, SCAN_STEP, 2 * SCAN_STEP, 0) if wide else (-SCAN_STEP, SCAN_STEP, 0)
    vals = []
    for a in angles:
        if S["mode"] != "AUTO":
            return None
        vals.append(await _visit(a, True, AUTO_SAMPLES))
    ok = [v for v in vals if v > 0]
    return min(ok) if ok else MAX_RANGE + 1


async def drive_forward(D, step_cm):
    """Tiến đúng 1 bước step_cm (tối đa 150 cm) rồi dừng hẳn để quét lại:
    - Kết hợp cả 2 cơ chế kiểm soát quãng đường:
      1) Đo khoảng cách thực tế liên tục bằng cảm biến ở góc 0 độ:
         + Khi biết vật cản ban đầu ở khoảng cách D: mục tiêu dừng là khi khoảng cách thực tế còn
           target_rem = max(SAFE_DIST, D - step_cm). Hễ cảm biến thấy khoảng cách <= target_rem là phanh ngay.
         + Khi ban đầu thông thoáng (D > MAX_RANGE): nếu đang đi mà phát hiện vật cản mới ở khoảng cách d,
           lập tức cập nhật lại quãng đường bước đi ngắn lại theo d (càng gần càng ngắn) để không húc vào vật cản.
      2) Ước lượng thời gian tối đa theo STEP_MS_PER_CM để vẫn dừng đúng bước ngay cả khi mặt thoáng không có echo.
    - Chống nhiễu mất echo (-1) khi đang ở gần vật cản: nếu vừa thấy vật cản gần mà mất echo thì đo xác nhận lại
      chứ không coi là thông thoáng."""
    if step_cm <= 0 or S["mode"] != "AUTO":
        return
    step_cm = min(step_cm, OPEN_MOVE_CM)
    cap = S["aspeed"]
    ms_cm = STEP_MS_PER_CM * AUTO_MAX_SPEED / max(5, cap)
    total_ms = int(step_cm * ms_cm)

    # Khoảng cách còn lại mục tiêu cần phải dừng để kết thúc bước đi này
    known_obs = D <= MAX_RANGE
    target_rem = max(SAFE_DIST, D - step_cm) if known_obs else SAFE_DIST
    last_valid_d = D if known_obs else (MAX_RANGE + 1)
    missed_near = 0

    S["phase"] = "DRIVE"
    t0 = ticks_ms()
    while S["mode"] == "AUTO":
        el = ticks_diff(ticks_ms(), t0)
        if el >= total_ms:
            break

        traveled_est = el / ms_cm
        rem_est = (D - traveled_est) if known_obs else (OPEN_MOVE_CM - traveled_est + SAFE_DIST)
        if rem_est <= AUTO_EMERGENCY_CM:
            break

        # Đo trực tiếp phía trước (nếu lần trước thấy vật đang ở gần <= 70 cm thì bật retry để tránh mất mẫu)
        d = await _visit(0, last_valid_d <= 70)
        if d > 0:
            missed_near = 0
            last_valid_d = d
            look = d
            # Nếu lúc đầu tưởng thoáng (hoặc vật cản ở xa hơn) nhưng khi tiến lại phát hiện vật cản gần hơn dự kiến:
            # lập tức thu ngắn bước di chuyển theo khoảng cách thực tế d vừa phát hiện
            if not known_obs or d < D - traveled_est - 5:
                known_obs = True
                new_step = calc_step_cm(d, False)
                if new_step <= 0:
                    break
                target_rem = max(SAFE_DIST, d - new_step)
                rem_ms = int(new_step * ms_cm)
                total_ms = min(total_ms, el + rem_ms)
        else:
            # d == -1 (không nhận được tiếng vọng)
            if last_valid_d <= 65:
                # Đang ở gần vật cản mà mất echo: rất dễ do sóng phản xạ lệch hoặc quá sát -> KHÔNG coi là thoáng
                missed_near += 1
                look = max(AUTO_EMERGENCY_CM, int(last_valid_d - traveled_est * 0.5))
                if missed_near >= 2 or last_valid_d <= SAFE_DIST + 5:
                    break
            else:
                look = MAX_RANGE + 1

        # Dừng ngay nếu gặp vật cản trong ngưỡng khẩn cấp hoặc đã đi đủ bước (khoảng cách còn lại <= target_rem)
        if look <= AUTO_EMERGENCY_CM or (known_obs and look <= target_rem):
            break

        f = min(look, rem_est)
        if S["autoslow"] and f < SLOW_DIST:                  # tự giảm tốc mượt khi tới gần vật cản
            sp = AUTO_MIN_SPEED + (cap - AUTO_MIN_SPEED) * max(0, f - SAFE_DIST) / max(1, SLOW_DIST - SAFE_DIST)
        else:
            sp = cap
        sp = int(max(min(AUTO_MIN_SPEED, cap), min(cap, sp)))
        S["cur"] = sp
        car.set(sp, sp)
        await asyncio.sleep_ms(10)

    if S["mode"] == "AUTO":
        await car.brake()
    else:
        car.stop()
    S["cur"] = 0


async def rotate_cw():
    """Quay xe 90 độ theo chiều kim đồng hồ (sang phải), cảm biến về 0 độ rồi quét lại."""
    await _visit(0, False)
    S["phase"] = "TURN"
    await turn_by(-90)
    S["radar"].clear()
    await asyncio.sleep_ms(AUTO_SETTLE_MS)


async def report_nopath():
    """Đã quay đủ vòng mà không có hướng đi: dừng xe, về thủ công và báo về giao diện + Bluetooth."""
    car.stop()
    set_mode("MAN")
    S["msg"] = "NOPATH"
    S["msg_t"] = ticks_ms()
    if ble:
        ble.send("NOPATH\n")


async def auto_task():
    """Chu trình tự động theo cơ chế mới:
    1) DỪNG HẲN & QUÉT: Sau mỗi lần di chuyển, xe dừng lại hoàn toàn, chờ hết rung rồi quét đo khoảng cách các hướng.
    2) CHỌN HƯỚNG & QUAY CẢM BIẾN: Hướng nào thông thoáng (>= 150 cm) hoặc có khoảng cách từ xe tới vật cản xa hơn
       thì quay cảm biến về đúng hướng đó (phase LOOK) để xác nhận khoảng cách.
    3) ĐIỀU CHỈNH HƯỚNG XE: Nếu hướng chọn khác 0 độ, điều chỉnh quay xe theo hướng đó (phase TURN) rồi đưa
       cảm biến về 0 độ (thẳng theo mũi xe mới) và kiểm tra hành lang phía trước.
    4) DI CHUYỂN THEO KHOẢNG CÁCH (CÀNG GẦN ĐI CÀNG NGẮN, TỐI ĐA 1,5 M):
       - Nếu thông thoáng: tiến tối đa 1,5 m (150 cm) (vừa đi vừa đo liên tục).
       - Nếu có vật cản: xe càng gần vật cản thì quãng đường 1 lần di chuyển càng ngắn.
       - Đi xong 1 bước lại dừng hẳn để quét và đo lại cho bước tiếp theo.
       - Nếu khoảng cách đã quá gần (<= SAFE_DIST): quay xe tìm đường mới; quá 4 lần (đủ 1 vòng) thì báo NOPATH."""
    rot = 0                                    # số lần quay 90 độ liên tiếp mà chưa đi được
    while True:
        if S["mode"] != "AUTO":
            rot = 0
            await asyncio.sleep_ms(100)
            continue

        car.stop()
        S["cur"] = 0
        await asyncio.sleep_ms(AUTO_SETTLE_MS)           # đứng yên cho xe hết rung trước khi quét

        # 1. Quét tìm hướng thông thoáng hoặc có khoảng cách xa nhất
        res = await search_direction()
        if res is None:
            continue
        ang, D = res

        # 2. Quay cảm biến về hướng đã chọn và đo xác nhận lại
        S["phase"] = "LOOK"
        d = await _visit(ang, True, AUTO_SAMPLES)
        if S["mode"] != "AUTO":
            continue
        d_eff = d if d > 0 else (MAX_RANGE + 1)
        D = min(D, d_eff)

        # 3. Điều chỉnh xe quay theo hướng đó (nếu hướng chọn khác 0 độ)
        if calc_step_cm(D, D > MAX_RANGE) > 0:
            if ang != 0:
                S["phase"] = "TURN"
                await turn_by(ang)                       # góc dương = quay trái, góc âm = quay phải
                S["radar"].clear()                       # xoá dữ liệu radar cũ vì góc xe đã đổi
                await asyncio.sleep_ms(AUTO_SETTLE_MS)
                if S["mode"] != "AUTO":
                    continue

            # Đưa cảm biến về 0 độ (nhìn thẳng theo hướng xe mới) và kiểm tra hành lang trước mũi xe
            S["phase"] = "LOOK"
            front = await front_probe(wide=True)
            if front is None:
                continue
            D = min(D, front) if ang == 0 else front

        # 4. Tính quãng đường 1 lần di chuyển (càng gần vật cản càng ngắn, thoáng tối đa 150 cm)
        open_ahead = D > MAX_RANGE
        step_cm = calc_step_cm(D, open_ahead)

        if step_cm <= 0:                                 # quá gần vật cản, không đủ khoảng trống an toàn để tiến
            rot += 1
            if rot >= NOPATH_ROTATIONS:
                await report_nopath()
                rot = 0
            else:
                await rotate_cw()
            continue

        rot = 0
        await drive_forward(D, step_cm)                  # đi hết 1 bước rồi dừng lại để vòng sau quét tiếp


# ========================= XỬ LÝ LỆNH ================================
def set_speed(v):
    v = max(0, min(100, int(v)))
    if S["mode"] == "AUTO":
        S["aspeed"] = min(v, AUTO_MAX_SPEED)   # tự động: không vượt AUTO_MAX_SPEED
    else:
        S["speed"] = v
        manual_step()


def set_mode(m):
    S["mode"] = m
    S["cmd"] = "S"
    car.stop()
    S["cur"] = 0
    S["scanreq"] = None
    S["scanning"] = False                  # nếu đang quét dở thì manual_sweep tự kết thúc
    if m == "AUTO":
        S["msg"] = ""
    S["phase"] = "MAN" if m == "MAN" else "WAIT"


def _cmd(ch, src):
    if ch in "FBLRGIHJ":
        if S["mode"] == "MAN":
            S["cmd"] = ch
            S["last"] = ticks_ms()
            S["timeout"] = (src != "ble")
            manual_step()
    elif ch == "S":
        if S["mode"] == "MAN":
            S["cmd"] = "S"
            S["last"] = ticks_ms()
            manual_step()
    elif ch == "X":                    # dừng khẩn cấp, về thủ công
        set_mode("MAN")
    elif ch == "M":
        set_mode("MAN")
    elif ch == "A":
        set_mode("AUTO")
    elif ch.isdigit():
        set_speed(int(ch) * 10)
    elif ch == "Q":
        set_speed(100)


# Ghép 2 nút hướng thành hướng chéo
COMBO = {
    frozenset("FL"): "G",   # tiến + trái
    frozenset("FR"): "I",   # tiến + phải
    frozenset("BL"): "H",   # lùi + trái
    frozenset("BR"): "J",   # lùi + phải
}


def handle_command(text, src="web"):
    """Giao thức chung cho web & BLE:
       F B L R  tiến/lùi/trái/phải     G I H J  chéo: tiến-trái/tiến-phải/lùi-trái/lùi-phải
       Gửi 2 chữ ghép (FL FR BL BR) cũng được: tự đổi thành G I H J
       S dừng   X dừng khẩn cấp         M thủ công   A tự động
       V<0-100> đặt tốc độ              0-9, Q  tốc độ 0..90, 100
       C1 / C0  bật/tắt tự phanh khi thủ công
       K1 / K0  bật/tắt tự điều chỉnh tốc độ khi tự động
       ZL / ZR  quét trái / quét phải (thủ công): servo quay 80 độ, mỗi lần 10 độ, rồi về 0 độ"""
    text = text.strip()
    if not text:
        return
    if text[0] in "Vv":
        try:
            set_speed(int(text[1:]))
        except ValueError:
            pass
        return
    if text[0] in "Cc":
        S["guard"] = text[1:2] != "0"
        return
    if text[0] in "Kk":
        S["autoslow"] = text[1:2] != "0"
        return
    if text[0] in "Zz":                    # quét thủ công: ZL = quét trái, ZR = quét phải
        side = text[1:2].upper()
        if S["mode"] == "MAN" and not S["scanning"] and side in ("L", "R"):
            S["scanning"] = True
            S["scanreq"] = side
        return
    up = text.upper()
    if len(up) == 2 and up[0] != up[1] and all(c in "FBLR" for c in up):
        # Kết hợp 2 nút hướng -> hướng chéo (VD: F+R = tiến-phải = "I")
        c = COMBO.get(frozenset(up))
        if c:
            _cmd(c, src)
        else:                              # F+B hoặc L+R triệt tiêu nhau -> dừng
            _cmd("S", src)
        return
    for ch in up:
        _cmd(ch, src)


def status():
    now = ticks_ms()
    pts = []
    for a, (d, t) in S["radar"].items():
        age = ticks_diff(now, t)
        if age < 8000:
            pts.append([90 + a, d, age // 100])      # giao diện radar dùng quy ước 90 = thẳng
    f = front_dist(2500)
    return {
        "mode": S["mode"], "cur": S["cur"],
        "speed": S["aspeed"] if S["mode"] == "AUTO" else S["speed"],
        "smax": AUTO_MAX_SPEED if S["mode"] == "AUTO" else 100,
        "phase": S["phase"], "angle": 90 + S["angle"], "guard": S["guard"], "autoslow": S["autoslow"],
        "scanning": S["scanning"],
        "msg": S["msg"] if (S["msg"] and ticks_diff(now, S["msg_t"]) < 10000) else "",
        "front": f if (f is not None and f <= MAX_RANGE) else -1,
        "maxr": MAX_RANGE, "step": SCAN_STEP, "radar": pts,
    }


# ============================ BLUETOOTH ==============================
_NUS_SVC = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
_NUS_TX = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"   # ESP32 -> điện thoại (notify)
_NUS_RX = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"   # điện thoại -> ESP32 (write)


class BLEUart:
    def __init__(self, name):
        self.name = name
        self.conns = set()
        self.rxq = []
        self.ble = bluetooth.BLE()
        self.ble.active(True)
        self.ble.irq(self._irq)
        tx = (bluetooth.UUID(_NUS_TX), bluetooth.FLAG_NOTIFY)
        rx = (bluetooth.UUID(_NUS_RX), bluetooth.FLAG_WRITE | bluetooth.FLAG_WRITE_NO_RESPONSE)
        svc = (bluetooth.UUID(_NUS_SVC), (tx, rx))
        ((self.tx, self.rx),) = self.ble.gatts_register_services((svc,))
        self.ble.gatts_set_buffer(self.rx, 128, True)
        self._advertise()

    def _advertise(self):
        raw = bytes.fromhex(_NUS_SVC.replace("-", ""))
        uuid = bytes([raw[i] for i in range(len(raw) - 1, -1, -1)])   # đảo byte (MicroPython không hỗ trợ [::-1])
        nm = self.name.encode()
        adv = bytes((2, 0x01, 0x06, len(nm) + 1, 0x09)) + nm
        resp = bytes((17, 0x07)) + uuid
        self.ble.gap_advertise(100000, adv_data=adv, resp_data=resp)

    def _irq(self, event, data):
        if event == 1:                       # kết nối
            self.conns.add(data[0])
        elif event == 2:                     # ngắt kết nối
            self.conns.discard(data[0])
            self.rxq.append(b"S")            # dừng xe nếu đang chạy tay
            self._advertise()
        elif event == 3:                     # nhận dữ liệu
            if data[1] == self.rx:
                self.rxq.append(self.ble.gatts_read(self.rx))

    def send(self, text):
        data = text.encode()[:20]
        for c in list(self.conns):
            try:
                self.ble.gatts_notify(c, self.tx, data)
            except OSError:
                pass


async def ble_task():
    n = 0
    while True:
        await asyncio.sleep_ms(10)
        while ble.rxq:
            raw = ble.rxq.pop(0)
            try:
                for line in raw.decode().replace("\r", "\n").split("\n"):
                    handle_command(line, "ble")
            except Exception:
                pass
        n += 1
        if n >= 50 and ble.conns:            # ~0.5 s gửi trạng thái 1 lần
            n = 0
            f = front_dist(2500)
            f = f if (f is not None and f <= MAX_RANGE) else -1
            ble.send("ST,%s,%d,%d\n" % (S["mode"], S["aspeed"] if S["mode"] == "AUTO" else S["speed"], f))


# ============================ WEB SERVER =============================
def query_param(path, key):
    if "?" not in path:
        return ""
    for kv in path.split("?", 1)[1].split("&"):
        k, _, v = kv.partition("=")
        if k == key:
            return v
    return ""


async def respond(w, code, ctype, body):
    if isinstance(body, str):
        body = body.encode()
    head = ("HTTP/1.0 %s\r\nContent-Type: %s\r\nContent-Length: %d\r\n"
            "Cache-Control: no-store\r\nConnection: close\r\n\r\n") % (code, ctype, len(body))
    w.write(head.encode())
    w.write(body)
    await w.drain()


async def send_file(w, path):
    w.write(b"HTTP/1.0 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n"
            b"Cache-Control: no-cache\r\nConnection: close\r\n\r\n")
    with open(path, "rb") as f:
        while True:
            chunk = f.read(1024)
            if not chunk:
                break
            w.write(chunk)
            await w.drain()


_WS_GUID = b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
WS_CLIENTS = set()


def ws_frame(op, payload=b""):
    n = len(payload)
    if n < 126:
        return bytes((0x80 | op, n)) + payload
    return bytes((0x80 | op, 126, n >> 8, n & 255)) + payload


async def ws_read(reader):
    h = await reader.readexactly(2)
    op = h[0] & 0x0F
    ln = h[1] & 0x7F
    if ln == 126:
        ln = int.from_bytes(await reader.readexactly(2), "big")
    elif ln == 127:
        return 8, b""                        # khung quá lớn -> đóng
    mask = await reader.readexactly(4) if (h[1] & 0x80) else None
    if ln > 512:
        return 8, b""
    data = await reader.readexactly(ln) if ln else b""
    if mask:
        data = bytes([b ^ mask[i & 3] for i, b in enumerate(data)])
    return op, data


async def ws_session(reader, writer, key):
    """Kết nối WebSocket giữ mở: lệnh đến ESP32 không còn phải bắt tay TCP/HTTP mỗi lần bấm."""
    acc = binascii.b2a_base64(hashlib.sha1(key + _WS_GUID).digest()).strip()
    writer.write(b"HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n"
                 b"Connection: Upgrade\r\nSec-WebSocket-Accept: " + acc + b"\r\n\r\n")
    await writer.drain()
    WS_CLIENTS.add(writer)
    try:
        while True:
            op, data = await asyncio.wait_for(ws_read(reader), 10)
            if op == 8:
                break
            if op == 9:                      # ping -> pong
                writer.write(ws_frame(10, data))
            elif op == 1:                    # văn bản = lệnh điều khiển
                try:
                    handle_command(data.decode(), "web")
                except Exception:
                    pass
    except Exception:
        pass
    finally:
        WS_CLIENTS.discard(writer)
        handle_command("S", "web")           # mất kết nối -> dừng xe (chỉ tác dụng ở chế độ thủ công)


async def ws_push_task():
    """Đẩy trạng thái + radar cho mọi trình duyệt ~5 lần/giây (không cần trình duyệt hỏi liên tục)."""
    while True:
        await asyncio.sleep_ms(200)
        if not WS_CLIENTS:
            continue
        frame = ws_frame(1, json.dumps(status()).encode())
        for w in list(WS_CLIENTS):
            try:
                w.write(frame)
                await asyncio.wait_for(w.drain(), 1)
            except Exception:
                WS_CLIENTS.discard(w)


async def handle_http(reader, writer):
    try:
        line = await asyncio.wait_for(reader.readline(), 5)
        key = None
        for _ in range(40):                  # đọc header, giữ lại khoá WebSocket nếu có
            h = await asyncio.wait_for(reader.readline(), 5)
            if not h or h == b"\r\n":
                break
            if h[:18].lower() == b"sec-websocket-key:":
                key = h[18:].strip()
        parts = line.decode().split()
        if len(parts) < 2:
            return
        path = parts[1]
        if path.startswith("/ws") and key:
            await ws_session(reader, writer, key)
        elif path.startswith("/api/status"):
            await respond(writer, "200 OK", "application/json", json.dumps(status()))
        elif path.startswith("/api/cmd"):
            handle_command(query_param(path, "c"), "web")
            await respond(writer, "200 OK", "application/json", '{"ok":1}')
        elif path == "/" or path.startswith("/index"):
            try:
                await send_file(writer, "index.html")
            except OSError:
                await respond(writer, "404 Not Found", "text/plain; charset=utf-8",
                              "Thieu file index.html tren board. Hay upload index.html cung main.py.")
        else:
            await respond(writer, "404 Not Found", "text/plain", "404")
    except Exception:
        pass
    finally:
        try:
            writer.close()
            await writer.wait_closed()
        except Exception:
            pass


def start_wifi():
    if STA_SSID:
        sta = network.WLAN(network.STA_IF)
        sta.active(True)
        try:
            sta.config(pm=network.WLAN.PM_NONE)   # tắt tiết kiệm điện WiFi -> giảm trễ
        except Exception:
            pass
        sta.connect(STA_SSID, STA_PASS)
        t0 = ticks_ms()
        while not sta.isconnected() and ticks_diff(ticks_ms(), t0) < 15000:
            time.sleep_ms(200)
        if sta.isconnected():
            return "WiFi nha", sta.ifconfig()[0]
        sta.active(False)
    ap = network.WLAN(network.AP_IF)
    ap.active(True)
    ap.config(essid=AP_SSID, password=AP_PASS, authmode=network.AUTH_WPA_WPA2_PSK)
    return "AP '%s' (mat khau %s)" % (AP_SSID, AP_PASS), ap.ifconfig()[0]


# =============================== MAIN ================================
async def main():
    global ble
    kind, ip = start_wifi()
    print("WiFi:", kind)
    print("Mo trinh duyet: http://%s" % ip)
    ble = BLEUart(BLE_NAME)
    print("BLE:", BLE_NAME)
    await asyncio.start_server(handle_http, "0.0.0.0", 80, backlog=5)
    asyncio.create_task(scan_task())
    asyncio.create_task(manual_task())
    asyncio.create_task(auto_task())
    asyncio.create_task(ble_task())
    asyncio.create_task(ws_push_task())
    while True:
        await asyncio.sleep(3600)


try:
    asyncio.run(main())
finally:
    car.stop()
    asyncio.new_event_loop()