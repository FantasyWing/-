from machine import ADC, Pin, PWM
import time

# ============================================
# 传感器部分 (不变)
# ============================================
SENSOR_PINS = {
    'adc1': 27,
    'adc2': 33,
    'adc3': 32,
    'adc4': 35,
    'adc5': 34
}
adc_objects = {}

def init_sensors():
    global adc_objects
    for name, pin in SENSOR_PINS.items():
        adc = ADC(Pin(pin))
        adc.atten(ADC.ATTN_11DB)
        adc.width(ADC.WIDTH_12BIT)
        adc_objects[name] = adc

def read_sensors_binary():
    result = []
    for name in ['adc1', 'adc2', 'adc3', 'adc4', 'adc5']:
        raw = adc_objects[name].read()
        result.append(0 if raw >= 3000 else 1)# 阈值为3000
    return result

# ============================================
# 电机部分 (不变)
# ============================================
PWM_FREQ = 20000

pin_m1_in1 = Pin(13, Pin.OUT, value=0)
pin_m1_in2 = Pin(15, Pin.OUT, value=0)
pwm_m1_in1 = PWM(pin_m1_in1, freq=PWM_FREQ, duty=0)
pwm_m1_in2 = PWM(pin_m1_in2, freq=PWM_FREQ, duty=0)

pin_m2_in1 = Pin(14, Pin.OUT, value=0)
pin_m2_in2 = Pin(25, Pin.OUT, value=0)
pwm_m2_in1 = PWM(pin_m2_in1, freq=PWM_FREQ, duty=0)
pwm_m2_in2 = PWM(pin_m2_in2, freq=PWM_FREQ, duty=0)

def init_motors():
    print("电机初始化完成 (左: G13/G15, 右: G14/G25)")

def motor(left_speed, right_speed):
    left_speed = max(-100, min(100, int(left_speed)))
    right_speed = max(-100, min(100, int(right_speed)))

    dl = abs(left_speed) * 1023 // 100
    if left_speed < 0:
        pwm_m1_in1.duty(dl); pwm_m1_in2.duty(0)
    elif left_speed > 0:
        pwm_m1_in1.duty(0); pwm_m1_in2.duty(dl)
    else:
        pwm_m1_in1.duty(0); pwm_m1_in2.duty(0)

    dr = abs(right_speed) * 1023 // 100
    if right_speed > 0:
        pwm_m2_in1.duty(dr); pwm_m2_in2.duty(0)
    elif right_speed < 0:
        pwm_m2_in1.duty(0); pwm_m2_in2.duty(dr)
    else:
        pwm_m2_in1.duty(0); pwm_m2_in2.duty(0)

# ============================================
# 循迹参数
# ============================================
BASE_SPEED = 77 # 79
KP = 9
KD = 8

def compute_deviation(sensor_data):
    total = 0
    count = 0
    for i, val in enumerate(sensor_data):
        if val == 1:
            total += i * 1.0
            count += 1
    if count == 0:
        return None
    center = total / count
    deviation = (center - 2.0)   # 负左正右
    return deviation

# ============================================
# 主程序（含十字路口判定）
# ============================================
CROSS_COUNT = 0
CROSS_THRESHOLD = 3
CROSS_DURATION = 15

if __name__ == "__main__":
    init_sensors()
    init_motors()

    print("开始五路循迹（带十字路口判定）")
    last_deviation = 0
    cross_mode = False
    cross_timer = 0

    while True:
        data = read_sensors_binary()
        print("传感器:", data)
        count = sum(data)

        # ---- 十字路口判定 ----
        if count == 5 and not cross_mode:
            CROSS_COUNT += 1
            if CROSS_COUNT >= CROSS_THRESHOLD:
                cross_mode = True
                cross_timer = CROSS_DURATION
                print("进入十字路口模式")
        else:
            CROSS_COUNT = 0

        if cross_mode:
            motor(BASE_SPEED - 2, BASE_SPEED - 2)
            cross_timer -= 1
            if cross_timer <= 0:
                cross_mode = False
                print("退出十字路口模式")
            time.sleep_ms(5)
            continue

        # ---- 正常循迹 ----
        if count == 0:
            if last_deviation >= 0:
                motor(70, -70)
            else:
                motor(-70, 70)
            time.sleep_ms(5)
            continue

        # if (count == 0) or (count == 1 and data[2] == 1):
        #     motor(80, 80)
        #     time.sleep_ms(10)
        #     continue

        dev = compute_deviation(data)
        if dev is None:
            continue
        # PD 控制：P项按偏差比例修正，D项抑制偏差变化速度
        correction = KP * dev + KD * (dev - last_deviation)
        last_deviation = dev

        left_speed = BASE_SPEED + correction 
        right_speed = BASE_SPEED - correction 

        left_speed = max(-100, min(100, left_speed))
        right_speed = max(-100, min(100, right_speed))

        motor(left_speed, right_speed)
        time.sleep_ms(5)