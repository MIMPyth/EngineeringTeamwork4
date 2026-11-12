"""Two-axis tray controller. BCM pins. MPU6050 is fixed to the moving basket."""
import math
import struct
import time
from contextlib import ExitStack

from gpiozero import Servo, DigitalInputDevice, PWMOutputDevice, RGBLED
from smbus2 import SMBus

# Pins must match the wiring guide.
PITCH_PIN, ROLL_PIN, IR_PIN, BUZZER_PIN = 12, 13, 17, 23
USE_RGB = False              # Enable after checking your LED/module wiring.
RGB_PINS = (16, 20, 21)      # R, G, B; physical pins 36, 38, 40.
RGB_ACTIVE_HIGH = True       # False for common-anode RGB LEDs.
USE_LCD = False              # This driver supports a PCF8574 LCD1602 backpack.
LCD_ADDRESS = 0x27           # Some backpacks use 0x3F.
MPU_ADDRESS = 0x68
IR_ACTIVE_LOW = True

# Sensor chip face UP; its X arrow parallel to the left-right pivot shaft.
# Check each motor direction with an empty basket, changing ONE sign if needed.
PITCH_DIRECTION, ROLL_DIRECTION = 1, 1
LIMIT_DEG = 10.0             # Estimated servo angles; start with 5 if uncertain.
KP, KD = 2.0, 0.35          # Angle error -> servo speed, with gyro damping.
MAX_SPEED = 15.0            # Estimated servo degrees/second.
DEADBAND = 0.7              # Degrees of tray error.
PERIOD = 0.02               # 50 Hz control loop.


def clamp(value, low, high):
    return max(low, min(high, value))


def angles(sample):
    ax, ay, az = sample[:3]
    return (math.degrees(math.atan2(ay, az)),
            math.degrees(math.atan2(-ax, math.hypot(ay, az))))


def read_imu(bus):
    data = bus.read_i2c_block_data(MPU_ADDRESS, 0x3B, 14)
    ax, ay, az, temperature, gx, gy, gz = struct.unpack('>7h', bytes(data))
    return (ax / 16384, ay / 16384, az / 16384,
            gx / 131, gy / 131, gz / 131)


def calibrate(bus):
    print('Hold the basket level and still for 2 seconds.')
    samples = []
    for _ in range(100):
        samples.append(read_imu(bus))
        time.sleep(PERIOD)
    average = [sum(s[i] for s in samples) / len(samples) for i in range(6)]
    magnitude = math.sqrt(sum(v * v for v in average[:3]))
    gyro_noise = max(max(s[i] for s in samples) - min(s[i] for s in samples)
                     for i in (3, 4, 5))
    zero = angles(average)
    if not 0.85 < magnitude < 1.15 or average[2] < 0.8:
        raise RuntimeError('Place sensor flat, chip face UP, and retry.')
    if max(abs(v) for v in zero) > 10 or gyro_noise > 5:
        raise RuntimeError('Sensor moved or was tilted during calibration. Retry.')
    return zero, average[3:]


class LCD1602:
    """Common PCF8574 mapping: RS=P0, E=P2, light=P3, data=P4..P7."""
    def __init__(self, bus):
        self.bus = bus
        time.sleep(0.05)
        for nibble in (0x30, 0x30, 0x30, 0x20):
            self.nibble(nibble)
            time.sleep(0.005)
        for command in (0x28, 0x0C, 0x06, 0x01):
            self.send(command)

    def nibble(self, value):
        value |= 0x08
        self.bus.write_byte(LCD_ADDRESS, value)
        self.bus.write_byte(LCD_ADDRESS, value | 0x04)
        time.sleep(0.0005)
        self.bus.write_byte(LCD_ADDRESS, value)

    def send(self, value, data=False):
        rs = int(data)
        self.nibble((value & 0xF0) | rs)
        self.nibble(((value << 4) & 0xF0) | rs)
        time.sleep(0.002)

    def show(self, first, second):
        for address, line in ((0x80, first), (0xC0, second)):
            self.send(address)
            for character in line[:16].ljust(16):
                self.send(ord(character), data=True)


def next_position(position, error, rate, direction, dt):
    if abs(error) < DEADBAND:
        error = 0.0
    speed = clamp(-direction * (KP * error + KD * rate), -MAX_SPEED, MAX_SPEED)
    return clamp(position + speed * dt, -LIMIT_DEG, LIMIT_DEG)


def main():
    with ExitStack() as resources:
        bus = resources.enter_context(SMBus(1))
        if bus.read_byte_data(MPU_ADDRESS, 0x75) != 0x68:
            raise RuntimeError('MPU6050 identity check failed. Check sensor/address.')
        for register, value in ((0x6B, 1), (0x1A, 3), (0x1B, 0),
                                (0x1C, 0), (0x19, 4)):
            bus.write_byte_data(MPU_ADDRESS, register, value)
        time.sleep(0.1)
        ir = resources.enter_context(DigitalInputDevice(
            IR_PIN, pull_up=None, active_state=not IR_ACTIVE_LOW))
        buzzer = resources.enter_context(PWMOutputDevice(
            BUZZER_PIN, frequency=2000, initial_value=0))
        rgb = resources.enter_context(RGBLED(
            *RGB_PINS, active_high=RGB_ACTIVE_HIGH)) if USE_RGB else None
        lcd = LCD1602(bus) if USE_LCD else None
        motors = [resources.enter_context(Servo(
            pin, initial_value=None, min_pulse_width=0.001,
            max_pulse_width=0.002, frame_width=0.020))
            for pin in (PITCH_PIN, ROLL_PIN)]
        zero, bias = calibrate(bus)
        filtered = list(zero)
        print('Calibration complete. Ctrl+C stops and releases both motors.')
        armed = input('Type RUN to enable motors, or Enter for sensors only: ').strip() == 'RUN'
        if armed:
            for motor in motors:
                motor.value = 0
            time.sleep(1)
        positions = [0.0, 0.0]
        last = time.monotonic()
        last_output = last
        candidate = ir.is_active
        cup_present = False
        candidate_since = last
        limit_since = None
        fault = ''
        try:
            while True:
                now = time.monotonic()
                dt = now - last
                last = now
                if dt > 0.25:
                    fault = 'LOOP STALLED'
                dt = clamp(dt, 0.001, 0.1)
                try:
                    sample = read_imu(bus)
                except OSError:
                    fault = 'IMU READ FAILED'
                    sample = None
                if sample is not None:
                    measured = angles(sample)
                    rates = [sample[3] - bias[0], sample[4] - bias[1]]
                    magnitude = math.sqrt(sum(v * v for v in sample[:3]))
                    alpha = 0.5 / (0.5 + dt) if 0.8 < magnitude < 1.2 else 1.0
                    for i in range(2):
                        filtered[i] = (alpha * (filtered[i] + rates[i] * dt)
                                       + (1 - alpha) * measured[i])
                    errors = [filtered[i] - zero[i] for i in range(2)]
                    if max(abs(v) for v in errors) > 25:
                        fault = 'EXCESS TILT'
                detected = ir.is_active
                if detected != candidate:
                    candidate, candidate_since = detected, now
                if now - candidate_since >= 0.25:
                    cup_present = candidate
                if armed and cup_present and not fault:
                    for i, direction in enumerate((PITCH_DIRECTION, ROLL_DIRECTION)):
                        positions[i] = next_position(positions[i], errors[i], rates[i], direction, dt)
                        motors[i].value = positions[i] / 90.0
                    limited = any(abs(positions[i]) >= LIMIT_DEG - 0.1
                                  and abs(errors[i]) > 4 for i in range(2))
                    limit_since = (now if limit_since is None else limit_since) if limited else None
                    if limit_since is not None and now - limit_since > 0.7:
                        fault = 'TRAVEL LIMIT'
                else:
                    limit_since = None
                # No cup: hold the last pose. Fault: release and require a restart.
                if fault:
                    for motor in motors:
                        motor.detach()
                buzzer.value = 0.5 if fault and now % 1.0 < 0.2 else 0
                status = fault or ('LEVELING' if armed and cup_present else
                                  'SENSORS ONLY' if not armed else 'WAITING FOR CUP')
                if rgb:
                    rgb.color = (1, 0, 0) if fault else (0, 1, 0) if armed and cup_present else (0, 0, 1)
                if now - last_output >= 0.5:
                    last_output = now
                    angle_line = (f'P{errors[0]:+5.1f} R{errors[1]:+5.1f}'
                                  if sample is not None else 'Sensor unavailable')
                    print(f'{status:18} {angle_line}  Cup={cup_present}', flush=True)
                    if lcd:
                        try:
                            lcd.show(status, angle_line)
                        except OSError as exc:
                            print(f'LCD disabled after I2C error: {exc}')
                            lcd = None
                time.sleep(max(0, PERIOD - (time.monotonic() - now)))
        finally:
            for motor in motors:
                motor.detach()
            buzzer.off()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nStopped. Disconnect servo power before adjusting the mechanism.')
    except (OSError, RuntimeError) as exc:
        print(f'Stopped: {exc}')
