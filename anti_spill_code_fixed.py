import math
import struct
import time
from contextlib import ExitStack

from gpiozero import (
    Servo,
    DigitalInputDevice,
    PWMOutputDevice,
    RGBLED,
)
from smbus2 import SMBus

# BCM GPIO numbers
PITCH_PIN = 12       # Physical pin 32
ROLL_PIN = 13        # Physical pin 33
IR_PIN = 17          # Physical pin 11
BUZZER_PIN = 23      # Physical pin 16

MPU_ADDRESS = 0x68   # Change to 0x69 only if your scan shows 69
IR_ACTIVE_LOW = True

USE_RGB = False
RGB_PINS = (16, 20, 21)
RGB_ACTIVE_HIGH = True

USE_LCD = False
LCD_ADDRESS = 0x27

# Change an axis direction to -1 if it increases the tilt.
PITCH_DIRECTION = 1
ROLL_DIRECTION = 1

LIMIT_DEG = 10.0
MAX_SPEED = 15.0
KP = 2.0
KD = 0.35
DEADBAND = 0.7
PERIOD = 0.02


def clamp(value, low, high):
    return max(low, min(high, value))


def angles(sample):
    ax, ay, az = sample[:3]

    pitch = math.degrees(math.atan2(ay, az))
    roll = math.degrees(
        math.atan2(-ax, math.hypot(ay, az))
    )

    return pitch, roll


def setup_imu(bus):
    identity = bus.read_byte_data(MPU_ADDRESS, 0x75)

    if identity != 0x68:
        raise RuntimeError(
            f"Unexpected sensor identity: {identity:#04x}"
        )

    settings = (
        (0x6B, 1),     # Wake sensor
        (0x1A, 3),     # Low-pass filter
        (0x1B, 0),     # Gyroscope: +/-250 degrees/second
        (0x1C, 0),     # Accelerometer: +/-2 g
        (0x19, 4),     # Sample-rate divider
    )

    for register, value in settings:
        for attempt in range(3):
            try:
                bus.write_byte_data(
                    MPU_ADDRESS, register, value
                )
                time.sleep(0.1)
                break

            except OSError as exc:
                if attempt == 2:
                    raise RuntimeError(
                        f"MPU setup failed at register "
                        f"{register:#04x}: {exc}"
                    ) from exc

                time.sleep(0.1)

    print("MPU6050 setup complete.")


def read_imu(bus):
    data = bus.read_i2c_block_data(
        MPU_ADDRESS, 0x3B, 14
    )

    ax, ay, az, temperature, gx, gy, gz = struct.unpack(
        ">7h", bytes(data)
    )

    return (
        ax / 16384,
        ay / 16384,
        az / 16384,
        gx / 131,
        gy / 131,
        gz / 131,
    )


def calibrate(bus):
    print("Hold the basket LEVEL and STILL for 2 seconds.")
    samples = []

    for _ in range(100):
        samples.append(read_imu(bus))
        time.sleep(PERIOD)

    average = [
        sum(sample[i] for sample in samples) / len(samples)
        for i in range(6)
    ]

    magnitude = math.sqrt(
        sum(value * value for value in average[:3])
    )

    gyro_noise = max(
        max(sample[i] for sample in samples)
        - min(sample[i] for sample in samples)
        for i in (3, 4, 5)
    )

    zero = angles(average)

    if not 0.85 < magnitude < 1.15 or average[2] < 0.8:
        raise RuntimeError(
            "Place the sensor flat with its chip face UP."
        )

    if max(abs(value) for value in zero) > 10:
        raise RuntimeError(
            "Sensor is tilted. Level the basket and retry."
        )

    if gyro_noise > 5:
        raise RuntimeError(
            "Sensor moved during calibration. Retry."
        )

    print("Calibration complete.")
    return zero, average[3:]


class LCD1602:
    """For a standard PCF8574 LCD1602 backpack."""

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
        for address, line in (
            (0x80, first),
            (0xC0, second),
        ):
            self.send(address)

            for character in line[:16].ljust(16):
                self.send(ord(character), data=True)


def next_position(position, error, rate, direction, dt):
    if abs(error) < DEADBAND:
        error = 0.0

    speed = -direction * (KP * error + KD * rate)
    speed = clamp(speed, -MAX_SPEED, MAX_SPEED)

    return clamp(
        position + speed * dt,
        -LIMIT_DEG,
        LIMIT_DEG,
    )


def main():
    with ExitStack() as resources:
        bus = resources.enter_context(SMBus(1))
        setup_imu(bus)

        ir = resources.enter_context(
            DigitalInputDevice(
                IR_PIN,
                pull_up=None,
                active_state=not IR_ACTIVE_LOW,
            )
        )

        buzzer = resources.enter_context(
            PWMOutputDevice(
                BUZZER_PIN,
                frequency=2000,
                initial_value=0,
            )
        )

        rgb = None

        if USE_RGB:
            rgb = resources.enter_context(
                RGBLED(
                    *RGB_PINS,
                    active_high=RGB_ACTIVE_HIGH,
                )
            )

        lcd = LCD1602(bus) if USE_LCD else None

        motors = []

        for pin in (PITCH_PIN, ROLL_PIN):
            motor = resources.enter_context(
                Servo(
                    pin,
                    initial_value=None,
                    min_pulse_width=0.001,
                    max_pulse_width=0.002,
                    frame_width=0.020,
                )
            )
            motors.append(motor)

        zero, bias = calibrate(bus)
        filtered = list(zero)

        print("Ctrl+C stops and releases both motors.")

        answer = input(
            "Type RUN for motors, "
            "or press Enter for sensors only: "
        )
        armed = answer.strip().upper() == "RUN"

        if armed:
            for motor in motors:
                motor.value = 0

            time.sleep(1)

        positions = [0.0, 0.0]
        errors = [0.0, 0.0]
        rates = [0.0, 0.0]

        last = time.monotonic()
        last_output = last

        candidate = ir.is_active
        candidate_since = last
        cup_present = False

        limit_since = None
        fault = ""

        try:
            while True:
                now = time.monotonic()
                dt = now - last
                last = now

                if dt > 0.25:
                    fault = "LOOP STALLED"

                dt = clamp(dt, 0.001, 0.1)

                try:
                    sample = read_imu(bus)

                except OSError:
                    fault = "IMU READ FAILED"
                    sample = None

                if sample is not None:
                    measured = angles(sample)

                    rates = [
                        sample[3] - bias[0],
                        sample[4] - bias[1],
                    ]

                    magnitude = math.sqrt(
                        sum(value * value for value in sample[:3])
                    )

                    if 0.8 < magnitude < 1.2:
                        alpha = 0.5 / (0.5 + dt)
                    else:
                        alpha = 1.0

                    for i in range(2):
                        filtered[i] = (
                            alpha * (
                                filtered[i] + rates[i] * dt
                            )
                            + (1 - alpha) * measured[i]
                        )

                        errors[i] = filtered[i] - zero[i]

                    if max(abs(value) for value in errors) > 25:
                        fault = "EXCESS TILT"

                detected = ir.is_active

                if detected != candidate:
                    candidate = detected
                    candidate_since = now

                if now - candidate_since >= 0.25:
                    cup_present = candidate

                if armed and cup_present and not fault:
                    directions = (
                        PITCH_DIRECTION,
                        ROLL_DIRECTION,
                    )

                    for i, direction in enumerate(directions):
                        positions[i] = next_position(
                            positions[i],
                            errors[i],
                            rates[i],
                            direction,
                            dt,
                        )

                        motors[i].value = positions[i] / 90.0

                    limited = any(
                        abs(positions[i]) >= LIMIT_DEG - 0.1
                        and abs(errors[i]) > 4
                        for i in range(2)
                    )

                    if limited:
                        if limit_since is None:
                            limit_since = now
                    else:
                        limit_since = None

                    if (
                        limit_since is not None
                        and now - limit_since > 0.7
                    ):
                        fault = "TRAVEL LIMIT"

                else:
                    limit_since = None

                # Cup removed: hold the last motor position.
                # Fault: release motors until the program restarts.
                if fault:
                    for motor in motors:
                        motor.detach()

                if fault and now % 1.0 < 0.2:
                    buzzer.value = 0.5
                else:
                    buzzer.value = 0

                if fault:
                    status = fault
                elif not armed:
                    status = "SENSORS ONLY"
                elif cup_present:
                    status = "LEVELING"
                else:
                    status = "WAITING FOR CUP"

                if rgb:
                    if fault:
                        rgb.color = (1, 0, 0)
                    elif armed and cup_present:
                        rgb.color = (0, 1, 0)
                    else:
                        rgb.color = (0, 0, 1)

                if now - last_output >= 0.5:
                    last_output = now

                    if sample is not None:
                        angle_line = (
                            f"P{errors[0]:+5.1f} "
                            f"R{errors[1]:+5.1f}"
                        )
                    else:
                        angle_line = "Sensor unavailable"

                    print(
                        f"{status:18} {angle_line} "
                        f"Cup={cup_present}",
                        flush=True,
                    )

                    if lcd:
                        try:
                            lcd.show(status, angle_line)

                        except OSError as exc:
                            print("LCD disabled:", exc)
                            lcd = None

                elapsed = time.monotonic() - now
                time.sleep(max(0, PERIOD - elapsed))

        finally:
            for motor in motors:
                motor.detach()

            buzzer.off()


if __name__ == "__main__":
    try:
        main()

    except KeyboardInterrupt:
        print("\nStopped.")

    except (OSError, RuntimeError) as exc:
        print("Stopped:", exc)