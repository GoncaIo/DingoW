#!/usr/bin/env python3
import rospy
import numpy as np
import time
import math
import struct
import smbus2


# ── MPU6050 Registers ─────────────────────────────────────────────────

MPU6050_ADDR = 0x68

PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG_REG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
WHO_AM_I = 0x75

GYRO_RANGE = 1
GYRO_SCALE = [131.0, 65.5, 32.8, 16.4][GYRO_RANGE]

ACCEL_RANGE = 0
ACCEL_SCALE = [16384.0, 8192.0, 4096.0, 2048.0][ACCEL_RANGE]

AXIS_MAP = [(1, 1), (0, -1), (2, 1)]


class IMU:
    def __init__(self, bus=1, address=MPU6050_ADDR, alpha=0.98):
        self.bus = smbus2.SMBus(bus)
        self.address = address
        self.alpha = alpha

        chip_id = self.bus.read_byte_data(self.address, WHO_AM_I)
        if chip_id not in (0x68, 0x98):
            raise RuntimeError(
                "MPU6050 not found at 0x{:02X} (WHO_AM_I=0x{:02X})".format(
                    address, chip_id
                )
            )

        self.bus.write_byte_data(self.address, PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        self.bus.write_byte_data(self.address, SMPLRT_DIV, 0x04)
        self.bus.write_byte_data(self.address, CONFIG_REG, 0x03)
        self.bus.write_byte_data(self.address, GYRO_CONFIG, GYRO_RANGE << 3)
        self.bus.write_byte_data(self.address, ACCEL_CONFIG, ACCEL_RANGE << 3)
        time.sleep(0.1)

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_time = time.monotonic()
        self.last_euler = np.array([0, 0, 0])

        rospy.loginfo("MPU6050 initialized at 0x{:02X}".format(address))

    def _read_raw(self, reg):
        data = self.bus.read_i2c_block_data(self.address, reg, 2)
        return struct.unpack(">h", bytes(data))[0]

    def _read_all(self):
        data = self.bus.read_i2c_block_data(self.address, ACCEL_XOUT_H, 14)
        vals = struct.unpack(">hhhhhhh", bytes(data))
        accel_raw = (vals[0] / ACCEL_SCALE, vals[1] / ACCEL_SCALE, vals[2] / ACCEL_SCALE)
        gyro_raw = (vals[4] / GYRO_SCALE, vals[5] / GYRO_SCALE, vals[6] / GYRO_SCALE)
        # Remap sensor axes to body axes (same rotation for accel and gyro)
        accel = tuple(sign * accel_raw[i] for i, sign in AXIS_MAP)
        gyro = tuple(sign * gyro_raw[i] for i, sign in AXIS_MAP)
        return accel, gyro

    def read_orientation(self):
        """Reads accelerometer and gyroscope data from the MPU6050 and"""
        try:
            now = time.monotonic()
            dt = now - self.last_time
            self.last_time = now

            if dt <= 0 or dt > 1.0:
                return self.last_euler.tolist()

            (ax, ay, az), (gx, gy, gz) = self._read_all()

            accel_roll = math.atan2(ay, math.sqrt(ax * ax + az * az))
            accel_pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))

            gx_rad = math.radians(gx)
            gy_rad = math.radians(gy)
            gz_rad = math.radians(gz)

            self.roll = self.alpha * (self.roll + gx_rad * dt) + (1 - self.alpha) * accel_roll
            self.pitch = self.alpha * (self.pitch + gy_rad * dt) + (1 - self.alpha) * accel_pitch
            self.yaw += gz_rad * dt

            self.last_euler = np.array([self.yaw, self.pitch, self.roll])
        except Exception:
            self.last_euler = np.array([0, 0, 0])
        return self.last_euler.tolist()
