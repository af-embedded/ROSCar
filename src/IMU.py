import math
from .MPU9250 import MPU9250

SHUFFLE = "xyz"


class IMU:
    def __init__(self):
        """
        9-DOF Inertial Measurement Unit (IMU) sensor.

        :param accel_shuffle: The order of accelerometer readings
        :param gyro_shuffle: The order of gyroscope readings
        :param magnet_shuffle: The order of magnetometer readings
        """
        self._mpu9250 = MPU9250()
        self._accel_shuffle = SHUFFLE
        self._gyro_shuffle = SHUFFLE
        self._magnet_shuffle = SHUFFLE
        self._reading = {}
        self._g_force = 9.81
        self._deg_to_rad = math.pi / 180.0
        self._calibration = {
            'ax': 0.0,
            'ay': 0.0,
            'az': 0.0,
            'gx': 0.0,
            'gy': 0.0,
            'gz': 0.0,
            'mx': 0.0,
            'my': 0.0,
            'mz': 0.0,
            'temp': 0.0
        }

    def set_calibration(self, calibration):
        self._calibration = calibration

    def tick(self):
        accel = self._mpu9250.readAccel()
        gyro = self._mpu9250.readGyro()
        mag = self._mpu9250.readMagnet()
        temp = self._mpu9250.readTemperature()

        accel = self._multiply_by(accel, self._g_force)
        gyro = self._multiply_by(gyro, self._deg_to_rad)

        accel = self._shuffle(accel, self._accel_shuffle)
        gyro = self._shuffle(gyro, self._gyro_shuffle)
        mag = self._shuffle(mag, self._magnet_shuffle)
        self._reading = {
            'ax': accel['x'] - self._calibration['ax'],
            'ay': accel['y'] - self._calibration['ay'],
            'az': accel['z'] - self._calibration['az'],
            'gx': gyro['x'] - self._calibration['gx'],
            'gy': gyro['y'] - self._calibration['gy'],
            'gz': gyro['z'] - self._calibration['gz'],
            'mx': mag['x'],
            'my': mag['y'],
            'mz': mag['z'],
            'temp': temp
        }

    def get_reading(self):
        return self._reading

    def _shuffle(self, data):
        output = {}
        axis = 'xyz'
        for i in range(3):
            output[SHUFFLE[i]] = data[axis[i]]
        return output

    def _multiply_by(self, data, factor):
        output = {}
        for k in data:
            output[k] = data[k] * factor
        return output
