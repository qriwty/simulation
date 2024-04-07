import copy
import math
import time
from collections import deque
import threading
from dataclasses import dataclass


class QueuePipe:
    def __init__(self, max_size=100):
        self._queue = deque(maxlen=max_size)
        self._lock = threading.Lock()

    def size(self):
        with self._lock:
            return len(self._queue)

    def add_data(self, data):
        with self._lock:
            self._queue.append(data)

    def get_data(self, size):
        with self._lock:
            available_size = min(size, len(self._queue))

            return list(self._queue)[:available_size]

    def get_latest(self):
        with self._lock:
            try:
                return self._queue[-1]
            except IndexError:
                return None


@dataclass
class LocalPosition:
    timestamp: float
    x: float
    y: float
    z: float
    vx: float = None
    vy: float = None
    vz: float = None

    @classmethod
    def from_mavlink(cls, data):
        return cls(
            timestamp=time.time(),
            x=data.x,
            y=data.y,
            z=data.z,
            vx=data.vx,
            vy=data.vy,
            vz=data.vz
        )

    def to_array(self):
        return self.x, self.y, self.z


@dataclass
class GlobalPosition:
    timestamp: float
    latitude: int
    longitude: int
    altitude: float
    relative_altitude: float
    vx: float
    vy: float
    vz: float
    heading: float

    @classmethod
    def from_mavlink(cls, data):
        return cls(
            timestamp=time.time(),
            latitude=data.lat / 1e7,
            longitude=data.lon / 1e7,
            altitude=data.alt / 1000,
            relative_altitude=data.relative_alt / 1000,
            vx=data.vx / 100,
            vy=data.vy / 100,
            vz=data.vz / 100,
            heading=data.hdg / 100
        )


@dataclass
class Attitude:
    timestamp: float
    roll: float
    pitch: float
    yaw: float
    roll_speed: float = None
    pitch_speed: float = None
    yaw_speed: float = None

    def copy(self):
        return copy.copy(self)

    @classmethod
    def from_mavlink(cls, data):
        return cls(
            timestamp=time.time(),
            roll=data.roll,
            pitch=data.pitch,
            yaw=data.yaw,
            roll_speed=data.rollspeed,
            pitch_speed=data.pitchspeed,
            yaw_speed=data.yawspeed
        )

    def to_degrees(self):
        roll_degrees = math.degrees(self.roll)
        pitch_degrees = math.degrees(self.pitch)
        yaw_degrees = math.degrees(self.yaw)
        roll_speed_degrees = math.degrees(self.roll_speed) if self.roll_speed is not None else None
        pitch_speed_degrees = math.degrees(self.pitch_speed) if self.pitch_speed is not None else None
        yaw_speed_degrees = math.degrees(self.yaw_speed) if self.yaw_speed is not None else None

        return Attitude(
            timestamp=self.timestamp,
            roll=roll_degrees,
            pitch=pitch_degrees,
            yaw=yaw_degrees,
            roll_speed=roll_speed_degrees,
            pitch_speed=pitch_speed_degrees,
            yaw_speed=yaw_speed_degrees
        )

    def to_quaterion(self):
        quaterion = Quaternion.from_euler(
            roll=self.roll,
            pitch=self.pitch,
            yaw=self.yaw
        )

        return quaterion

    def add_offset(self, roll=0, pitch=0, yaw=0):
        self.roll += roll
        self.pitch += pitch
        self.yaw += yaw

        return self

    def transfer_offset(self, roll=0, pitch=0, yaw=0):
        new_attitude = self.copy()

        new_attitude.add_offset(
            roll=roll,
            pitch=pitch,
            yaw=yaw
        )

        return new_attitude


@dataclass
class Quaternion:
    w: float
    x: float
    y: float
    z: float

    @classmethod
    def from_euler(cls, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cy * cp * cr + sy * sp * sr
        x = cy * cp * sr - sy * sp * cr
        y = sy * cp * sr + cy * sp * cr
        z = sy * cp * cr - cy * sp * sr

        return cls(w, x, y, z)

    @classmethod
    def from_attitude(cls, attitude):
        return cls.from_euler(
            roll=attitude.roll,
            pitch=attitude.pitch,
            yaw=attitude.yaw
        )

