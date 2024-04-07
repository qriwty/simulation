import math
import time
import re
import copy
from collections import deque
import threading
from typing import Type, Any

import numpy
from abc import ABC, abstractmethod
from enum import Enum

from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
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

    def get_data(self):
        with self._lock:
            return list(self._queue)

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


class LocalPositionProcessor:
    def __init__(self, queue):
        self.queue = queue

    def add_data(self, data):
        format_data = LocalPosition.from_mavlink(data)
        self.queue.add_data(format_data)

    def simple_extrapolation(self, target_timestamp):
        position = self.queue.get_latest()

        if position is None:
            return None

        time_delta = target_timestamp - position.timestamp

        x = position.x + time_delta * position.vx
        y = position.y + time_delta * position.vy
        z = position.z + time_delta * position.vz

        local_position = LocalPosition(
            timestamp=target_timestamp,
            x=x,
            y=y,
            z=z
        )

        return local_position

    def extrapolate(self, target_timestamp):
        current_data = self.queue.get_data()

        if current_data is None or len(current_data) < 1:
            return None

        timestamps = []
        x_values = []
        y_values = []
        z_values = []
        for position in current_data:
            timestamps.append(position.timestamp)
            x_values.append(position.x)
            y_values.append(position.y)
            z_values.append(position.z)

        x = numpy.interp(target_timestamp, timestamps, x_values)
        y = numpy.interp(target_timestamp, timestamps, y_values)
        z = numpy.interp(target_timestamp, timestamps, z_values)

        current_position = LocalPosition(
            timestamp=target_timestamp,
            x=x,
            y=y,
            z=z
        )

        return current_position


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


class GlobalPositionProcessor:
    def __init__(self, queue):
        self.queue = queue

    def add_data(self, data):
        format_data = GlobalPosition.from_mavlink(data)
        self.queue.add_data(format_data)


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


class AttitudeProcessor:
    def __init__(self, queue):
        self.queue = queue

    def add_data(self, data):
        format_data = Attitude.from_mavlink(data)
        self.queue.add_data(format_data)

    def simple_extrapolation(self, target_timestamp):
        attitude = self.queue.get_latest()

        if attitude is None:
            return None

        time_delta = target_timestamp - attitude.timestamp

        roll = attitude.roll + time_delta * attitude.roll_speed
        pitch = attitude.pitch + time_delta * attitude.pitch_speed
        yaw = attitude.yaw + time_delta * attitude.yaw_speed

        current_attitude = Attitude(
            timestamp=target_timestamp,
            roll=roll,
            pitch=pitch,
            yaw=yaw
        )

        return current_attitude

    def extrapolate(self, target_timestamp):
        current_data = self.queue.get_data()

        if current_data is None or len(current_data) < 1:
            return None

        timestamps = []
        roll_values = []
        pitch_values = []
        yaw_values = []
        for attitude in current_data:
            timestamps.append(attitude.timestamp)
            roll_values.append(attitude.roll)
            pitch_values.append(attitude.pitch)
            yaw_values.append(attitude.yaw)

        roll = numpy.interp(target_timestamp, timestamps, roll_values)
        pitch = numpy.interp(target_timestamp, timestamps, pitch_values)
        yaw = numpy.interp(target_timestamp, timestamps, yaw_values)

        current_attitude = Attitude(
            timestamp=target_timestamp,
            roll=roll,
            pitch=pitch,
            yaw=yaw
        )

        return current_attitude


@dataclass
class SpatialSnapshot:
    timestamp: float
    attitude: Attitude
    target_attitude: Attitude
    position: LocalPosition


class MAVLinkHandler:
    def __init__(self, device):
        self.connection = self.create_connection(device)
        self.boot_time = 0
        self.lock = threading.Lock()

    def create_connection(self, url):
        self.connection = mavutil.mavlink_connection(url)
        self.connection.wait_heartbeat()
        self.boot_time = time.time()

        return self.connection

    def send_attitude(self, attitude):
        attitude_quaterion = QuaternionBase([attitude.roll, attitude.pitch, attitude.yaw])

        byte_mask = 0b00000111

        composed_attitude = self.connection.mav.set_attitude_target_encode(
            0,
            self.connection.target_system,
            self.connection.target_component,
            byte_mask,
            attitude_quaterion,
            0, 0, 0, thrust=0.5
        )

        self.send_packet(composed_attitude)

    def send_control(self, control_values):
        composed_control = self.connection.mav.rc_channels_override_encode(
            self.connection.target_system,
            self.connection.target_component,
            *control_values
        )

        self.send_packet(composed_control)

    def send_packet(self, message):
        with self.lock:
            self.connection.mav.send(message)

    def receive_packet(self, packet_type):
        with self.lock:
            message = self.connection.recv_match(type=packet_type, blocking=True)

            return message


class DataAcquisitionThread(threading.Thread):
    def __init__(self, mavlink_handler, data_type, data_processor):
        super().__init__()
        self.mavlink_handler = mavlink_handler
        self.data_type = data_type
        self.data_processor = data_processor

    def run(self):
        while True:
            message = self.mavlink_handler.receive_packet(self.data_type)

            self.data_processor.add_data(message)


class ControlChannel(ABC):
    def __init__(self, id: int, name: str, value: int):
        self.id = id
        self.name = name
        self.value = value

    @abstractmethod
    def update_value(self, raw_value: float) -> None:
        pass

    @abstractmethod
    def get_value(self) -> Any:
        pass


class Axis(ControlChannel, ABC):
    def __init__(self, channel_id: int, name: str, value: int, value_range: Type[Enum]):
        super().__init__(channel_id, name, 0)
        self.value_range = value_range
        self.update_value(value)

    def update_value(self, raw_value: int) -> None:
        if self.value_range.MIN.value <= raw_value <= self.value_range.MAX.value:
            self.value = int(raw_value)
        else:
            closest_value = min(self.value_range, key=lambda x: abs(x.value - raw_value))

            self.value = int(closest_value.value)

    def get_value(self) -> int:
        return self.value


class Switch(ControlChannel, ABC):
    def __init__(self, channel_id: int, name: str, value: int, value_range: Type[Enum]):
        super().__init__(channel_id, name, 0)
        self.value_range = value_range
        self.update_value(value)

    def update_value(self, raw_value: float) -> None:
        closest_value = min(self.value_range, key=lambda x: abs(x.value - raw_value))

        self.value = closest_value

    def get_value(self) -> Type[Enum]:
        return self.value


class AxisRange(Enum):
    MIN = 1000
    MAX = 2000


class ToggleSwitch(Enum):
    UP = 1000
    DOWN = 2000


class TernarySwitch(Enum):
    UP = 1000
    MIDDLE = 1500
    DOWN = 2000


class ChannelProcessor:
    def __init__(self, *watch_channel):
        self.channels = {}

        for channel in watch_channel:
            if isinstance(channel, ControlChannel):
                self.channels[channel.id] = channel
            else:
                raise ValueError(f"Invalid channel type: {type(channel)}")

    def update_channel(self, key, value):
        if key in self.channels:
            self.channels[key].update_value(value)

    def add_data(self, mavlink_message):
        pattern = r'chan(\d+)_raw'

        message_dict = mavlink_message.__dict__

        for key, value in message_dict.items():
            if re.match(pattern, key):
                key_id = int(re.match(pattern, key).group(1))
                self.update_channel(key_id, value)


mavlink_connection = MAVLinkHandler("udp:0.0.0.0:14551")

print("CONNECTED")

# position_pipe = QueuePipe()
# attitude_pipe = QueuePipe()
#
# position_processor = LocalPositionProcessor(position_pipe)
# attitude_processor = AttitudeProcessor(attitude_pipe)
#
# position_acquisition_thread = DataAcquisitionThread(mavlink_connection, "LOCAL_POSITION_NED", position_processor)
# attitude_acquisition_thread = DataAcquisitionThread(mavlink_connection, "ATTITUDE", attitude_processor)
#
# position_acquisition_thread.start()
# attitude_acquisition_thread.start()
