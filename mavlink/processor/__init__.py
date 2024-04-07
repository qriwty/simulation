from abc import ABC, abstractmethod
import numpy

from mavlink.data import QueuePipe, LocalPosition, GlobalPosition, Attitude


class DataProcessor(ABC):
    def __init__(self, queue: QueuePipe):
        self.queue = queue

    @abstractmethod
    def add_data(self, data):
        pass


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

    def extrapolate(self, target_timestamp, elements=100):
        current_data = self.queue.get_data(size=elements)

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


class GlobalPositionProcessor:
    def __init__(self, queue):
        self.queue = queue

    def add_data(self, data):
        format_data = GlobalPosition.from_mavlink(data)
        self.queue.add_data(format_data)


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

    def extrapolate(self, target_timestamp, elements=100):
        current_data = self.queue.get_data(size=elements)

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
