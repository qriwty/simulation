import base64
import json
from dataclasses import dataclass, asdict, field
from typing import List

import numpy
import cv2


@dataclass
class RangefinderData:
    timestamp: int
    width: int
    height: int
    fps: float
    fov: float
    min_range: float
    max_range: float
    data_type: str
    frame: numpy.ndarray = field(repr=False)

    def encode_frame(self):
        _, buffer = cv2.imencode(".jpg", self.frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        encoded_frame = base64.b64encode(buffer).decode("utf-8")

        return encoded_frame

    def decode_frame(self, frame):
        image_decoded = base64.b64decode(frame)

        if self.data_type == "uint8":
            data_type = numpy.uint8
        elif self.data_type == "uint16":
            data_type = numpy.uint16
        else:
            raise ValueError("Unsupported data type specified")

        frame_array = numpy.frombuffer(image_decoded, dtype=data_type)
        decoded_frame = cv2.imdecode(frame_array, cv2.IMREAD_GRAYSCALE)

        return decoded_frame

    def to_json(self):
        data = asdict(self)
        data.pop("frame")

        data["frame"] = self.encode_frame()

        return json.dumps(data)

    @classmethod
    def from_dict(cls, data):
        encoded_frame = data.pop("frame")
        instance = cls(frame=numpy.array([]), **data)
        instance.frame = instance.decode_frame(encoded_frame)

        return instance


@dataclass
class CameraData:
    timestamp: int
    width: int
    height: int
    fps: float
    fov: float
    data_type: str
    frame: numpy.ndarray = field(repr=False)

    def encode_frame(self):
        _, buffer = cv2.imencode(".jpg", self.frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        encoded_frame = base64.b64encode(buffer).decode("utf-8")

        return encoded_frame

    def decode_frame(self, frame):
        image_decoded = base64.b64decode(frame)

        if self.data_type == "uint8":
            data_type = numpy.uint8
        elif self.data_type == "uint16":
            data_type = numpy.uint16
        else:
            raise ValueError("Unsupported data type specified")

        frame_array = numpy.frombuffer(image_decoded, dtype=data_type)
        decoded_frame = cv2.imdecode(frame_array, cv2.IMREAD_COLOR)

        return decoded_frame

    def to_json(self):
        data = asdict(self)
        data.pop("frame")

        data["frame"] = self.encode_frame()

        return json.dumps(data)

    @classmethod
    def from_dict(cls, data):
        encoded_frame = data.pop("frame")
        instance = cls(frame=numpy.array([]), **data)
        instance.frame = instance.decode_frame(encoded_frame)

        return instance


@dataclass
class FDMData:
    timestamp: float
    imu_angular_velocity_rpy: List[float]
    imu_linear_acceleration_xyz: List[float]
    imu_orientation_rpy: List[float]
    velocity_xyz: List[float]
    position_xyz: List[float]

    @classmethod
    def create(cls, timestamp, imu, gyroscope, accelerometer, gps_position, gps_velocity):
        return cls(
            timestamp=timestamp,
            imu_angular_velocity_rpy=[gyroscope[0], -gyroscope[1], -gyroscope[2]],
            imu_linear_acceleration_xyz=[accelerometer[0], -accelerometer[1], -accelerometer[2]],
            imu_orientation_rpy=[imu[0], -imu[1], -imu[2]],
            velocity_xyz=[gps_velocity[0], -gps_velocity[1], -gps_velocity[2]],
            position_xyz=[gps_position[0], -gps_position[1], -gps_position[2]]
        )

    def to_json(self):
        return json.dumps(asdict(self))


@dataclass
class GimbalAxisData:
    min: float
    max: float
    current: float
    target: float

    def to_json(self):
        return json.dumps(asdict(self))


@dataclass
class GimbalData:
    timestamp: float
    roll: GimbalAxisData
    pitch: GimbalAxisData
    yaw: GimbalAxisData

    def to_json(self):
        return json.dumps({"timestamp": self.timestamp,
                           "roll": json.loads(self.roll.to_json()),
                           "pitch": json.loads(self.pitch.to_json()),
                           "yaw": json.loads(self.yaw.to_json())})


@dataclass
class DroneData:
    timestamp: float
    fdm: FDMData
    gimbal: GimbalData
    camera: CameraData
    rangefinder: RangefinderData

    def to_json(self):
        return json.dumps({"timestamp": self.timestamp,
                           "fdm": json.loads(self.fdm.to_json()),
                           "gimbal": json.loads(self.gimbal.to_json()),
                           "camera": json.loads(self.camera.to_json()),
                           "rangefinder": json.loads(self.rangefinder.to_json())})
