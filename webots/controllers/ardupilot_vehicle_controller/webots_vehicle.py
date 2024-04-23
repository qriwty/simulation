'''
This file implements a class that acts as a bridge between ArduPilot SITL and Webots

AP_FLAKE8_CLEAN
'''
import base64
# Imports
import os
import sys
import time
import socket
import select
import struct
import numpy as np
import zmq
from threading import Thread
from typing import List

from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase

# Here we set up environment variables so we can run this script
# as an external controller outside of Webots (useful for debugging)
# https://cyberbotics.com/doc/guide/running-extern-robot-controllers
if sys.platform.startswith("win"):
    WEBOTS_HOME = "C:\\Program Files\\Webots"
elif sys.platform.startswith("darwin"):
    WEBOTS_HOME = "/Applications/Webots.app"
elif sys.platform.startswith("linux"):
    WEBOTS_HOME = "/usr/local/webots"
else:
    raise Exception("Unsupported OS")

if os.environ.get("WEBOTS_HOME") is None:
    os.environ["WEBOTS_HOME"] = WEBOTS_HOME
else:
    WEBOTS_HOME = os.environ.get("WEBOTS_HOME")

os.environ["PYTHONIOENCODING"] = "UTF-8"
sys.path.append(f"{WEBOTS_HOME}/lib/controller/python")

from controller import Robot, Camera, RangeFinder
from drone_data import RangefinderData, CameraData, FDMData, GimbalAxisData, GimbalData, DroneData
from dataclasses import dataclass, asdict, is_dataclass
import json


class WebotsArduVehicle:
    """Class representing an ArduPilot controlled Webots Vehicle"""

    controls_struct_format = 'f'*16
    controls_struct_size = struct.calcsize(controls_struct_format)
    fdm_struct_format = 'd'*(1+3+3+3+3+3)
    fdm_struct_size = struct.calcsize(fdm_struct_format)

    def __init__(self,
                 motor_names: List[str],
                 accel_name: str,
                 imu_name: str,
                 gyro_name: str,
                 gps_name: str,
                 gimbal_roll: str,
                 gimbal_pitch: str,
                 gimbal_yaw: str,
                 color_camera_name: str,
                 depth_camera_name: str,
                 rangefinder_name: str,
                 data_stream_fps: int = 30,
                 data_stream_port: int = 5588,
                 instance: int = 0,
                 sitl_address: str = "127.0.0.1"):
        # init class variables
        self.motor_velocity_cap = float("inf")
        self._instance = instance
        self._webots_connected = True

        self.robot = Robot()

        # set robot time step relative to sim time step
        self._timestep = int(self.robot.getBasicTimeStep())

        # init sensors
        self.accel = self.robot.getDevice(accel_name)
        self.imu = self.robot.getDevice(imu_name)
        self.gyro = self.robot.getDevice(gyro_name)
        self.gps = self.robot.getDevice(gps_name)

        self.accel.enable(self._timestep)
        self.imu.enable(self._timestep)
        self.gyro.enable(self._timestep)
        self.gps.enable(self._timestep)

        self.gimbal_roll = self.robot.getDevice(gimbal_roll)
        self.gimbal_pitch = self.robot.getDevice(gimbal_pitch)
        self.gimbal_yaw = self.robot.getDevice(gimbal_yaw)

        self.gimbal_roll.getPositionSensor().enable(self._timestep)
        self.gimbal_pitch.getPositionSensor().enable(self._timestep)
        self.gimbal_yaw.getPositionSensor().enable(self._timestep)

        self.gimbal_roll.setPosition(0)
        self.gimbal_pitch.setPosition(0)
        self.gimbal_yaw.setPosition(0)

        self.color_camera = self.robot.getDevice(color_camera_name)
        self.color_camera.enable(1000 // data_stream_fps)  # takes frame period in ms

        self.depth_camera = self.robot.getDevice(depth_camera_name)
        self.depth_camera.enable(1000 // data_stream_fps)

        self.rangefinder = self.robot.getDevice(rangefinder_name)
        self.rangefinder.enable(1000 // data_stream_fps)

        self._data_thread = Thread(daemon=True,
                                   target=self._handle_stream,
                                   args=[data_stream_port])
        self._data_thread.start()

        self._camera_data_type = "uint8"
        self._rangefinder_data_type = "uint16"

        # init motors (and setup velocity control)
        self._motors = [self.robot.getDevice(n) for n in motor_names]
        for motor in self._motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(0)

        # start ArduPilot SITL communication thread
        self._sitl_thread = Thread(daemon=True, target=self._handle_sitl, args=[sitl_address, 9002+10*instance])
        self._sitl_thread.start()

        self._sitl_gimbal_thread = Thread(daemon=True, target=self._handle_sitl_gimbal, args=[sitl_address, 14551])
        self._sitl_gimbal_thread.start()

    def clamp(self, value, minimum, maximum):
        return max(minimum, min(value, maximum))

    def _handle_sitl_gimbal(self, sitl_address: str = "127.0.0.1", port: int = 14551):
        connection = mavutil.mavlink_connection(f"{sitl_address}:{port}")

        while True:
            gimbal_message = connection.recv_match(type="GIMBAL_DEVICE_ATTITUDE_STATUS", blocking=True)

            gimbal_target_position = QuaternionBase(gimbal_message.q).euler
            target_roll, target_pitch, target_yaw = gimbal_target_position
            target_pitch = -target_pitch
            target_yaw = -target_yaw

            roll_value = self.clamp(
                value=target_roll,
                minimum=self.gimbal_roll.getMinPosition(),
                maximum=self.gimbal_roll.getMaxPosition()
            )
            pitch_value = self.clamp(
                value=target_pitch,
                minimum=self.gimbal_pitch.getMinPosition(),
                maximum=self.gimbal_pitch.getMaxPosition()
            )
            yaw_value = self.clamp(
                value=target_yaw,
                minimum=self.gimbal_yaw.getMinPosition(),
                maximum=self.gimbal_yaw.getMaxPosition()
            )

            self.gimbal_roll.setPosition(roll_value)
            self.gimbal_pitch.setPosition(pitch_value)
            self.gimbal_yaw.setPosition(yaw_value)

    def _handle_sitl(self, sitl_address: str = "127.0.0.1", port: int = 9002):
        """Handles all communications with the ArduPilot SITL

        Args:
            port (int, optional): Port to listen for SITL on. Defaults to 9002.
        """

        # create a local UDP socket server to listen for SITL
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # SOCK_STREAM
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((sitl_address, port))

        # wait for SITL to connect
        print(f"Listening for ardupilot SITL (I-{self._instance}) at {sitl_address}:{port}")
        self.robot.step(self._timestep) # flush print in webots console

        while not select.select([s], [], [], 0)[0]: # wait for socket to be readable
            # if webots is closed, close the socket and exit
            if self.robot.step(self._timestep) == -1:
                s.close()
                self._webots_connected = False
                return

        print(f"Connected to ardupilot SITL (I-{self._instance})")

        # main loop handling communications
        while True:
            # check if the socket is ready to send/receive
            readable, writable, _ = select.select([s], [s], [], 0)

            # send data to SITL port (one lower than its output port as seen in SITL_cmdline.cpp)
            if writable:
                fdm_struct = self._get_fdm_struct()
                s.sendto(fdm_struct, (sitl_address, port+1))

            # receive data from SITL port
            if readable:
                data = s.recv(512)
                if not data or len(data) < self.controls_struct_size:
                    continue

                # parse a single struct
                command = struct.unpack(self.controls_struct_format, data[:self.controls_struct_size])
                self._handle_controls(command)

                # wait until the next Webots time step as no new sensor data will be available until then
                step_success = self.robot.step(self._timestep)
                if step_success == -1: # webots closed
                    break

        # if we leave the main loop then Webots must have closed
        s.close()
        self._webots_connected = False
        print(f"Lost connection to Webots (I-{self._instance})")

    def _get_fdm_struct(self) -> bytes:
        """Form the Flight Dynamics Model struct (aka sensor data) to send to the SITL

        Returns:
            bytes: bytes representing the struct to send to SITL
        """
        # get data from Webots
        i = self.imu.getRollPitchYaw()
        g = self.gyro.getValues()
        a = self.accel.getValues()
        gps_pos = self.gps.getValues()
        gps_vel = self.gps.getSpeedVector()

        # pack the struct, converting ENU to NED (ish)
        # https://discuss.ardupilot.org/t/copter-x-y-z-which-is-which/6823/3
        # struct fdm_packet {
        #     double timestamp;
        #     double imu_angular_velocity_rpy[3];
        #     double imu_linear_acceleration_xyz[3];
        #     double imu_orientation_rpy[3];
        #     double velocity_xyz[3];
        #     double position_xyz[3];
        # };
        return struct.pack(self.fdm_struct_format,
                           self.robot.getTime(),
                           g[0], -g[1], -g[2],
                           a[0], -a[1], -a[2],
                           i[0], -i[1], -i[2],
                           gps_vel[0], -gps_vel[1], -gps_vel[2],
                           gps_pos[0], -gps_pos[1], -gps_pos[2])

    def _handle_controls(self, command: tuple):
        """Set the motor speeds based on the SITL command

        Args:
            command (tuple): tuple of motor speeds 0.0-1.0 where -1.0 is unused
        """

        # get only the number of motors we have
        command_motors = command[:len(self._motors)]
        if -1 in command_motors:
            print(f"Warning: SITL provided {command.index(-1)} motors "
                  f"but model specifies {len(self._motors)} (I-{self._instance})")

        linearized_motor_commands = [np.sqrt(np.abs(v))*np.sign(v) for v in command_motors]

        # set velocities of the motors in Webots
        for i, m in enumerate(self._motors):
            m.setVelocity(linearized_motor_commands[i] * min(m.getMaxVelocity(), self.motor_velocity_cap))

    def _get_drone_data(self):
        fdm_data = FDMData.create(
            timestamp=self.robot.getTime(),
            imu=self.imu.getRollPitchYaw(),
            gyroscope=self.gyro.getValues(),
            accelerometer=self.accel.getValues(),
            gps_position=self.gps.getValues(),
            gps_velocity=self.gps.getSpeedVector()
        )

        gimbal_data = GimbalData(
            timestamp=self.robot.getTime(),
            roll=GimbalAxisData(
                min=self.gimbal_roll.getMinPosition(),
                max=self.gimbal_roll.getMaxPosition(),
                current=self.gimbal_roll.getPositionSensor().getValue(),
                target=self.gimbal_roll.getTargetPosition()
            ),
            pitch=GimbalAxisData(
                min=self.gimbal_pitch.getMinPosition(),
                max=self.gimbal_pitch.getMaxPosition(),
                current=self.gimbal_roll.getPositionSensor().getValue(),
                target=self.gimbal_pitch.getTargetPosition()
            ),
            yaw=GimbalAxisData(
                min=self.gimbal_yaw.getMinPosition(),
                max=self.gimbal_yaw.getMaxPosition(),
                current=self.gimbal_roll.getPositionSensor().getValue(),
                target=self.gimbal_yaw.getTargetPosition()
            )
        )

        camera_data = CameraData(
            timestamp=self.robot.getTime(),
            width=self.color_camera.getWidth(),
            height=self.color_camera.getHeight(),
            fps=1000 / self.color_camera.getSamplingPeriod(),
            fov=self.color_camera.getFov(),
            data_type=self._camera_data_type,
            frame=self.get_camera_frame()
        )

        depth_data = RangefinderData(
            timestamp=self.robot.getTime(),
            width=self.depth_camera.getWidth(),
            height=self.depth_camera.getHeight(),
            fps=1000 / self.depth_camera.getSamplingPeriod(),
            fov=self.depth_camera.getFov(),
            min_range=self.depth_camera.getMinRange(),
            max_range=self.depth_camera.getMaxRange(),
            data_type=self._rangefinder_data_type,
            frame=self.get_rangefinder_frame(self.depth_camera)
        )

        rangefinder_data = RangefinderData(
            timestamp=self.robot.getTime(),
            width=self.rangefinder.getWidth(),
            height=self.rangefinder.getHeight(),
            fps=1000 / self.rangefinder.getSamplingPeriod(),
            fov=self.rangefinder.getFov(),
            min_range=self.rangefinder.getMinRange(),
            max_range=self.rangefinder.getMaxRange(),
            data_type=self._rangefinder_data_type,
            frame=self.get_rangefinder_frame(self.rangefinder)
        )

        drone_data = DroneData(
            timestamp=self.robot.getTime(),
            fdm=fdm_data,
            gimbal=gimbal_data,
            camera=camera_data,
            depth=depth_data,
            rangefinder=rangefinder_data
        )

        return drone_data

    def _handle_stream(self, port: int):
        context = zmq.Context()
        socket = context.socket(zmq.REP)
        socket.bind(f"tcp://*:{port}")

        while self._webots_connected:
            start_time = self.robot.getTime()

            message = socket.recv_string()
            if message == "get_data":
                data = self._get_drone_data()
                serialized_data = data.to_json()
                socket.send_json(serialized_data)

            # delay at sample rate
            while self.robot.getTime() - start_time < self.color_camera.getSamplingPeriod() / 1000:
                time.sleep(0.001)

    def get_camera_frame(self) -> np.ndarray:
        """Get the RGB image from the camera as a numpy array of bytes"""
        img = self.color_camera.getImage()

        if self._camera_data_type == "uint8":
            data_type = np.uint8
        elif self._camera_data_type == "uint16":
            data_type = np.float32
        else:
            raise ValueError("Unsupported data type specified")

        img = np.frombuffer(img, data_type).reshape((self.color_camera.getHeight(), self.color_camera.getWidth(), 4))
        return img[:, :, :3] # RGB only, no Alpha

    def get_rangefinder_frame(self, rangefinder) -> np.ndarray:
        """Get the rangefinder depth image as a numpy array of int8 or int16"""\

        # get range image size
        height = rangefinder.getHeight()
        width = rangefinder.getWidth()

        # get image, and convert raw ctypes array to numpy array
        # https://cyberbotics.com/doc/reference/rangefinder
        image_c_ptr = rangefinder.getRangeImage(data_type="buffer")
        img_arr = np.ctypeslib.as_array(image_c_ptr, (width*height,))
        img_floats = img_arr.reshape((height, width))

        # normalize and set unknown values to max range
        range_range = rangefinder.getMaxRange() - rangefinder.getMinRange()
        img_normalized = (img_floats - rangefinder.getMinRange()) / range_range
        img_normalized[img_normalized == float('inf')] = 1

        # convert to int8 or int16, allowing for the option of higher precision if desired
        if self._rangefinder_data_type == "uint8":
            img = (img_normalized * 255).astype(np.uint8)
        elif self._rangefinder_data_type == "uint16":
            img = (img_normalized * 65535).astype(np.uint16)
        else:
            raise ValueError("Unsupported data type specified")

        return img

    def stop_motors(self):
        """Set all motors to zero velocity"""
        for m in self._motors:
            m.setPosition(float('inf'))
            m.setVelocity(0)

    def webots_connected(self) -> bool:
        """Check if Webots client is connected"""
        return self._webots_connected
