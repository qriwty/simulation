#!/usr/bin/env python3
import time
from webots_vehicle import WebotsArduVehicle


if __name__ == "__main__":
    vehicle = WebotsArduVehicle(
        motor_names=["m1_motor", "m2_motor", "m3_motor", "m4_motor"],
        accel_name="accelerometer",
        imu_name="inertial unit",
        gyro_name="gyro",
        gps_name="gps",
        gimbal_roll="camera roll",
        gimbal_pitch="camera pitch",
        gimbal_yaw="camera yaw",
        color_camera_name="color_camera",
        depth_camera_name="depth_camera",
        rangefinder_name="rangefinder",
        sitl_address="127.0.0.1",
        instance=0,
    )

    while vehicle.webots_connected():
        time.sleep(1)
