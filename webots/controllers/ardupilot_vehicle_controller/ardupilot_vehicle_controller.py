#!/usr/bin/env python3
import time
import toml
from webots_vehicle import WebotsArduVehicle


configuration_file = "arducopter_configuration.toml"
configuration = toml.load(configuration_file)


def get_configuration_value(section, key, default=None):
    return configuration.get(section, {}).get(key, default)


motor_names = get_configuration_value("powerplant", "motors")
motor_cap = get_configuration_value("powerplant", "motor_cap", float("inf"))
reversed_motors = get_configuration_value("powerplant", "reversed", False)
bidirectional_motors = get_configuration_value("powerplant", "bidirectional", False)
uses_propellers = get_configuration_value("powerplant", "uses_propellers", True)

accel_name = get_configuration_value("sensors", "accel", "accelerometer")
imu_name = get_configuration_value("sensors", "imu", "inertial unit")
gyro_name = get_configuration_value("sensors", "gyro", "gyro")
gps_name = get_configuration_value("sensors", "gps", "gps")

gimbal_roll = get_configuration_value("gimbal", "roll", "camera roll")
gimbal_pitch = get_configuration_value("gimbal", "pitch", "camera roll")
gimbal_yaw = get_configuration_value("gimbal", "yaw", "camera roll")

gimbal_roll_sensor = get_configuration_value("gimbal", "roll_sensor", "camera roll sensor")
gimbal_pitch_sensor = get_configuration_value("gimbal", "pitch_sensor", "camera roll sensor")
gimbal_yaw_sensor = get_configuration_value("gimbal", "yaw_sensor", "camera roll sensor")

camera_name = get_configuration_value("camera", "name", "")
camera_mode = get_configuration_value("camera", "mode", "color")
camera_fps = get_configuration_value("camera", "fps", 30)
camera_host = get_configuration_value("camera", "host", "localhost")
camera_port = get_configuration_value("camera", "port", None)

rangefinder_name = get_configuration_value("rangefinder", "name", "")
rangefinder_fps = get_configuration_value("rangefinder", "fps", 30)
rangefinder_host = get_configuration_value("rangefinder", "host", "localhost")
rangefinder_port = get_configuration_value("rangefinder", "port", None)

instance = get_configuration_value("sitl", "instance", 0)
sitl_address = get_configuration_value("sitl", "sitl_address", "127.0.0.1")


if __name__ == "__main__":
    vehicle = WebotsArduVehicle(
        motor_names=motor_names,
        reversed_motors=reversed_motors,
        accel_name=accel_name,
        imu_name=imu_name,
        gyro_name=gyro_name,
        gps_name=gps_name,
        gimbal_roll=gimbal_roll,
        gimbal_pitch=gimbal_pitch,
        gimbal_yaw=gimbal_yaw,
        gimbal_roll_sensor=gimbal_roll_sensor,
        gimbal_pitch_sensor=gimbal_pitch_sensor,
        gimbal_yaw_sensor=gimbal_yaw_sensor,
        camera_name=camera_name,
        camera_fps=camera_fps,
        camera_stream_host=camera_host,
        camera_stream_port=camera_port,
        camera_mode=camera_mode,
        rangefinder_name=rangefinder_name,
        rangefinder_fps=rangefinder_fps,
        rangefinder_stream_host=rangefinder_host,
        rangefinder_stream_port=rangefinder_port,
        instance=instance,
        motor_velocity_cap=motor_cap,
        bidirectional_motors=bidirectional_motors,
        uses_propellers=uses_propellers,
        sitl_address=sitl_address
    )

    while vehicle.webots_connected():
        time.sleep(1)
