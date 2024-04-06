#!/usr/bin/env python3

import tomllib
from webots_vehicle import WebotsArduVehicle


with open("arducopter_configuration.toml", "rb") as configuration_file:
    configuration = tomllib.load(configuration_file)


motor_names = configuration["powerplant"]["names"].split(", ")
motor_cap = float(configuration["powerplant"]["motor_cap"]) if configuration["powerplant"]["motor_cap"] != "inf" else float("inf")
reversed_motors = configuration["powerplant"]["reversed"]
bidirectional_motors = configuration["powerplant"]["bidirectional"]
uses_propellers = configuration["powerplant"]["uses_propellers"]

accel_name = configuration["sensors"]["accel"]
imu_name = configuration["sensors"]["imu"]
gyro_name = configuration["sensors"]["gyro"]
gps_name = configuration["sensors"]["gps"]

camera_name = configuration["camera"]["name"]
camera_mode = configuration["camera"]["mode"]
camera_fps = configuration["camera"]["fps"]
camera_host = configuration["camera"]["host"]
camera_port = configuration["camera"]["port"]

rangefinder_name = configuration["rangefinder"]["name"]
rangefinder_fps = configuration["rangefinder"]["fps"]
rangefinder_host = configuration["rangefinder"]["host"]
rangefinder_port = configuration["rangefinder"]["port"]

instance = configuration["sitl"]["instance"]
sitl_address = configuration["sitl"]["sitl_address"]


vehicle = WebotsArduVehicle(
    motor_names=motor_names,
    reversed_motors=reversed_motors,
    accel_name=accel_name,
    imu_name=imu_name,
    gyro_name=gyro_name,
    gps_name=gps_name,
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
