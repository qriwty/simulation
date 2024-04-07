import time
import threading

from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase


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
