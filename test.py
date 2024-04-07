import time

from mavlink import MAVLinkHandler, DataAcquisitionThread
from mavlink.processor import LocalPositionProcessor, AttitudeProcessor


mavlink_connection = MAVLinkHandler("udp:0.0.0.0:14551")

print("CONNECTED")

position_processor = LocalPositionProcessor()
attitude_processor = AttitudeProcessor()

position_acquisition_thread = DataAcquisitionThread(mavlink_connection, "LOCAL_POSITION_NED", position_processor)
attitude_acquisition_thread = DataAcquisitionThread(mavlink_connection, "ATTITUDE", attitude_processor)

position_acquisition_thread.start()
attitude_acquisition_thread.start()


while True:
    start_time = time.time()

    for i in range(100):
        attitude, position = attitude_processor.extrapolate(time.time()), position_processor.extrapolate(time.time())

    end_time = time.time()

    execution_time = end_time - start_time

    print(f"Результат функции: \n"
          f"\t {attitude_processor.queue.size()} | {position_processor.queue.size()}\n"
          f"\t {attitude} \n"
          f"\t {position} \n"
          f"Время выполнения: {execution_time} сек.")

    time.sleep(1)
