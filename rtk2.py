# pylint: disable=broad-except

from sys import platform
from io import BufferedReader
from threading import Thread, Lock, Event
from queue import Queue
from time import sleep
from serial import Serial
from pyubx2 import (
    UBXReader,
    NMEA_PROTOCOL,
    UBX_PROTOCOL,
    RTCM3_PROTOCOL,
    protocol,
)
from pyrtcm import RTCM_MSGIDS
from pygnssutils import GNSSNTRIPClient, VERBOSITY_LOW

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import utm

class GPS_Publisher(Node):  
	def __init__(self):
		super().__init__('GPS')
		self.publisher_gnss = self.create_publisher(Pose, '/pose', 1)


# Set to True to print entire GNSS/NTRIP message rather than just identity
PRINT_FULL = False
rclpy.init(args=None)
GPS = GPS_Publisher()

def read_gnss(stream, lock, stopevent):
    """
    THREADED
    Reads and parses incoming GNSS data from receiver.
    """

    ubr = UBXReader(
        BufferedReader(serial),
        protfilter=(NMEA_PROTOCOL | UBX_PROTOCOL | RTCM3_PROTOCOL),
    )
    pose = Pose()
    
    while not stopevent.is_set():
        try:
            if stream.in_waiting:
                lock.acquire()
                (raw_data, parsed_data) = ubr.read()  # pylint: disable=unused-variable
                lock.release()
                if parsed_data:
                    idy = parsed_data.identity
                    # if it's an RXM-RTCM message, show which RTCM3 message
                    # it's acknowledging and whether it's been used or not.""
                    if idy == "RXM-RTCM":
                        nty = (
                            f" - {parsed_data.msgType} "
                            f"{'Used' if parsed_data.msgUsed > 0 else 'Not used'}"
                        )
                    elif idy == "NAV-POSLLH":
                       print("POS>>", parsed_data.lat, parsed_data.lon)
                       pose.position.x, pose.position.y =  utm.from_latlon(parsed_data.lat, parsed_data.lon)[0:2]
                       GPS.publisher_gnss.publish(pose)
                       nty = ""
                       
                    else:
                        nty = ""
                    if PRINT_FULL:
                        print(parsed_data)
                    else:
                        print(f"GNSS>> {idy}{nty}")
        except Exception as err:
            print(f"Something went wrong in read thread {err}")
            break


def send_gnss(stream, lock, stopevent, inqueue):
    """
    THREADED
    Reads RTCM3 data from message queue and sends it to receiver.
    """

    while not stopevent.is_set():
        try:
            raw_data, parsed_data = inqueue.get()
            if protocol(raw_data) == RTCM3_PROTOCOL:
                if PRINT_FULL:
                    print(parsed_data)
                else:
                    print(
                        f"NTRIP>> {parsed_data.identity} {RTCM_MSGIDS[parsed_data.identity]}"
                    )
                lock.acquire()
                stream.write(raw_data)
                lock.release()
        except Exception as err:
            print(f"Something went wrong in send thread {err}")
            break


def start_read_thread(stream, lock, stopevent):
    """
    Start read thread.
    """

    rth = Thread(
        target=read_gnss,
        args=(
            stream,
            lock,
            stopevent,
        ),
        daemon=True,
    )
    rth.start()
    return rth


def start_send_thread(stream, lock, stopevent, msgqueue):
    """
    Start send thread.
    """

    kth = Thread(
        target=send_gnss,
        args=(
            stream,
            lock,
            stopevent,
            msgqueue,
        ),
        daemon=True,
    )
    kth.start()
    return kth


if __name__ == "__main__":
    # pylint: disable=invalid-name

    # GNSS receiver serial port parameters - AMEND AS REQUIRED:
    if platform == "win32":  # Windows
        SERIAL_PORT = "COM13"
    elif platform == "darwin":  # MacOS
        SERIAL_PORT = "/dev/tty.usbmodem141101"
    else:  # Linux
        SERIAL_PORT = "/dev/ttyGPS"
    BAUDRATE = 9600
    TIMEOUT = 0.1

    # NTRIP server parameters - AMEND AS REQUIRED:
    # Ideally, mountpoint should be <30 km from location.
    NTRIP_SERVER = "192.168.1.128"
    NTRIP_PORT = 2101
    MOUNTPOINT = "U-BLOX"
    NTRIP_USER = "labor"
    NTRIP_PASSWORD = "labor"
    REFLAT = 0
    REFLON = 0
    REFALT = 0
    REFSEP = 0
    GGAINT = 10  # -1 = do not send NMEA GGA sentences

    serial_lock = Lock()
    ntrip_queue = Queue()
    stop = Event()
    
    try:

        print(f"Opening serial port {SERIAL_PORT} @ {BAUDRATE}...\n")
        with Serial(SERIAL_PORT, BAUDRATE, timeout=TIMEOUT) as serial:

            stop.clear()
            print("Starting read thread...\n")
            gnss_thread = start_read_thread(serial, serial_lock, stop)

            print("Starting send thread...\n")
            rtk_thread = start_send_thread(serial, serial_lock, stop, ntrip_queue)

            print(f"Starting NTRIP client on {NTRIP_SERVER}:{NTRIP_PORT}...\n")
            with GNSSNTRIPClient(None, verbosity=VERBOSITY_LOW) as gnc:

                streaming = gnc.run(
                    server=NTRIP_SERVER,
                    mountpoint=MOUNTPOINT,
                    user=NTRIP_USER,
                    password=NTRIP_PASSWORD,
                    reflat=REFLAT,
                    reflon=REFLON,
                    refalt=REFALT,
                    refsep=REFSEP,
                    ggainterval=GGAINT,
                    output=ntrip_queue,
                )

                while streaming and not stop.is_set():  # run until user presses CTRL-C
                    sleep(1)
                sleep(1)

    except KeyboardInterrupt:
        stop.set()

    print("Terminated by user")

