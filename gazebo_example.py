#!/usr/bin/env python3
import time, math
import time
import threading
import numpy as np
import cv2

from gz.transport14 import Node
from gz.msgs11.actuators_pb2 import Actuators
from gz.msgs11.imu_pb2 import IMU
from gz.msgs11.magnetometer_pb2 import Magnetometer   # /X3/mag
from gz.msgs11.navsat_pb2 import NavSat               # /X3/navsat
from gz.msgs11.altimeter_pb2 import Altimeter     
from gz.msgs11.image_pb2 import Image

MOTOR_TOPIC = "/X3/gazebo/command/motor_speed"
IMU_TOPIC   = "/X3/imu"
GPS_TOPIC   = "/X3/navsat"
MAG_TOPIC   = "/X3/mag"
BARO_TOPIC   = "/X3/mag"

state = {
    "imu":  {"ax":0.0,"ay":0.0,"az":0.0,"gx":0.0,"gy":0.0,"gz":0.0, "t":0.0},
    "gps":  {"lat":0.0,"lon":0.0,"alt":0.0,"vn":0.0,"ve":0.0,"vu":0.0, "t":0.0},
    "mag":  {"mx":0.0,"my":0.0,"mz":0.0, "t":0.0},
    "baro": {"alt":0.0,"vz":0.0,"p":0.0, "t":0.0},
    "frame": None,
    "lock": threading.Lock()
}
def cam_cb(msg: Image):
    print(f"CAM: {msg.width}x{msg.height}, step={msg.step}, ts={msg.header.stamp.sec}.{msg.header.stamp.nsec:09d}")
    # If you want raw bytes: msg.data is a bytes field (RGB8)


def gps_cb(msg: NavSat):
    print(f"GPS: lat={msg.latitude_deg:.7f}, lon={msg.longitude_deg:.7f}, "
          f"alt={msg.altitude:.2f} m, vel=({msg.velocity_east:.2f}, "
          f"{msg.velocity_north:.2f}, {msg.velocity_up:.2f}) m/s")

def mag_cb(msg: Magnetometer):
    print(f"MAG: field=({msg.field_tesla.x:.6f}, "
          f"{msg.field_tesla.y:.6f}, {msg.field_tesla.z:.6f}) T")

def imu_cb(msg: IMU):
    print(f"IMU: acc=({msg.linear_acceleration.x:.2f},"
          f"{msg.linear_acceleration.y:.2f},{msg.linear_acceleration.z:.2f}) "
          f"gyro=({msg.angular_velocity.x:.2f},"
          f"{msg.angular_velocity.y:.2f},{msg.angular_velocity.z:.2f})")
def baro_cb(msg: Altimeter):
    with state["lock"]:
        state["baro"].update(dict(
            alt=msg.vertical_position,
            vz=msg.vertical_velocity,
            t=time.time()
        ))
    print(f"BARO: alt={msg.vertical_position:.2f} m, vel_z={msg.vertical_velocity:.2f} m/s")

def cam_cb(msg: Image):
    # msg.format should be R8G8B8 per your SDF. Convert bytes -> numpy -> BGR for OpenCV.
    # If your step != width*3, respect it; here we assume tightly packed RGB8.
    w, h = msg.width, msg.height
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    if arr.size != h * msg.step:
        return  # unexpected layout
    rgb = arr.reshape((h, msg.step))[:, :w*3].reshape((h, w, 3))
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    # Overlay minimal OSD text (you can replace this with parsed MSP DisplayPort later)
    with state["lock"]:
        imu = state["imu"].copy()
        gps = state["gps"].copy()
        baro = state["baro"].copy()
    overlay_osd(bgr, imu, gps, baro)

    state["frame"] = bgr


def overlay_osd(img, imu, gps, baro):
    def put(x, y, txt, color=(50,255,50)):
        cv2.putText(img, txt, (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

    put(10, 20, f"ALT {baro['alt']:.1f} m  Vz {baro['vz']:.1f} m/s")
    put(10, 40, f"LAT {gps['lat']:.6f}  LON {gps['lon']:.6f}")
    put(10, 60, f"VEL N/E/U {gps['vn']:.1f}/{gps['ve']:.1f}/{gps['vu']:.1f} m/s")
    put(10, 80, f"ACC {imu['ax']:.1f} {imu['ay']:.1f} {imu['az']:.1f} m/s^2")
    put(10,100, f"GYR {imu['gx']:.2f} {imu['gy']:.2f} {imu['gz']:.2f} rad/s")

def main():
    node = Node()
    mag_sub_ok = node.subscribe(Magnetometer, "/X3/mag", mag_cb)
    gps_sub_ok = node.subscribe(NavSat, "/X3/navsat", gps_cb)
    baro_sub_ok = node.subscribe(Altimeter, "/X3/baro", baro_cb)
    cam_sub_ok = node.subscribe(Image, "/X3/cam/front/image", cam_cb)
    sub_ok = node.subscribe(IMU, IMU_TOPIC, imu_cb)
    pub = node.advertise(MOTOR_TOPIC, Actuators)
    if not (sub_ok and pub):
        raise SystemExit("IMU sub or motor pub failed")

    msg = Actuators()
    try:
        t0 = time.time()
        while True:
            t = time.time() - t0
            w = 700.0 if t < 3.0 else 600.0  # take off then settle
            msg.velocity[:] = [w, w, w, w] #fr, rl, fl, rr
            pub.publish(msg)

            frame = state["frame"]
            if frame is not None:
                cv2.imshow("Gazebo Front Cam + OSD", frame)
                if cv2.waitKey(1) & 0xFF == 27:  # ESC
                    break
            time.sleep(0.02)
    except KeyboardInterrupt:
        msg.velocity[:] = [0,0,0,0]; pub.publish(msg)

if __name__ == "__main__":
    main()
