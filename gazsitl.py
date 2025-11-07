#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import os
import threading
import time
from dataclasses import dataclass
from typing import Mapping, Optional, Sequence, Tuple

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

# no fallbacks
from gz.msgs11.actuators_pb2 import Actuators
from gz.msgs11.altimeter_pb2 import Altimeter
from gz.msgs11.imu_pb2 import IMU
from gz.msgs11.magnetometer_pb2 import Magnetometer
from gz.msgs11.navsat_pb2 import NavSat
from gz.transport14 import Node

from lib import InavEnums, InavMSP
from msp_api import MSPApi


MOTOR_TOPIC = "/X3/gazebo/command/motor_speed"
IMU_TOPIC = "/X3/imu"
GPS_TOPIC = "/X3/navsat"
MAG_TOPIC = "/X3/mag"
BARO_TOPIC = "/X3/baro"

GRAVITY = 9.80665
STANDARD_PRESSURE = 101325.0
LAPSE_RATE = 0.0065
SEA_LEVEL_TEMP_K = 288.15
GAS_CONSTANT = 8.3144598
AIR_MOLAR_MASS = 0.0289644
EARTH_GRAVITY = 9.80665
UPDATE_RATE_HZ = 500.0
RC_HZ = 200.0
MAX_SENSOR_STALE_S = 0.25
SIM_BATTERY_VOLTAGE = 15.4
MOTOR_PWM_MIN = 1000.0
MOTOR_PWM_MAX = 2100.0
MOTOR_SPEED_MAX = 800
RC_CHANNEL_COUNT = 16
RC_MID = 1500
RC_LOW = 910
RC_HIGH = 2099
slowdown = 1

@dataclass
class ImuSample:
    timestamp: float
    linear_accel: Tuple[float, float, float]
    angular_velocity: Tuple[float, float, float]
    orientation: Tuple[float, float, float, float]


@dataclass
class GpsSample:
    timestamp: float
    lat_deg: float
    lon_deg: float
    alt_m: float
    vel_east: float
    vel_north: float
    vel_up: float
    hdop: Optional[float] = None


@dataclass
class MagSample:
    timestamp: float
    tesla: Tuple[float, float, float]


@dataclass
class BaroSample:
    timestamp: float
    altitude_m: float
    vertical_velocity: float
    pressure_pa: Optional[float]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Gazebo ↔ INAV HITL bridge using MSP_SIMULATOR")
    parser.add_argument("--port", default=None, help="Serial device path (ignored if --tcp is used)")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--tcp", metavar="HOST:PORT", help="Connect using TCP socket instead of serial, e.g. localhost:5760")
    parser.add_argument("--read-timeout", type=float, default=0.05, help="MSP read timeout in seconds")
    parser.add_argument("--write-timeout", type=float, default=0.25, help="MSP write timeout in seconds")
    parser.add_argument("--debug", action="store_true", help="Print MSP/Gazebo telemetry debug info")
    return parser.parse_args()


def quaternion_to_euler_deg(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.degrees(math.copysign(math.pi / 2.0, sinp))
    else:
        pitch = math.degrees(math.asin(sinp))

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    return roll, pitch, yaw


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def standard_atmosphere_pressure(altitude_m: float) -> float:
    if altitude_m <= 0.0:
        return STANDARD_PRESSURE
    temp = SEA_LEVEL_TEMP_K - LAPSE_RATE * altitude_m
    exponent = (EARTH_GRAVITY * AIR_MOLAR_MASS) / (GAS_CONSTANT * LAPSE_RATE)
    return STANDARD_PRESSURE * (temp / SEA_LEVEL_TEMP_K) ** exponent


def decode_signed_16(value: int) -> int:
    return value - 65536 if value > 32767 else value


class GazeboSITLBridge:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.node = Node()
        self.lock = threading.Lock()
        self.imu: Optional[ImuSample] = None
        self.gps: Optional[GpsSample] = None
        self.mag: Optional[MagSample] = None
        self.baro: Optional[BaroSample] = None
        self.motor_pub = self.node.advertise(MOTOR_TOPIC, Actuators)
        if not self.motor_pub:
            raise SystemExit(f"Failed to advertise motor topic {MOTOR_TOPIC}")
        self.motor_msg = Actuators()
        self.motor_msg.velocity.extend([0.0, 0.0, 0.0, 0.0])

        if not self.node.subscribe(IMU, IMU_TOPIC, self._imu_cb):
            raise SystemExit(f"Failed to subscribe to {IMU_TOPIC}")
        if not self.node.subscribe(NavSat, GPS_TOPIC, self._gps_cb):
            raise SystemExit(f"Failed to subscribe to {GPS_TOPIC}")
        if not self.node.subscribe(Magnetometer, MAG_TOPIC, self._mag_cb):
            raise SystemExit(f"Failed to subscribe to {MAG_TOPIC}")
        if not self.node.subscribe(Altimeter, BARO_TOPIC, self._baro_cb):
            raise SystemExit(f"Failed to subscribe to {BARO_TOPIC}")

    def _imu_cb(self, msg: IMU) -> None:
        timestamp = time.time()
        orientation = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        accel = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        gyro = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        sample = ImuSample(timestamp, accel, gyro, orientation)
        with self.lock:
            self.imu = sample

    def _gps_cb(self, msg: NavSat) -> None:
        timestamp = time.time()
        sample = GpsSample(
            timestamp=timestamp,
            lat_deg=msg.latitude_deg,
            lon_deg=msg.longitude_deg,
            alt_m=msg.altitude,
            vel_east=msg.velocity_east,
            vel_north=msg.velocity_north,
            vel_up=msg.velocity_up,
            hdop=getattr(msg, "horizontal_position_covariance", None),
        )
        with self.lock:
            self.gps = sample

    def _mag_cb(self, msg: Magnetometer) -> None:
        timestamp = time.time()
        sample = MagSample(
            timestamp=timestamp,
            tesla=(msg.field_tesla.x, msg.field_tesla.y, msg.field_tesla.z),
        )
        with self.lock:
            self.mag = sample

    def _baro_cb(self, msg: Altimeter) -> None:
        timestamp = time.time()
        pressure = getattr(msg, "pressure", 0.0)
        if pressure <= 0.0:
            pressure = None
        sample = BaroSample(
            timestamp=timestamp,
            altitude_m=msg.vertical_position,
            vertical_velocity=msg.vertical_velocity,
            pressure_pa=pressure,
        )
        with self.lock:
            self.baro = sample

    def _snapshot(self, max_stale: float) -> Optional[Tuple[ImuSample, Optional[GpsSample], Optional[MagSample], BaroSample]]:
        now = time.time()
        with self.lock:

            imu = self.imu
            baro = self.baro
            gps = self.gps if self.gps and now - self.gps.timestamp <= max_stale else None
            mag = self.mag if self.mag and now - self.mag.timestamp <= max_stale else None

        if imu is None or baro is None:
            return None
        if now - imu.timestamp > max_stale or now - baro.timestamp > max_stale:
            return None
        return imu, gps, mag, baro

    def _prepare_gps_payload(self, gps: Optional[GpsSample]) -> Tuple[int, int, float, float, float, float, float, float, float, float]:
        if gps is None:
            return (
                InavEnums.gpsFixType_e.GPS_NO_FIX,
                0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )

        fix_type = InavEnums.gpsFixType_e.GPS_FIX_3D
        num_sat = 10
        speed = math.sqrt(gps.vel_east * gps.vel_east + gps.vel_north * gps.vel_north)
        course = math.degrees(math.atan2(gps.vel_east, gps.vel_north)) % 360.0
        vel_n = gps.vel_north
        vel_e = gps.vel_east
        vel_d = -gps.vel_up
        return fix_type, num_sat, gps.lat_deg, gps.lon_deg, gps.alt_m, speed, course, vel_n, vel_e, vel_d

    def _prepare_attitude(self, imu: ImuSample) -> Tuple[float, float, float]:
        qx, qy, qz, qw = imu.orientation
        return quaternion_to_euler_deg(qx, qy, qz, qw)

    def _prepare_accel(self, imu: ImuSample) -> Tuple[float, float, float]:
        ax, ay, az = imu.linear_accel
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        # Gazebo tends to publish acceleration already expressed in G; fall back
        # to converting from m/s^2 only if we detect larger magnitudes.
        if norm > 3.0:  # likely m/s^2, convert to G
            scale = 1.0 / GRAVITY
        else:
            scale = 1.0
        return (ax * scale, ay * scale, az * scale)

    def _prepare_gyro(self, imu: ImuSample) -> Tuple[float, float, float]:
        gx, gy, gz = imu.angular_velocity
        return (math.degrees(gx), math.degrees(gy), math.degrees(gz))

    def _prepare_mag(self, mag: Optional[MagSample]) -> Tuple[int, int, int]:
        if mag is None:
            return (0, 0, 0)
        scale = 1e6 * 20.0
        mx = int(clamp(round(mag.tesla[0] * scale), -32768, 32767))
        my = int(clamp(round(mag.tesla[1] * scale), -32768, 32767))
        mz = int(clamp(round(mag.tesla[2] * scale), -32768, 32767))
        return mx, my, mz

    def _baro_pressure(self, baro: BaroSample) -> float:
        if baro.pressure_pa and baro.pressure_pa > 0.0:
            return baro.pressure_pa
        return standard_atmosphere_pressure(baro.altitude_m)

    def _airspeed(self, gps: Optional[GpsSample]) -> float:
        if gps is None:
            return 0.0
        return math.sqrt(gps.vel_east ** 2 + gps.vel_north ** 2 + gps.vel_up ** 2)

    def _motor_pwm_to_speed(self, pwm: float) -> float:
        span = MOTOR_PWM_MAX - MOTOR_PWM_MIN
        if span <= 0:
            return 0.0
        normalised = (pwm - MOTOR_PWM_MIN) / span
        normalised = clamp(normalised, 0.0, 1.0)
        return normalised * MOTOR_SPEED_MAX * slowdown

    def _publish_motors(self, pwm_values: object) -> None:
        if isinstance(pwm_values, Mapping):
            pwm_values = pwm_values.get("motorOutputs", ())

        if isinstance(pwm_values, Sequence):
            pwm_seq = list(pwm_values)
        else:
            pwm_seq = [float(pwm_values)] if pwm_values is not None else []

        if not pwm_seq:
            return

        inav_to_gazebo = [1, 2, 3, 0]


        reordered = []
        for idx in inav_to_gazebo:
            if idx < len(pwm_seq):
                reordered.append(pwm_seq[idx])
            else:
                reordered.append(MOTOR_PWM_MIN)

        speeds = [self._motor_pwm_to_speed(pwm) for pwm in reordered[:4]]
        #0 = front right
        #1 = rear left
        #2 = front left
        #3 = rear right
        print("motor speeds:", speeds)
        for idx, value in enumerate(speeds):
            self.motor_msg.velocity[idx] = value
        self.motor_pub.publish(self.motor_msg)

    def stop_motors(self) -> None:
        for idx in range(len(self.motor_msg.velocity)):
            self.motor_msg.velocity[idx] = 0.0
        self.motor_pub.publish(self.motor_msg)

    def run(self) -> None:
        args = self.args
        poll_period = 1.0 / UPDATE_RATE_HZ if UPDATE_RATE_HZ > 0 else 0.01
        port = None if args.tcp else args.port
        with MSPApi(
            port=port,
            baudrate=args.baudrate,
            read_timeout=args.read_timeout,
            write_timeout=args.write_timeout,
            tcp_endpoint=args.tcp,
        ) as api:
            rc_channels = [RC_LOW] * RC_CHANNEL_COUNT
            rc_channels[0] = RC_MID
            rc_channels[1] = RC_MID
            rc_channels[3] = RC_MID

            armtime = 10
            last_status_log = 0.0
            ccc=0
            start_time = time.time()
            print()
            print("Mode ranges:")
            moderanges = api.get_mode_ranges()
            for entry in moderanges:
                print(entry)
            armclear = False
            arming = False

            
            while True:
                loop_start = time.time()
                snapshot = self._snapshot(MAX_SENSOR_STALE_S)
                if snapshot is None:
                    time.sleep(0.01)
                    ccc+=1
                    if ccc>100:
                        print('hung sleeping no snapshot - is Gazebo running?')
                    continue
                ccc=0
                elapsed_since_start = loop_start - start_time

                if armclear and not arming:
                    arming = True
                    armtime = elapsed_since_start

                rc_channels[api.chmap.index("ch6")] = RC_HIGH
                if arming:
                    if elapsed_since_start >= armtime:
                        rc_channels[api.chmap.index("ch5")] = RC_HIGH
                    if elapsed_since_start >= armtime + 3:
                        rc_channels[api.chmap.index("throttle")] = RC_HIGH
                    if elapsed_since_start >= armtime + 8:
                        rc_channels[api.chmap.index("ch7")] = RC_HIGH
                        rc_channels[api.chmap.index("throttle")] = RC_MID
                api.set_rc_channels(rc_channels)

                imu, gps, mag, baro = snapshot
                fix_type, num_sat, lat, lon, alt, speed, course, vel_n, vel_e, vel_d = self._prepare_gps_payload(gps)
                att_roll, att_pitch, att_yaw = self._prepare_attitude(imu)
                acc = self._prepare_accel(imu)
                gyro = self._prepare_gyro(imu)
                mag_counts = self._prepare_mag(mag)
                pressure = self._baro_pressure(baro)
                airspeed = self._airspeed(gps)

                flags = int(InavEnums.simulatorFlags_t.HITL_ENABLE) | int(InavEnums.simulatorFlags_t.HITL_USE_IMU) | int(
                    InavEnums.simulatorFlags_t.HITL_EXT_BATTERY_VOLTAGE
                )
                if gps:
                    flags |= int(InavEnums.simulatorFlags_t.HITL_HAS_NEW_GPS_DATA)
                if airspeed > 0.1:
                    flags |= int(InavEnums.simulatorFlags_t.HITL_AIRSPEED)

                reply = api.set_simulator(
                    simulator_version=2,
                    flags=flags,
                    gps={
                        "fix_type": fix_type,
                        "num_sat": num_sat,
                        "lat": lat,
                        "lon": lon,
                        "alt": alt,
                        "speed": speed,
                        "course": course,
                        "vel_n": vel_n,
                        "vel_e": vel_e,
                        "vel_d": vel_d,
                    },
                    attitude={"roll": att_roll, "pitch": att_pitch, "yaw": att_yaw},
                    acc=acc,
                    gyro=gyro,
                    baro_pressure=pressure,
                    mag=mag_counts,
                    battery_voltage=SIM_BATTERY_VOLTAGE,
                    airspeed=airspeed,
                    ext_flags=0,
                )

                motors = api._request_unpack(InavMSP.MSP_MOTOR)
                self._publish_motors(motors)

                if args.debug and reply:
                    stab_roll = decode_signed_16(reply["stabilizedRoll"])
                    stab_pitch = decode_signed_16(reply["stabilizedPitch"])
                    stab_yaw = decode_signed_16(reply["stabilizedYaw"])
                    stab_thr = decode_signed_16(reply["stabilizedThrottle"])
                    print(
                        f"MSP_SIM: roll={stab_roll} pitch={stab_pitch} yaw={stab_yaw} thr={stab_thr} "
                        f"motors={motors[:4]} flags=0x{reply['debugFlags']:02x}"
                    )

                if loop_start - last_status_log >= 1.0:
                    status = api.get_inav_status()
                    rc_state_raw = api.get_rc_channels() or []
                    rc_state = rc_state_raw[:8]
                    attitude = api.get_attitude()
                    altitude = api.get_altitude()
                    armingflags = status['armingFlags']['decoded']
                    allowed = {
                        InavEnums.armingFlag_e.SIMULATOR_MODE_HITL,
                        InavEnums.armingFlag_e.WAS_EVER_ARMED,
                    }

                    if not armingflags or set(armingflags) <= allowed:
                        armclear = True
                    
                    print()
                    print(f"T: {elapsed_since_start:.3f}")
                    if armclear: print('READY')
                    if arming: print("ARMED")
                    print('gazebo atti:',att_roll, att_pitch, att_yaw)
                    print('gazebo acc:',acc)
                    print('gazebo gyro:',gyro)
                    print('gazebo baro pressure:',pressure)
                    print("Arming flags:", status['armingFlags']['decoded'])
                    print("Active Modes:", status['activeModes'])
                    print("RC channels (AETR):", rc_channels[:8])
                    print("RC state (AERT):", rc_state)
                    print("Attitude:", attitude)
                    print("Altitude:", altitude)
                    print("Motors:",motors)
                    last_status_log = loop_start

                elapsed = time.time() - loop_start
                if poll_period > elapsed:
                    time.sleep(poll_period - elapsed)


def main() -> None:
    args = parse_args()
    bridge = GazeboSITLBridge(args)
    try:
        bridge.run()
    except KeyboardInterrupt:
        bridge.stop_motors()


if __name__ == "__main__":
    main()
