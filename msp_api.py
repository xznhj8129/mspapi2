from __future__ import annotations

import struct
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Union

from lib import InavDefines, InavEnums, InavMSP
import lib.boxes as boxes
from mspcodec import MSPCodec
from msp_serial import MSPSerial
import time

__all__ = ["MSPApi"]


def _scale(value: float, scale: float) -> int:
    return int(round(value * scale))

def _vec3(vec: Sequence[float], scale: float) -> tuple[int, int, int]:
    return (
        int(round(vec[0] * scale)),
        int(round(vec[1] * scale)),
        int(round(vec[2] * scale)),
    )

class MSPApi:
    """
    High-level MSP helper backed by MSPCodec + MSPSerial.
    Provides typed / unit-converted accessors that mirror test_msp expectations.
    """

    def __init__(
        self,
        port: Optional[str] = "/dev/ttyACM0",
        baudrate: int = 115200,
        *,
        read_timeout: float = 0.05,
        write_timeout: float = 0.25,
        codec_path: Optional[Path] = None,
        tcp_endpoint: Optional[str] = None,
    ) -> None:
        schema_path = codec_path or Path(__file__).with_name("lib") / "msp_messages.json"
        self._codec = MSPCodec.from_json_file(str(schema_path))
        endpoint = tcp_endpoint.strip() if tcp_endpoint else None
        if endpoint:
            if ":" not in endpoint:
                raise ValueError("tcp_endpoint must be in HOST:PORT format")
            self._serial = MSPSerial(endpoint, baudrate, read_timeout=read_timeout, write_timeout=write_timeout, tcp=True)
        else:
            if not port:
                raise ValueError("Serial port must be provided when tcp_endpoint is not set")
            self._serial = MSPSerial(port, baudrate, read_timeout=read_timeout, write_timeout=write_timeout)
        self.box_ids = None

    def open(self) -> None:
        self._serial.open()

    def close(self) -> None:
        self._serial.close()

    def __enter__(self) -> "MSPApi":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    # ----- helpers -----

    def _request_raw(
        self,
        code: InavMSP,
        payload: bytes = b"",
        *,
        timeout: float = 1.0,
    ) -> bytes:
        self.open()
        rsp_code, rsp_payload = self._serial.request(int(code), payload, timeout=timeout)
        if rsp_code != int(code):
            raise RuntimeError(f"Unexpected MSP response code {rsp_code} for request {int(code)}")
        return rsp_payload

    def _request_unpack(
        self,
        code: InavMSP,
        payload: bytes = b"",
        *,
        timeout: float = 1.0,
    ) -> Union[Mapping[str, Any], List[Mapping[str, Any]]]:
        raw = self._request_raw(code, payload, timeout=timeout)
        return self._codec.unpack_reply(code, raw)

    def _pack_request(self, code: InavMSP, data: Mapping[str, Any]) -> bytes:
        return self._codec.pack_request(code, data)

    # ----- API surface -----

    def get_api_version(self) -> Dict[str, int]:
        rep = self._request_unpack(InavMSP.MSP_API_VERSION)
        return {
            "mspProtocolVersion": rep["mspProtocolVersion"],
            "apiVersionMajor": rep["apiVersionMajor"],
            "apiVersionMinor": rep["apiVersionMinor"],
        }

    def get_fc_variant(self) -> Dict[str, str]:
        rep = self._request_unpack(InavMSP.MSP_FC_VARIANT)
        identifier = rep["fcVariantIdentifier"].rstrip(b"\x00").decode("ascii", errors="ignore")
        return {"fcVariantIdentifier": identifier}

    def get_board_info(self) -> Dict[str, Any]:
        rep = self._request_unpack(InavMSP.MSP_BOARD_INFO)
        board_identifier = rep["boardIdentifier"].rstrip(b"\x00").decode("ascii", errors="ignore")
        target_name = rep["targetName"].rstrip(b"\x00").decode("ascii", errors="ignore")
        comm_capabilities = rep["commCapabilities"]
        return {
            "boardIdentifier": board_identifier,
            "hardwareRevision": rep["hardwareRevision"],
            "osdSupport": rep["osdSupport"],
            "commCapabilities": {
                "vcp": bool(comm_capabilities & 0x1),
                "softSerial": bool(comm_capabilities & 0x2),
            },
            "targetName": target_name,
        }

    def get_sensor_config(self) -> Dict[str, InavEnums]:
        rep = self._request_unpack(InavMSP.MSP_SENSOR_CONFIG)
        sensor_enums = {
            "accHardware": InavEnums.accelerationSensor_e,
            "baroHardware": InavEnums.baroSensor_e,
            "magHardware": InavEnums.magSensor_e,
            "pitotHardware": InavEnums.pitotSensor_e,
            "rangefinderHardware": InavEnums.rangefinderType_e,
            "opflowHardware": InavEnums.opticalFlowSensor_e,
        }
        converted: Dict[str, InavEnums] = {}
        for key, enum_cls in sensor_enums.items():
            if key in rep:
                converted[key] = enum_cls(rep[key])
        return converted

    def get_box_ids(self) -> List[int]:
        rep = self._request_unpack(InavMSP.MSP_BOXIDS)
        return list(rep["boxIds"])

    def get_mode_ranges(self) -> List[Dict[str, Any]]:
        if not self.box_ids:
            self.box_ids = self.get_box_ids()
        entries = self._request_unpack(InavMSP.MSP_MODE_RANGES)
        min_pwm = InavDefines.CHANNEL_RANGE_MIN
        step_width = InavDefines.CHANNEL_RANGE_STEP_WIDTH
        summary: List[Dict[str, Any]] = []
        for entry in entries:
            permanent_id = entry["modePermanentId"]
            if permanent_id == 0:
                continue
            aux_index = entry["auxChannelIndex"]
            start_step = entry["rangeStartStep"]
            end_step = entry["rangeEndStep"]
            box_info = boxes.MODEBOXES.get(permanent_id, {})
            box_name = box_info.get("boxName", f"UNKNOWN_{permanent_id}")
            pwm_start = min_pwm + start_step * step_width
            pwm_end = min_pwm + end_step * step_width
            summary.append(
                {
                    "mode": box_name,
                    "boxIndex": self.box_ids.index(permanent_id) if permanent_id in self.box_ids else None,
                    "permanentId": permanent_id,
                    "auxChannelIndex": aux_index,
                    "pwmRange": (pwm_start, pwm_end),
                }
            )
        return summary

    def get_inav_status(self) -> Dict[str, Any]:
        rep = self._request_unpack(InavMSP.MSP2_INAV_STATUS)
        if not self.box_ids:
            self.box_ids = self.get_box_ids()
        active_modes_bits = rep["activeModes"]
        active_modes: List[Dict[str, Any]] = []
        for idx, permanent_id in enumerate(self.box_ids):
            if active_modes_bits & (1 << idx):
                box_info = boxes.MODEBOXES.get(permanent_id, {})
                active_modes.append(
                    {
                        "boxIndex": idx,
                        "permanentId": permanent_id,
                        "boxName": box_info.get("boxName", f"UNKNOWN_{permanent_id}"),
                    }
                )
        arming_flags_raw = rep["armingFlags"]
        arming_flags = [
            flag
            for flag in InavEnums.armingFlag_e
            if flag is not InavEnums.armingFlag_e.ARMING_DISABLED_ALL_FLAGS and (arming_flags_raw & flag.value)
        ]
        sensor_status_raw = rep["sensorStatus"]
        sensor_status = [sensor for sensor in InavEnums.sensors_e if sensor_status_raw & sensor.value]
        return {
            "cycleTime": rep["cycleTime"],
            "i2cErrors": rep["i2cErrors"],
            "sensorStatus": {
                "raw": sensor_status_raw,
                "decoded": sensor_status,
            },
            "cpuLoad": rep["cpuLoad"],
            "profileAndBattProfile": rep["profileAndBattProfile"],
            "armingFlags": {
                "raw": arming_flags_raw,
                "decoded": arming_flags,
            },
            "activeModes": active_modes,
            "mixerProfile": rep["mixerProfile"],
        }

    def get_inav_analog(self) -> Dict[str, Any]:
        rep = self._request_unpack(InavMSP.MSP2_INAV_ANALOG)
        battery_flags_raw = rep["batteryFlags"]
        battery_state = InavEnums.batteryState_e((battery_flags_raw >> 2) & 0x3)
        return {
            "batteryFlags": {
                "raw": battery_flags_raw,
                "fullOnPlugIn": bool(battery_flags_raw & 0x1),
                "useCapacityThreshold": bool(battery_flags_raw & 0x2),
                "state": battery_state,
                "cellCount": battery_flags_raw >> 4,
            },
            "vbat": rep["vbat"] / 100.0,
            "amperage": rep["amperage"] / 100.0,
            "powerDraw": rep["powerDraw"] / 1000.0,
            "mAhDrawn": rep["mAhDrawn"],
            "mWhDrawn": rep["mWhDrawn"],
            "remainingCapacity": rep["remainingCapacity"],
            "percentageRemaining": rep["percentageRemaining"],
            "rssi": rep["rssi"],
        }

    def get_rx_config(self) -> Dict[str, Any]:
        rep = self._request_unpack(InavMSP.MSP_RX_CONFIG)
        return {
            "serialRxProvider": InavEnums.rxSerialReceiverType_e(rep["serialRxProvider"]),
            "maxCheck": rep["maxCheck"],
            "midRc": rep["midRc"],
            "minCheck": rep["minCheck"],
            "spektrumSatBind": rep["spektrumSatBind"],
            "rxMinUsec": rep["rxMinUsec"],
            "rxMaxUsec": rep["rxMaxUsec"],
            "bfCompatRcInterpolation": rep["bfCompatRcInterpolation"],
            "bfCompatRcInterpolationInt": rep["bfCompatRcInterpolationInt"],
            "bfCompatAirModeThreshold": rep["bfCompatAirModeThreshold"],
            "reserved1": rep["reserved1"],
            "reserved2": rep["reserved2"],
            "reserved3": rep["reserved3"],
            "bfCompatFpvCamAngle": rep["bfCompatFpvCamAngle"],
            "receiverType": InavEnums.rxReceiverType_e(rep["receiverType"]),
        }

    def get_logic_conditions(self) -> List[Dict[str, Any]]:
        # causes *** stack smashing detected ***: terminated
        # why?
        reps = self._request_unpack(InavMSP.MSP2_INAV_LOGIC_CONDITIONS)
        if isinstance(reps, Mapping):
            reps = [reps]  # defensive; schema should yield list
        conditions: List[Dict[str, Any]] = []
        for entry in reps or []:
            flags_raw = entry["flags"]
            conditions.append(
                {
                    "enabled": bool(entry["enabled"]),
                    "activatorId": None if entry["activatorId"] == 0xFF else entry["activatorId"],
                    "operation": InavEnums.logicOperation_e(entry["operation"]),
                    "operandAType": InavEnums.logicOperandType_e(entry["operandAType"]),
                    "operandAValue": entry["operandAValue"],
                    "operandBType": InavEnums.logicOperandType_e(entry["operandBType"]),
                    "operandBValue": entry["operandBValue"],
                    "flags": {
                        "raw": flags_raw,
                        "decoded": [
                            flag
                            for flag in InavEnums.logicConditionFlags_e
                            if flags_raw & flag.value
                        ],
                    },
                }
            )
        return conditions

    def get_attitude(self) -> Dict[str, float]:
        rep = self._request_unpack(InavMSP.MSP_ATTITUDE)
        return {axis: rep[axis] / 10.0 for axis in ("roll", "pitch", "yaw")}

    def get_altitude(self) -> Dict[str, float]:
        rep = self._request_unpack(InavMSP.MSP_ALTITUDE)
        return {
            "estimatedAltitude": rep["estimatedAltitude"] / 100.0,
            "variometer": rep["variometer"] / 100.0,
            "baroAltitude": rep["baroAltitude"] / 100.0,
        }

    def get_imu(self) -> Dict[str, Dict[str, float]]:
        rep = self._request_unpack(InavMSP.MSP_RAW_IMU)
        axes = ("X", "Y", "Z")
        return {
            "acc": {axis: rep[f"acc{axis}"] / 512.0 for axis in axes},
            "gyro": {axis: rep[f"gyro{axis}"] for axis in axes},
            "mag": {axis: rep[f"mag{axis}"] for axis in axes},
        }

    def get_rc_channels(self) -> List[int]:
        payload = self._request_raw(InavMSP.MSP_RC)
        channel_width = 2
        if len(payload) % channel_width:
            raise ValueError("RC payload not aligned to 16-bit channel width")
        channel_count = len(payload) // channel_width
        return list(struct.unpack(f"<{channel_count}H", payload)) if channel_count else []

    def set_rc_channels(self, channels: Sequence[int]) -> Mapping[str, Any]:
        if not channels:
            raise ValueError("channels must not be empty")
        payload = struct.pack(f"<{len(channels)}H", *channels)
        return self._request_unpack(InavMSP.MSP_SET_RAW_RC, payload)

    def get_battery_config(self) -> Dict[str, Any]:
        rep = self._request_unpack(InavMSP.MSP2_INAV_BATTERY_CONFIG)
        return {
            "vbatScale": rep["vbatScale"],
            "vbatSource": InavEnums.batVoltageSource_e(rep["vbatSource"]),
            "cellCount": rep["cellCount"],
            "vbatCellDetect": rep["vbatCellDetect"] / 100.0,
            "vbatMinCell": rep["vbatMinCell"] / 100.0,
            "vbatMaxCell": rep["vbatMaxCell"] / 100.0,
            "vbatWarningCell": rep["vbatWarningCell"] / 100.0,
            "currentOffset": rep["currentOffset"],
            "currentScale": rep["currentScale"],
            "capacityValue": rep["capacityValue"],
            "capacityWarning": rep["capacityWarning"],
            "capacityCritical": rep["capacityCritical"],
            "capacityUnit": InavEnums.batCapacityUnit_e(rep["capacityUnit"]),
        }

    def get_gps_statistics(self) -> Dict[str, float]:
        rep = self._request_unpack(InavMSP.MSP_GPSSTATISTICS)
        packet_count = max(rep["packetCount"], 1)
        return {
            "lastMessageDt": rep["lastMessageDt"] / 1000.0,
            "errors": rep["errors"] / packet_count,
            "timeouts": rep["timeouts"] / packet_count,
            "hdop": rep["hdop"] / 100.0,
            "eph": rep["eph"] / 100.0,
            "epv": rep["epv"] / 100.0,
        }

    def get_waypoint_info(self) -> Dict[str, Any]:
        rep = self._request_unpack(InavMSP.MSP_WP_GETINFO)
        mission_valid = bool(rep["missionValid"])
        info = {
            "wpCapabilities": rep["wpCapabilities"],
            "maxWaypoints": rep["maxWaypoints"],
            "missionValid": mission_valid,
            "waypointCount": rep["waypointCount"],
        }
        if mission_valid:
            remaining = rep["maxWaypoints"] - rep["waypointCount"]
            info["waypointsRemaining"] = max(remaining, 0)
        return info

    def get_raw_gps(self) -> Dict[str, Any]:
        rep = self._request_unpack(InavMSP.MSP_RAW_GPS)
        ground_course = rep["groundCourse"] / 10.0 if "groundCourse" in rep else rep["speed"] / 10.0
        return {
            "fixType": InavEnums.gpsFixType_e(rep["fixType"]),
            "numSat": rep["numSat"],
            "latitude": rep["latitude"] / 1e7,
            "longitude": rep["longitude"] / 1e7,
            "altitude": rep["altitude"] / 100.0,
            "speed": rep["speed"] / 100.0,
            "groundCourse": ground_course,
        }

    def set_waypoint(
        self,
        *,
        waypointIndex: int,
        action: InavEnums.navWaypointActions_e,
        latitude: float,
        longitude: float,
        altitude: float,
        param1: int = 0,
        param2: int = 0,
        param3: int = 0,
        flag: int = 0,
    ) -> Mapping[str, Any]:
        payload = self._pack_request(
            InavMSP.MSP_SET_WP,
            {
                "waypointIndex": waypointIndex,
                "action": int(action),
                "latitude": int(round(latitude * 1e7)),
                "longitude": int(round(longitude * 1e7)),
                "altitude": int(round(altitude * 100.0)),
                "param1": param1,
                "param2": param2,
                "param3": param3,
                "flag": flag,
            },
        )
        return self._request_unpack(InavMSP.MSP_SET_WP, payload)

    def get_waypoint(self, waypoint_index: int) -> Dict[str, Any]:
        payload = self._pack_request(InavMSP.MSP_WP, {"waypointIndex": waypoint_index})
        rep = self._request_unpack(InavMSP.MSP_WP, payload)
        return {
            "waypointIndex": rep["waypointIndex"],
            "action": InavEnums.navWaypointActions_e(rep["action"]),
            "latitude": rep["latitude"] / 1e7,
            "longitude": rep["longitude"] / 1e7,
            "altitude": rep["altitude"] / 100.0,
            "param1": rep["param1"],
            "param2": rep["param2"],
            "param3": rep["param3"],
            "flag": rep["flag"],
        }

    def get_nav_status(self) -> Dict[str, Any]:
        rep = self._request_unpack(InavMSP.MSP_NAV_STATUS)
        return {
            "navMode": InavEnums.navSystemStatus_Mode_e(rep["navMode"]),
            "navState": InavEnums.navigationFSMState_t(rep["navState"]),
            "activeWaypoint": {
                "action": InavEnums.navWaypointActions_e(rep["activeWpAction"]),
                "number": rep["activeWpNumber"],
            },
            "navError": InavEnums.navSystemStatus_Error_e(rep["navError"]),
            "targetHeading": rep["targetHeading"],
        }



    def set_simulator(
        self,
        simulator_version: int,
        flags: int,
        *,
        gps: Mapping[str, Any],
        attitude: Mapping[str, float],
        acc: Sequence[float],
        gyro: Sequence[float],
        baro_pressure: float,
        mag: Sequence[int],
        battery_voltage: float,
        airspeed: float,
        ext_flags: int,
    ) -> Mapping[str, Any]:
        gps_fix_type = int(gps["fix_type"])
        gps_num_sat = int(gps["num_sat"])
        gps_lat = _scale(gps["lat"], 1e7)
        gps_lon = _scale(gps["lon"], 1e7)
        gps_alt = _scale(gps["alt"], 100.0)
        gps_speed = _scale(gps["speed"], 100.0)
        gps_course = _scale(gps["course"], 10.0)
        gps_vel_n = _scale(gps["vel_n"], 100.0)
        gps_vel_e = _scale(gps["vel_e"], 100.0)
        gps_vel_d = _scale(gps["vel_d"], 100.0)

        imu_roll = _scale(attitude["roll"], 10.0)
        imu_pitch = _scale(attitude["pitch"], 10.0)
        imu_yaw = _scale(attitude["yaw"], 10.0)

        acc_x, acc_y, acc_z = _vec3(acc, 1000.0)
        gyro_x, gyro_y, gyro_z = _vec3(gyro, 16.0)
        mag_x, mag_y, mag_z = _vec3(mag, 1.0)

        payload = self._pack_request(
            InavMSP.MSP_SIMULATOR,
            {
                "simulatorVersion": simulator_version,
                "simulatorFlags_t": flags,
                "gpsFixType": gps_fix_type,
                "gpsNumSat": gps_num_sat,
                "gpsLat": gps_lat,
                "gpsLon": gps_lon,
                "gpsAlt": gps_alt,
                "gpsSpeed": gps_speed,
                "gpsCourse": gps_course,
                "gpsVelN": gps_vel_n,
                "gpsVelE": gps_vel_e,
                "gpsVelD": gps_vel_d,
                "imuRoll": imu_roll,
                "imuPitch": imu_pitch,
                "imuYaw": imu_yaw,
                "accX": acc_x,
                "accY": acc_y,
                "accZ": acc_z,
                "gyroX": gyro_x,
                "gyroY": gyro_y,
                "gyroZ": gyro_z,
                "baroPressure": _scale(baro_pressure, 1.0),
                "magX": mag_x,
                "magY": mag_y,
                "magZ": mag_z,
                "vbat": _scale(battery_voltage, 10.0),
                "airspeed": _scale(airspeed, 100.0),
                "extFlags": ext_flags,
            },
        )
        raw_reply = self._request_raw(InavMSP.MSP_SIMULATOR, payload)
        if not raw_reply:
            return {}
        return self._codec.unpack_reply(InavMSP.MSP_SIMULATOR, raw_reply)

    
