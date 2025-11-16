from __future__ import annotations

import struct
import base64
import json
import socket
import uuid
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Union, Tuple

from .lib import InavDefines, InavEnums, InavMSP
from .lib import boxes
from .mspcodec import MSPCodec
from .msp_serial import MSPSerial
import time

__all__ = ["MSPApi", "MSPServerTransport"]


def _scale(value: float, scale: float) -> int:
    return int(round(value * scale))

def _vec3(vec: Sequence[float], scale: float) -> tuple[int, int, int]:
    min_val = -32768
    max_val = 32767
    return (
        max(min(int(round(vec[0] * scale)), max_val), min_val),
        max(min(int(round(vec[1] * scale)), max_val), min_val),
        max(min(int(round(vec[2] * scale)), max_val), min_val),
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
        serial_transport: Optional[Any] = None,
    ) -> None:
        schema_path = codec_path or Path(__file__).with_name("lib") / "msp_messages.json"
        self._codec = MSPCodec.from_json_file(str(schema_path))
        endpoint = tcp_endpoint.strip() if tcp_endpoint else None
        keepalive_kwargs = {
            "keepalive_code": int(InavMSP.MSP_API_VERSION),
            "keepalive_interval": 5.0,
            "keepalive_timeout": 0.5,
        }
        if serial_transport is not None:
            self._serial = serial_transport
        else:
            if endpoint:
                if ":" not in endpoint:
                    raise ValueError("tcp_endpoint must be in HOST:PORT format")
                self._serial = MSPSerial(
                    endpoint,
                    baudrate,
                    read_timeout=read_timeout,
                    write_timeout=write_timeout,
                    tcp=True,
                    **keepalive_kwargs,
                )
            else:
                if not port:
                    raise ValueError("Serial port must be provided when tcp_endpoint is not set")
                self._serial = MSPSerial(
                    port,
                    baudrate,
                    read_timeout=read_timeout,
                    write_timeout=write_timeout,
                    **keepalive_kwargs,
                )


        self.box_ids = None
        self.chmap = [
            "roll",
            "pitch",
            "throttle",
            "yaw",
            "ch5",
            "ch6",
            "ch7",
            "ch8",
            "ch9",
            "ch10",
            "ch11",
            "ch12",
            "ch13",
            "ch14",
            "ch15",
            "ch16",
            "ch17",
            "ch18",
        ]
        self.rxmap = None
        self.diag: Optional[Dict[str, Any]] = None
        self._last_code: Optional[int] = None

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

    def _build_info(self, diag: Optional[Dict[str, Any]], code: Optional[Union[InavMSP, int]]) -> Dict[str, Any]:
        if code is not None:
            code_int = int(code.value) if isinstance(code, InavMSP) else int(code)
        elif diag is not None and "code" in diag:
            code_int = int(diag.get("code"))
        else:
            code_int = None
        info: Dict[str, Any] = {
            "code": code_int,
            "latency_ms": None,
            "cached": None,
            "cache_age_ms": None,
            "scheduled": False,
            "schedule_delay_s": None,
            "transport": None,
            "attempt": None,
            "timestamp": None,
            "server": None,
            "client": None,
            "code_stats": None,
        }
        if not diag:
            return info
        info["latency_ms"] = diag.get("duration_ms")
        info["cached"] = diag.get("cached")
        info["cache_age_ms"] = diag.get("cache_age_ms")
        info["scheduled"] = bool(diag.get("scheduled"))
        info["schedule_delay_s"] = diag.get("schedule_delay")
        info["transport"] = diag.get("transport")
        info["attempt"] = diag.get("attempt")
        info["timestamp"] = diag.get("timestamp")
        info["server"] = diag.get("server")
        info["client"] = diag.get("client")
        info["code_stats"] = diag.get("code_stats")
        info["raw"] = diag
        return info

    def _capture_info(self, code: Optional[InavMSP]) -> Dict[str, Any]:
        diag = getattr(self._serial, "last_diag", None)
        info = self._build_info(diag, code)
        self.diag = info
        self._last_code = info.get("code")
        return info

    def info_from_diag(self, diag: Optional[Dict[str, Any]], code: Optional[Union[InavMSP, int]] = None) -> Dict[str, Any]:
        info = self._build_info(diag, code)
        self.diag = info
        return info

    def _request_raw(
        self,
        code: InavMSP,
        payload: bytes = b"",
        *,
        timeout: float = 1.0,
    ) -> Tuple[Dict[str, Any], bytes]:
        self.open()
        rsp_code, rsp_payload = self._serial.request(int(code), payload, timeout=timeout)
        info = self._capture_info(code)
        if rsp_code != int(code):
            raise RuntimeError(f"Unexpected MSP response code {rsp_code} for request {int(code)}")
        return info, rsp_payload

    def _request(
        self,
        code: InavMSP,
        payload: bytes = b"",
        *,
        timeout: float = 1.0,
    ) -> Tuple[Dict[str, Any], Union[Mapping[str, Any], List[Mapping[str, Any]]]]:
        info, raw = self._request_raw(code, payload, timeout=timeout)
        return info, self._codec.unpack_reply(code, raw)

    def _pack_request(self, code: InavMSP, data: Mapping[str, Any]) -> bytes:
        return self._codec.pack_request(code, data)

    def _ensure_box_ids_cached(self) -> List[int]:
        if self.box_ids is None:
            self.get_box_ids()
        if self.box_ids is None:
            raise RuntimeError("BOX IDs could not be retrieved")
        return self.box_ids

    @staticmethod
    def _active_mode_mask_from_raw(raw_modes: Any) -> int:
        if raw_modes is None:
            return 0
        if isinstance(raw_modes, int):
            return raw_modes
        mask = 0
        if isinstance(raw_modes, list):
            for mode in raw_modes:
                if not isinstance(mode, Mapping):
                    continue
                box_index = mode.get("boxIndex")
                if box_index is None:
                    continue
                try:
                    mask |= 1 << int(box_index)
                except (TypeError, ValueError):
                    continue
        return mask

    def _decode_active_modes_mask(self, mask: Optional[int]) -> List[Dict[str, Any]]:
        if not mask:
            return []
        box_ids = self._ensure_box_ids_cached()
        active_modes: List[Dict[str, Any]] = []
        for idx, permanent_id in enumerate(box_ids):
            if not (mask & (1 << idx)):
                continue
            box_info = boxes.MODEBOXES.get(permanent_id, {})
            active_modes.append(
                {
                    "boxIndex": idx,
                    "permanentId": permanent_id,
                    "boxName": box_info.get("boxName", f"UNKNOWN_{permanent_id}"),
                }
            )
        return active_modes

    def sched_set(
        self,
        code: Union[InavMSP, int],
        delay: float,
        payload: Optional[Mapping[str, Any]] = None,
        timeout: float = 1.0,
    ) -> Tuple[Dict[str, Any], Any]:
        handler = getattr(self._serial, "sched_set", None)
        if handler is None:
            raise NotImplementedError("Transport does not support scheduling")
        code_int = int(code.value if isinstance(code, InavMSP) else code)
        result = handler(code_int, delay, payload, timeout)
        info = self._build_info(getattr(self._serial, "last_diag", None), code_int)
        self.diag = info
        return info, result

    def sched_get(self) -> Tuple[Dict[str, Any], Any]:
        handler = getattr(self._serial, "sched_get", None)
        if handler is None:
            raise NotImplementedError("Transport does not support scheduling")
        result = handler()
        info = self._build_info(getattr(self._serial, "last_diag", None), None)
        self.diag = info
        return info, result

    def sched_remove(self, code: Union[InavMSP, int]) -> Tuple[Dict[str, Any], Any]:
        handler = getattr(self._serial, "sched_remove", None)
        if handler is None:
            # fall back to sched_set delay<=0 if available
            handler_set = getattr(self._serial, "sched_set", None)
            if handler_set is None:
                raise NotImplementedError("Transport does not support scheduling")
            return self.sched_set(code, delay=0.0)
        code_int = int(code.value if isinstance(code, InavMSP) else code)
        result = handler(code_int)
        info = self._build_info(getattr(self._serial, "last_diag", None), code_int)
        self.diag = info
        return info, result

    # ----- API surface -----

    def get_api_version(self) -> Tuple[Dict[str, Any], Dict[str, int]]:
        info, rep = self._request(InavMSP.MSP_API_VERSION)
        return info, {
            "mspProtocolVersion": rep["mspProtocolVersion"],
            "apiVersionMajor": rep["apiVersionMajor"],
            "apiVersionMinor": rep["apiVersionMinor"],
        }

    def get_fc_variant(self) -> Tuple[Dict[str, Any], Dict[str, str]]:
        info, rep = self._request(InavMSP.MSP_FC_VARIANT)
        identifier = rep["fcVariantIdentifier"].rstrip(b"\x00").decode("ascii", errors="ignore")
        return info, {"fcVariantIdentifier": identifier}

    def get_board_info(self) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        info, rep = self._request(InavMSP.MSP_BOARD_INFO)
        board_identifier = rep["boardIdentifier"].rstrip(b"\x00").decode("ascii", errors="ignore")
        target_name = rep["targetName"].rstrip(b"\x00").decode("ascii", errors="ignore")
        comm_capabilities = rep["commCapabilities"]
        return info, {
            "boardIdentifier": board_identifier,
            "hardwareRevision": rep["hardwareRevision"],
            "osdSupport": rep["osdSupport"],
            "commCapabilities": {
                "vcp": bool(comm_capabilities & 0x1),
                "softSerial": bool(comm_capabilities & 0x2),
            },
            "targetName": target_name,
        }

    def get_sensor_config(self) -> Tuple[Dict[str, Any], Dict[str, InavEnums]]:
        info, rep = self._request(InavMSP.MSP_SENSOR_CONFIG)
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
        return info, converted

    def get_box_ids(self) -> Tuple[Dict[str, Any], List[int]]:
        info, rep = self._request(InavMSP.MSP_BOXIDS)
        self.box_ids = list(rep["boxIds"])
        return info, list(self.box_ids)

    def get_mode_ranges(self) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
        box_ids = self._ensure_box_ids_cached()
        info, entries = self._request(InavMSP.MSP_MODE_RANGES)
        min_pwm = InavDefines.CHANNEL_RANGE_MIN
        step_width = InavDefines.CHANNEL_RANGE_STEP_WIDTH
        summary: List[Dict[str, Any]] = []
        armfound = False
        for entry in entries:
            permanent_id = entry["modePermanentId"]
            if armfound and permanent_id == 0:
                continue
            else:
                armfound = True
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
                    "boxIndex": box_ids.index(permanent_id) if permanent_id in box_ids else None,
                    "permanentId": permanent_id,
                    "auxChannelIndex": aux_index,
                    "pwmRange": (pwm_start, pwm_end),
                }
            )
        return info, summary

    def get_rx_map(self) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        info, rep = self._request(InavMSP.MSP_RX_MAP)
        rc_map = list(rep.get("rcMap"))
        decoded= {}
        for idx, mapped_idx in enumerate(rc_map):
            source_name = self.chmap[idx] if idx < len(self.chmap) else f"aux{idx + 1}"
            target_name = self.chmap[mapped_idx] if mapped_idx < len(self.chmap) else f"aux{mapped_idx + 1}"
            decoded[idx] = {
                    "name": source_name,
                    "mappedTo": mapped_idx
                }
        self.rxmap = decoded
        return info, decoded

    def get_inav_status(self) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        info, rep = self._request(InavMSP.MSP2_INAV_STATUS)
        active_modes = self._decode_active_modes_mask(rep.get("activeModes"))
        arming_flags_raw = rep["armingFlags"]
        arming_flags = [
            flag
            for flag in InavEnums.armingFlag_e
            if flag is not InavEnums.armingFlag_e.ARMING_DISABLED_ALL_FLAGS and (arming_flags_raw & flag.value)
        ]
        sensor_status_raw = rep["sensorStatus"]
        sensor_status = [sensor for sensor in InavEnums.sensors_e if sensor_status_raw & sensor.value]
        return info, {
            "cycleTime": rep["cycleTime"],
            "i2cErrors": rep["i2cErrors"],
            "sensorStatus": sensor_status,
            "cpuLoad": rep["cpuLoad"],
            "profileAndBattProfile": rep["profileAndBattProfile"],
            "armingFlags": arming_flags,
            "activeModes": active_modes,
            "mixerProfile": rep["mixerProfile"],
        }

    def get_inav_analog(self) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        info, rep = self._request(InavMSP.MSP2_INAV_ANALOG)
        battery_flags_raw = rep["batteryFlags"]
        battery_state = InavEnums.batteryState_e((battery_flags_raw >> 2) & 0x3)
        return info, {
            "batteryFlags": {
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

    def get_rx_config(self) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        info, rep = self._request(InavMSP.MSP_RX_CONFIG)
        return info, {
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

    def get_logic_conditions(self) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
        # causes *** stack smashing detected ***: terminated
        # why?
        info, reps = self._request(InavMSP.MSP2_INAV_LOGIC_CONDITIONS)
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
                    "flags": [
                            flag
                            for flag in InavEnums.logicConditionFlags_e
                            if flags_raw & flag.value
                        ],
                }
            )
        return info, conditions

    def get_attitude(self) -> Tuple[Dict[str, Any], Dict[str, float]]:
        info, rep = self._request(InavMSP.MSP_ATTITUDE)
        return info, {axis: rep[axis] / 10.0 for axis in ("roll", "pitch", "yaw")}

    def get_altitude(self) -> Tuple[Dict[str, Any], Dict[str, float]]:
        info, rep = self._request(InavMSP.MSP_ALTITUDE)
        return info, {
            "estimatedAltitude": rep["estimatedAltitude"] / 100.0,
            "variometer": rep["variometer"] / 100.0,
            "baroAltitude": rep["baroAltitude"] / 100.0,
        }

    def get_imu(self) -> Tuple[Dict[str, Any], Dict[str, Dict[str, float]]]:
        info, rep = self._request(InavMSP.MSP_RAW_IMU)
        axes = ("X", "Y", "Z")
        return info, {
            "acc": {axis: rep[f"acc{axis}"] / 512.0 for axis in axes},
            "gyro": {axis: rep[f"gyro{axis}"] for axis in axes},
            "mag": {axis: rep[f"mag{axis}"] for axis in axes},
        }

    def get_rc_channels(self) -> Tuple[Dict[str, Any], List[int]]:
        info, payload = self._request_raw(InavMSP.MSP_RC)
        channel_width = 2
        if len(payload) % channel_width:
            raise ValueError("RC payload not aligned to 16-bit channel width")
        channel_count = len(payload) // channel_width
        values = list(struct.unpack(f"<{channel_count}H", payload)) if channel_count else []
        return info, values
        
    def set_rc_channels(self, channels: Sequence[int]) -> Tuple[Dict[str, Any], Mapping[str, Any]]:
        if not channels:
            raise ValueError("channels must not be empty")
        payload = struct.pack(f"<{len(channels)}H", *channels)
        return self._request(InavMSP.MSP_SET_RAW_RC, payload)

    def get_battery_config(self) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        info, rep = self._request(InavMSP.MSP2_INAV_BATTERY_CONFIG)
        return info, {
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

    def get_gps_statistics(self) -> Tuple[Dict[str, Any], Dict[str, float]]:
        info, rep = self._request(InavMSP.MSP_GPSSTATISTICS)
        packet_count = max(rep["packetCount"], 1)
        return info, {
            "lastMessageDt": rep["lastMessageDt"] / 1000.0,
            "errors": rep["errors"] / packet_count,
            "timeouts": rep["timeouts"] / packet_count,
            "hdop": rep["hdop"] / 100.0,
            "eph": rep["eph"] / 100.0,
            "epv": rep["epv"] / 100.0,
        }

    def get_waypoint_info(self) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        info, rep = self._request(InavMSP.MSP_WP_GETINFO)
        mission_valid = bool(rep["missionValid"])
        payload = {
            "wpCapabilities": rep["wpCapabilities"],
            "maxWaypoints": rep["maxWaypoints"],
            "missionValid": mission_valid,
            "waypointCount": rep["waypointCount"],
        }
        if mission_valid:
            remaining = rep["maxWaypoints"] - rep["waypointCount"]
            payload["waypointsRemaining"] = max(remaining, 0)
        return info, payload

    def get_raw_gps(self) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        info, rep = self._request(InavMSP.MSP_RAW_GPS)
        ground_course = rep["groundCourse"] / 10.0 if "groundCourse" in rep else rep["speed"] / 10.0
        return info, {
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
    ) -> Tuple[Dict[str, Any], Mapping[str, Any]]:
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
        return self._request(InavMSP.MSP_SET_WP, payload)

    def get_waypoint(self, waypoint_index: int) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        payload = self._pack_request(InavMSP.MSP_WP, {"waypointIndex": waypoint_index})
        info, rep = self._request(InavMSP.MSP_WP, payload)
        return info, {
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

    def get_nav_status(self) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        info, rep = self._request(InavMSP.MSP_NAV_STATUS)
        return info, {
            "navMode": InavEnums.navSystemStatus_Mode_e(rep["navMode"]),
            "navState": InavEnums.navigationFSMState_t(rep["navState"]),
            "activeWaypoint": {
                "action": InavEnums.navWaypointActions_e(rep["activeWpAction"]),
                "number": rep["activeWpNumber"],
            },
            "navError": InavEnums.navSystemStatus_Error_e(rep["navError"]),
            "targetHeading": rep["targetHeading"],
        }

    def get_active_modes(self) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
        """
        Returns a decoded list of currently active modes by combining NAV/INAV status data with cached box IDs.
        Falls back to MSP2_INAV_STATUS or MSP_ACTIVEBOXES if MSP_NAV_STATUS does not provide the bitmask.
        """
        self._ensure_box_ids_cached()
        info, nav_status = self.get_nav_status()
        raw_modes = nav_status.get("activeModes")
        has_raw_modes = "activeModes" in nav_status
        mask = self._active_mode_mask_from_raw(raw_modes)
        source_info = info

        if not has_raw_modes:
            info, status = self.get_inav_status()
            raw_modes = status.get("activeModes")
            has_raw_modes = "activeModes" in status
            mask = self._active_mode_mask_from_raw(raw_modes)
            source_info = info

        if not has_raw_modes:
            info, rep = self._request(InavMSP.MSP_ACTIVEBOXES)
            mask = self._active_mode_mask_from_raw(rep.get("activeModes"))
            source_info = info

        return source_info, self._decode_active_modes_mask(mask)

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
    ) -> Tuple[Dict[str, Any], Mapping[str, Any]]:
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
        info, raw_reply = self._request_raw(InavMSP.MSP_SIMULATOR, payload)
        if not raw_reply:
            return info, {}
        return info, self._codec.unpack_reply(InavMSP.MSP_SIMULATOR, raw_reply)


class MSPServerTransport:
    """TCP transport that speaks the JSON/line protocol exposed by msp_server.py."""

    def __init__(self, host: str, port: int, *, client_id: Optional[str] = None) -> None:
        self.host = host
        self.port = port
        self.client_id = client_id
        self._sock: Optional[socket.socket] = None
        self._reader: Optional[Any] = None
        self.last_diag: Optional[Dict[str, Any]] = None

    def open(self) -> None:
        if self._sock:
            return
        self._sock = socket.create_connection((self.host, self.port))
        self._reader = self._sock.makefile("r", encoding="utf-8")

    def close(self) -> None:
        if self._reader:
            try:
                self._reader.close()
            except Exception:
                pass
            self._reader = None
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None

    def _ensure_open(self) -> None:
        if not self._sock or not self._reader:
            self.open()

    def _send(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        self._ensure_open()
        payload.setdefault("id", uuid.uuid4().hex)
        if self.client_id:
            payload.setdefault("client_id", self.client_id)
        data = json.dumps(payload, separators=(",", ":"), ensure_ascii=False).encode("utf-8") + b"\n"
        assert self._sock and self._reader
        self._sock.sendall(data)
        line = self._reader.readline()
        if not line:
            raise RuntimeError("MSP server closed the connection")
        resp = json.loads(line)
        if resp.get("id") != payload["id"]:
            raise RuntimeError("Mismatched response ID from server")
        return resp

    def request(
        self,
        code: int,
        payload: bytes = b"",
        timeout: float = 1.0,
        force_version: Optional[int] = None,
        cacheable: bool = True,
    ) -> Tuple[int, bytes]:
        message: Dict[str, Any] = {
            "code": int(code),
            "timeout_ms": max(100, int(timeout * 1000)),
            "raw": base64.b64encode(payload or b"").decode("ascii"),
        }
        if not cacheable:
            message["no_cache"] = True
        resp = self._send(message)
        if not resp.get("ok"):
            raise RuntimeError(resp.get("error") or "MSP server error")
        payload_b64 = resp.get("payload_b64", "")
        decoded = base64.b64decode(payload_b64) if payload_b64 else b""
        self.last_diag = resp.get("diag")
        return resp.get("code", int(code)), decoded

    def sched_set(
        self,
        code: int,
        delay: float,
        payload: Optional[Mapping[str, Any]] = None,
        timeout: float = 1.0,
    ) -> Dict[str, Any]:
        message: Dict[str, Any] = {
            "action": "sched_set",
            "code": int(code),
            "delay": float(delay),
            "timeout_ms": max(100, int(timeout * 1000)),
        }
        if payload is not None:
            message["payload"] = payload
        resp = self._send(message)
        if not resp.get("ok"):
            raise RuntimeError(resp.get("error") or "Scheduler error")
        self.last_diag = resp.get("diag")
        return resp.get("schedule", {})

    def sched_get(self) -> Dict[str, Any]:
        resp = self._send({"action": "sched_get"})
        if not resp.get("ok"):
            raise RuntimeError(resp.get("error") or "Scheduler error")
        self.last_diag = resp.get("diag")
        return resp.get("schedules", {})

    def sched_remove(self, code: int) -> Dict[str, Any]:
        resp = self._send({"action": "sched_remove", "code": int(code)})
        if not resp.get("ok"):
            raise RuntimeError(resp.get("error") or "Scheduler error")
        self.last_diag = resp.get("diag")
        return resp

    
