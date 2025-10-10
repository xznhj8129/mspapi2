# mspapi2
Clean rewrite of an API for MultiWii Serial Protocol for [BetaFlight](https://github.com/betaflight/betaflight) and [INAV](https://github.com/INAVFlight/INAV)

[unavlib](https://github.com/xznhj8129/uNAVlib) was horrifying spaghetti code and driving me insane; this will use clearly seperated layers to keep serial handling, message brokering and threaded blocking clients completely separated

* Heavy use of [msp-documentation](https://github.com/xznhj8129/msp_documentation) and source files for source of truth (and not abandonned projects old enough to walk to school)
* Direct use of source code enums
* Direct use of source code defines wherever possible
* Focus on parsing and converting actual flight controller code into API functions, not re-re-re-reimplementing stuff
* Use parsed and interpreted direct source information to generate documentation (see msp-documentation)
* Will implement robust asynchronous non-blocking message brokering for single-serial connection handler shared between multiple clients (multiprocessing or zeromq)


Will change quickly while i figure out cleanest way to seperate everything


### bare minimum messages needed:
* MSP_API_VERSION
* MSP_FC_VARIANT
* MSP_FC_VERSION
* MSP_BUILD_INFO
* MSP_BOARD_INFO
* MSP_UID
* MSP_NAME
* MSP_STATUS
* MSP_STATUS_EX
* MSP_SENSOR_CONFIG
* MSP_SENSOR_DATA
* MSP_BATTERY_CONFIG
* MSP_BATTERY_STATE
* MSP_BOXIDS
* MSP2_INAV_STATUS
* MSP2_INAV_ANALOG
* MSP_MOTOR
* MSP_ATTITUDE
* MSP_ALTITUDE
* MSP_MODE_RANGES 
* MSP_VOLTAGE_METER_CONFIG
* MSP_RC
* MSP_RC_OVERRIDE
* MSP_RAW_GPS
* MSP_GPSSTATISTICS
* MSP_NAV_STATUS
* MSP_WP
* MSP_SET_WP
* MSP_WP_GETINFO