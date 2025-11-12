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


# Architecture:
## MSP Handler (msp_serial.py)
* Single point of contact to FC and only serial socket handler
* socket communication to API
* packed bytes in -> serial req/resp -> packed bytes out
* robust
* will not block API

## API (msp_api.py)
* Encodes/decodes MSP bytes from/to parsed data using msp_codec
* Provides high level function API for common commands

## Server
* Message broker, handles multiple client connections
* uses message queues to queue MSP message requests and responses
* Programmable fixed interval message scheduler
* Keeps timing and latency information per message
 * deduplicates messages (crc32 it, identical MSP message within a small interval (1 sec?) will yield same response, hence send only once even if multiple clients request it)
 * does not block waiting for MSP handler response

## Client-Server connection
* Client sends message and checks 
* JSON packet {"code":int, "payload":{}, "time":int, "client":str} to msgpack
* sent to TCP socket

scheduler: clients have ability to set timer on a message to send periodically
message deduping: crc32 message to check similarity of code+payload, check if it's already being sent