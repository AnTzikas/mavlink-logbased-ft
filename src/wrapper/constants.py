import struct
from typing import Final

#* SHARED CONFIGURATION
# Using Little Endian (<) as standard for serialization
ENDIANNESS: Final = "<"


#* RECEIVE SIDE LOGGING 
# Structure: 
#   1. record_type (u8) - 1 byte
#   2. api_id (u8)      - 1 byte
#   3. payload_len (u32)- 4 bytes
#   4. timestamp (u64)  - 8 bytes
# Total Size: 14 bytes

RECV_HEADER_FMT: Final = f"{ENDIANNESS}BBIQ" 
RECV_HEADER_SZ: Final = struct.calcsize(RECV_HEADER_FMT) # Result: 14

# Record Types 
REC_NONE: Final = 0   # Timeout/None payload
REC_MSG: Final  = 1   # Valid MAVLink message payload

# API Identifiers
RECV_MATCH_ID: Final     = 0
RECV_MSG_ID: Final       = 1
WAIT_HEARTBEAT_ID: Final = 2 


#* SEND SIDE LOGGING
# Structure:
#   1. api_id (u8)      - 1 byte 
#   2. payload_len (u32)- 4 bytes
#   3. timestamp (u64)  - 8 bytes
# Total Size: 13 bytes

SEND_HEADER_FMT: Final = f"{ENDIANNESS}BIQ"
SEND_HEADER_SZ: Final = struct.calcsize(SEND_HEADER_FMT) # Result: 13

# API Identifiers
COMMAND_LONG_SEND_ID: Final = 0
COMMAND_INT_SEND_ID: Final  = 1
SET_MODE_SEND_ID: Final     = 2
PARAM_SET_SEND_ID: Final    = 3