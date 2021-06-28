
#ifndef _MP_TLV_H
#define _MP_TLV_H

#include <stdint.h>
#include <stddef.h>

/*
 * High byte is major, low byte is minor, corresponds to revision of
 * mp-protocol.txt.
 */
#define MP_PROTOCOL_VERSION 0x0104
 
/*
 * These types must be consistent with values in mp-protocol.txt.
 */
enum mp_tlv_type {
    MP_TLV_NULL                  = 0,
    MP_TLV_CONNECTED             = 1,
    MP_TLV_DISCONNECTED          = 2,
    MP_TLV_PROTOCOL_VERSION      = 3,
    MP_TLV_DISCOVER_PROBES       = 4,
    MP_TLV_HZ                    = 5, 
    MP_TLV_PROBE_DEF             = 6, 
    MP_TLV_ENABLE_PROBES         = 7, 
    MP_TLV_START_PROBES          = 8, 
    MP_TLV_STOP_PROBES           = 9, 
    MP_TLV_PROBE_DATA            = 10,
    MP_TLV_CURRENT_TICK          = 11,
    MP_TLV_MISSED_DATA           = 12,
    MP_TLV_DEBUG_MESSAGE         = 13,
    MP_TLV_MESSAGE_TEXT          = 14,
    MP_TLV_MISSED_DEBUG_MESSAGES = 15,
    MP_TLV_WRITE_PROBES          = 16,
    MP_TLV_TIMESTAMP             = 17,   /* Time in seconds since the epoch as a big-endian 64-bit value. */
    MP_TLV_HOSTNAME              = 18,   /* Hostname of system running RCP as a zero-terminated string. */
    MP_TLV_NICKNAME              = 19,   /* Nickname of system running RCP as a zero-terminated string. */
};

/*
 * Reserved bytes for UART framing.  Must also match mp-protocol.txt.
 */
enum mp_framing_bytes {
    MP_ESCAPE = 0x80,
    MP_SYNC   = 0x81,
};

#endif /* _MP_TLV_H */
