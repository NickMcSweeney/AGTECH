
/*
 * mp.h -- external interfaces for the Mindprobe module
 */

#ifndef SYS_MP_H
#define SYS_MP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdarg.h>

enum MP_probe_type {
    MP_PROBE_TYPE_NULL,
    MP_PROBE_TYPE_UINT8,
    MP_PROBE_TYPE_INT8,
    MP_PROBE_TYPE_UINT16,
    MP_PROBE_TYPE_INT16,
    MP_PROBE_TYPE_UINT32,
    MP_PROBE_TYPE_INT32,
    MP_PROBE_TYPE_UINT64,
    MP_PROBE_TYPE_INT64,
    MP_PROBE_TYPE_FLOAT,
    MP_PROBE_TYPE_DOUBLE,
    MP_PROBE_TYPE_TLV_DATA,   /* variable-size TLV-encoded data */
    MP_PROBE_TYPE_BOOL,
    MP_PROBE_TYPE_STRING,     /* variable-size null-terminated char string */
};

enum MP_TLV_probe_data_type {
    MP_TLV_TYPE_UNKNOWN = 0,

    // See:
    // bot/vision/PotOccupancy.cpp
    // com/qrobotics/mindprobe/ui/cartesian/PlotOccupancy.java
    MP_TLV_TYPE_OCCUPANCY_GRID_pre_r4317 = 1,
    MP_TLV_TYPE_OCCUPANCY_GRID = 2,

    // See:
    // bot/vision/Sick.h
    // com/qrobotics/mindprobe/ui/cartesian/Scan.java
    MP_TLV_TYPE_SCAN = 3,

    // See:
    // bot/mobility/DubinsController.h
    // com/qrobotics/mindprobe/ui/cartesian/DubinsPath.java
    MP_TLV_TYPE_DUBINS = 4,

    // See:
    // bot/vision/PotPickSensor.h
    // com/qrobotics/mindprobe/ui/cartesian/PotHypotheses.java
    MP_TLV_TYPE_POT_HYPOTHESES = 5,

    // See:
    // bot/boundary/BoundaryCamera.cpp
    // com/qrobotics/mindprobe/ui/cartesian/PotBoundaryCameraLines.java
    MP_TLV_TYPE_BOUNDARY_CAMERA = 6,

    // See:
    // bot/boundary/BoundaryCamera.cpp
    // com/qrobotics/mindprobe/ui/cartesian/BoundaryCameraThumbnail.java
    MP_TLV_TYPE_BOUNDARY_CAM_THUMBNAIL = 7,
    
    // See:
    // bot/frame/Mindprobe.h
    // com/qrobotics/mindprobe/ui/cartesian/PlotPolygon.java
    MP_TLV_TYPE_POLYGON = 8,

    // See:
    // bot/mobility/PolyPathController.h
    // com/qrobotics/mindprobe/ui/cartesian/PolyPath.java
    MP_TLV_TYPE_POLYPATH = 9,
};

#define MP_NULL_PROBE_ID (-1)

// MP_init() initializes the mindprobe subsystem, and must be called
// before any calls to MP_add_probe_* functions.
void MP_init();

#ifdef MP_EMBEDDED
// MP_EMBEDDED implies that the mindprobe subsystem is being compiled
// for an embedded microcontroller without a real-time subsystem or a TCP
// stack.  It will communicate to the mindprobe application over a serial
// port using the interfaces passed into MP_begin().
struct mp_ops {
    // The uart_read() operation will copy pending characters from the
    // uart's receive buffer into buf, up to buflen.  It returns the
    // number of characters copied.
    uint32_t (*uart_read)(void *buf, uint32_t buflen);

    // The uart_read_ready() operation will return non-zero if there
    // are any characters ready to be read from the uart's receive
    // buffers.
    uint32_t (*uart_read_ready)(void);

    // The uart_write_room() operation returns the number of bytes
    // available in the UART's transmit buffer.
    uint32_t (*uart_write_room)(void);

    // The uart_write() operation copies up to 'len' characters from
    // 'buf' into the UART's transmit buffer.  It returns the number
    // of characters copied out.
    uint32_t (*uart_write)(void *buf, uint32_t len);

    // Gets the current tick count of the system
    uint32_t (*get_current_tick)(void);

    // Called when a fatal error has been detected by the mindprobe
    // subsystem.
    void (*fatal_error)(const char *file, int line);
};

// MP_begin() must be called after all MP_add_probe_var_* calls and
// before calling MP_do_tick() for the first time.  This is the
// embedded version that takes an mp_uart_ops structure that specifies
// the interface mindprobe will use to talk to the UART.
void MP_begin(struct mp_ops *ops);
#else

// MP_begin() must be called after all MP_add_probe_var_* calls and
// before calling MP_do_tick() for the first time.  This is the full
// RCP version that specifies a TCP port that mindprobe should listen
// for connections on.
void MP_begin(int port);
void MP_set_host_info(char const *hostname, char const *nickname);
#endif

// MP_do_tick() should be called once per iteration of the main
// processing loop.  It will handle any incoming commands from the
// MindProbe application and send probe updates.
void MP_do_tick(void);

void MP_printf(const char *message, ...)
     __attribute__ ((__format__ (__printf__, 1, 2)));
void MP_vprintf(const char *message, va_list ap);

int MP_puts(const char *message);

int MP_add_probe_var(const char *name, enum MP_probe_type type, unsigned len,
		     void *variable_address);

void MP_set_probe_var_addr( int probe_id, void *variable_address );

static inline int MP_add_probe_enum(const char *name,
                                    int enum_num_bytes,
				    void *variable_address)
{
    enum MP_probe_type mp_int_type = MP_PROBE_TYPE_NULL;
    switch( enum_num_bytes ) {
        case 1: mp_int_type = MP_PROBE_TYPE_INT8;  break;
        case 2: mp_int_type = MP_PROBE_TYPE_INT16; break;
        case 4: mp_int_type = MP_PROBE_TYPE_INT32; break;
        case 8: mp_int_type = MP_PROBE_TYPE_INT64; break;
    }
    return MP_add_probe_var(name, mp_int_type, enum_num_bytes,
			    variable_address);
}

static inline int MP_add_probe_bool(const char *name,
                                    uint8_t *variable_address)
{
    return MP_add_probe_var(name, MP_PROBE_TYPE_BOOL, sizeof(uint8_t),
			    variable_address);
}

static inline int MP_add_probe_int8(const char *name,
				     int8_t *variable_address)
{
    return MP_add_probe_var(name, MP_PROBE_TYPE_INT8, sizeof(int8_t),
			    variable_address);
}

static inline int MP_add_probe_uint8(const char *name,
				      uint8_t *variable_address)
{
    return MP_add_probe_var(name, MP_PROBE_TYPE_UINT8, sizeof(uint8_t),
			    variable_address);
}

static inline int MP_add_probe_int16(const char *name,
				     int16_t *variable_address)
{
    return MP_add_probe_var(name, MP_PROBE_TYPE_INT16, sizeof(int16_t),
			    variable_address);
}

static inline int MP_add_probe_uint16(const char *name,
				      uint16_t *variable_address)
{
    return MP_add_probe_var(name, MP_PROBE_TYPE_UINT16, sizeof(uint16_t),
			    variable_address);
}

static inline int MP_add_probe_int32(const char *name,
				     int32_t *variable_address)
{
    return MP_add_probe_var(name, MP_PROBE_TYPE_INT32, sizeof(int32_t),
			    variable_address);
}

static inline int MP_add_probe_uint32(const char *name,
				      uint32_t *variable_address)
{
    return MP_add_probe_var(name, MP_PROBE_TYPE_UINT32, sizeof(uint32_t),
			    variable_address);
}

static inline int MP_add_probe_uint64(const char *name,
				      uint64_t *variable_address)
{
    return MP_add_probe_var(name, MP_PROBE_TYPE_UINT64, sizeof(uint64_t),
			    variable_address);
}

static inline int MP_add_probe_int64(const char *name,
				      int64_t *variable_address)
{
    return MP_add_probe_var(name, MP_PROBE_TYPE_INT64, sizeof(int64_t),
			    variable_address);
}

static inline int MP_add_probe_float(const char *name,
				     float *variable_address)
{
    return MP_add_probe_var(name, MP_PROBE_TYPE_FLOAT, sizeof(float),
			    variable_address);
}

static inline int MP_add_probe_double(const char *name,
				      double *variable_address)
{
    return MP_add_probe_var(name, MP_PROBE_TYPE_DOUBLE, sizeof(double),
			    variable_address);
}

int MP_is_probe_enabled(int probe_id);

int MP_get_probe_info(int probe_id, enum MP_probe_type *type,
                      unsigned *len, const char **name);

void MP_add_probe_hooks(int probe_id, void *hook_arg,
                        void (*init_hook)(int probe_id, void *arg),
			int (*enable_hook)(int probe_id, void *arg, int enabled),
			int (*read_hook)(int probe_id, void *arg),
			void (*write_hook)(int probe_id, void *arg));

int MP_get_probe_ID_for_name(char const * probe_name);
int MP_write_probe(int probe_id, void *data, int len, char const * source);
int MP_read_probe(int probe_id, void *data, int max_len);

    
#ifdef __cplusplus
}
#endif

#endif /* SYS_MP_H */
