
/*
 * mp_internal.h -- internal interfaces for Mindprobe module
 */

#include <assert.h>
#include <stdarg.h>

#include "mp/mp_tlv.h"
#include "mp/mp.h"
#include "util/util.h"
#ifndef MP_EMBEDDED
#include "rts/rts.h"
#include <syslog.h>
#include "log/log.h"
#endif

#ifdef MP_EMBEDDED
#define MP_WARNING(message, ...)		\
    MP_printf(message, ## __VA_ARGS__)
#else /* MP_EMBEDDED */
#define MP_WARNING(message, ...) \
    RCP_log(LOG_WARNING, message, ## __VA_ARGS__)
#endif /* MP_EMBEDDED */

#define MP_FATAL(message, ...)					\
    mp_fatal_error(__FILE__, __LINE__, message, ## __VA_ARGS__)

/* For internal consistency checks, the sorts of checks that would be
   reasonable to disable in production code. */
#ifdef MP_EMBEDDED
#define MP_ASSERT(condition)					\
    do {							\
	if (!(condition)) {					\
	    MP_FATAL("assertion " #condition " failed");	\
	}							\
    } while (0)
#else
#define MP_ASSERT(condition) assert(condition)
#endif

/*
 * The mindprobe internal interface supports multiple clients
 * (initially to enable the blackbox recorder) connected to the
 * realtime pipe.  This is the maximum number of simultaneous clients.
 */
#ifdef MP_EMBEDDED
#define MP_MAX_RT_CLIENTS 1
#else
#define MP_MAX_RT_CLIENTS 4
#endif

/*
 * Internal TLV's used between mp listeners/clients and mp realtime (in RCP).
 */
enum mp_internal_tlv_type {
    MP_INT_TLV_CONNECT        = 1000,
    MP_INT_TLV_CONNECT_REPLY  = 1001,

    MP_INT_TLV_CLIENT_NAME    = 1002,
    MP_INT_TLV_PIPE_NAME      = 1003,
    MP_INT_TLV_CONNECT_RESULT = 1004,

    MP_INT_TLV_KEEPALIVE      = 1005,
};

/*
 * Interfaces in mp_realtime.c:
 */
void mp_realtime_init(void);
void mp_set_ready(int status);
int mp_is_ready(void);
int prepare_debug_message(struct UTIL_tlv_msg_buf *tlv_buf, const char *message, ...);
int prepare_debug_message_va(struct UTIL_tlv_msg_buf *tlv_buf, const char *message, va_list *app);
void mp_rt_process_command(struct UTIL_tlv_msg_buf *tlv_buf, int client_id);


/*
 * Interfaces in mp_probes.c:
 */
void mp_update_enabled_probes(int client_id);
void mp_enable_probe(int client_id, int probe_id, int enabled);
void mp_disable_all_probes(int client_id);
int mp_prepare_next_enabled_probe(int client_id, int probe_id, void **addr, unsigned *len );

#ifndef MP_EMBEDDED
void mp_printf_probe_value( int probe_id, char const *value_source );
int mp_parse_probe_value( char const * str, int probe_id );
void mp_call_init_hooks();
void mp_dump_probe_names( char const * filename );
#endif /* MP_EMBEDDED */

/*
 * Interfaces in mp_listener_common.c:
 */
void mp_send_client_protocol_version(void);
void mp_send_client_build_version(void);
void mp_reset_client_input(void);
int mp_receive_from_client(void); /* Returns non-zero on error from client. */


/*
 * Interfaces in mp_listener_tcp.c and/or mp_listener_uart.c:
 */
int mp_send_client(struct UTIL_tlv_msg_buf *tlv_buf); /* Returns non-zero on error writing to client. */
int mp_client_read(void *buf, int len);
void mp_close_client(void);
void mp_handle_command(struct UTIL_tlv_msg_buf *tlv_buf);
uint32_t mp_get_current_tick(void);
void mp_fatal_error(const char *file, int line, const char *format, ...);
int mp_client_receive_ready(void);

