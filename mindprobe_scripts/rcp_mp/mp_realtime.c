
/*
 * mp_realtime.c -- real-time side of the Mindprobe code
 */

/*
 * To do:
 *
 * - Add debug warnings for errors when parsing.
 */

#include <errno.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

#ifndef MP_EMBEDDED
#include <syslog.h>
#ifndef BTB
#include <rtdk.h>
#endif  /* BTB */
#endif  /* MP_EMBEDDED */

#include "mp_internal.h"

#include "log/log.h"
#include "util/util.h"

#include "hwspecs.h"
#include "hai_build_version.h"

#ifndef MP_HZ
#define MP_HZ HW_TICK_HZ
#endif

#define MAX_MISSED_PROBE_UPDATES (MP_HZ * 5)

static int mp_ready = 0;   /* Has the MindProbe subsystem been launched? */

typedef struct {
    uint8_t probing;
    int missed_probe_updates;
    int missed_messages;
#ifndef MP_EMBEDDED
    /* Xenomai pipe that talks to client (MP listener or blackboc recorder), via RTS abstraction: */
    char *pipe_name;
    struct RTS_rt_pipe pipe;
    int pipe_opened;

    /* We have to close pipes in a non-realtime context as the pipe
     * implementation will switch out of realtime on pipe deletes.
     * Set this flag, and the background "closer" thread will close it
     * when it sees it.
     */
    int should_be_closed;

    /* Information used by the state machine in do_send_client_probe_discovery() */
    int sending_probe_discovery;
    int probe_discovery_index;
#endif
} client_info_t;

static client_info_t clients[MP_MAX_RT_CLIENTS];

#ifndef MP_EMBEDDED
/* Xenomai pipe that listens for connection requests. */
static struct RTS_rt_pipe connection_pipe;
#endif

#undef DEBUG_MP

#ifdef DEBUG_MP
static int32_t test_messages = 0;
static int32_t skip_probes[MP_MAX_RT_CLIENTS] = { 0 };
#ifdef NOTYET
static char test_string[11] = "shazam";
#endif
#endif

/*
 * Forward declarations:
 */
static int send_dropped_count(int client_id, int count, enum mp_tlv_type t);
static int send_client_tlv(int client_id, enum mp_tlv_type t, unsigned int l, void *v);
static int send_tlv(int client_id, struct UTIL_tlv_msg_buf *tlv_buf);
static void send_client_host_info();

#ifndef MP_EMBEDDED
int mp_client_is_alive(int client_id)
{
    return clients[client_id].pipe_opened && !clients[client_id].should_be_closed;
}

pthread_t mp_client_closer_thread;

void *mp_client_closer(void *dummy_arg)
{
    int client_id;

    while (1) {
	sleep(1);

	for (client_id = 0; client_id < MP_MAX_RT_CLIENTS; client_id++) {
	    if (clients[client_id].should_be_closed) {
		clients[client_id].should_be_closed = 0;
		clients[client_id].pipe_opened = 0;
		clients[client_id].missed_probe_updates = 0;
		clients[client_id].missed_messages = 0;

		clients[client_id].sending_probe_discovery = 0;

		/* Destroy and re-create the pipe so that clients will
		 * notice and are forced to reconnect.
		 */
		RTS_rt_pipe_delete(&clients[client_id].pipe);
		RTS_rt_pipe_create(&clients[client_id].pipe, clients[client_id].pipe_name,
				   sizeof(struct UTIL_tlv_msg_buf));
	    }
	}
    }

    return NULL;
}
#endif

void mp_realtime_init(void)
{
    int client_id;

#ifndef MP_EMBEDDED
    char pipe_name[16];
    int status;

    RTS_pthread_create(&mp_client_closer_thread, NULL, mp_client_closer, NULL);

    status = RTS_rt_pipe_create(&connection_pipe, RTS_MINDPROBE_PIPE,
				sizeof(struct UTIL_tlv_msg_buf));
    if (status != 0) {
	MP_FATAL("Could not create Mindprobe connection pipe: %s", strerror(-status));
    }
#endif /* MP_EMBEDDED */

    for (client_id = 0; client_id < MP_MAX_RT_CLIENTS; client_id++) {
	clients[client_id].missed_probe_updates = 0;
	clients[client_id].missed_messages = 0;
#ifndef MP_EMBEDDED
	sprintf(pipe_name, "client%d", client_id);
	clients[client_id].pipe_name = strdup(pipe_name);
	status = RTS_rt_pipe_create(&clients[client_id].pipe, pipe_name, 
				    sizeof(struct UTIL_tlv_msg_buf));
	if (status != 0) {
	    MP_FATAL("Could not create Mindprobe pipe %s: %s", pipe_name, strerror(-status));
	}
#endif
    }

#ifdef DEBUG_MP
    MP_add_probe_int32("mp.test_messages", &test_messages);
    for (client_id = 0; client_id < MP_MAX_RT_CLIENTS; client_id++) {
	char probe_name[32];

	snprintf(probe_name, sizeof(probe_name), "mp.skip_probes[%d]", client_id);
	MP_add_probe_int32(probe_name, &skip_probes[client_id]);
    }
#ifdef NOTYET
    MP_add_probe_var("mp.test_string", MP_PROBE_TYPE_STRING,
		     sizeof(test_string), &test_string);
#endif
#endif    
}

void mp_set_ready (int status) {
    mp_ready = status;
}

int mp_is_ready (void)
{
    return mp_ready;
}


/*
 * Form the probe data message to send to the Mindprobe client.
 * Walk the probe table, and for each enabled probe, read the
 * probe value and encode it into a TLV sub-message.
 */
static void encode_probes(int client_id, struct UTIL_tlv_msg_buf *tlv_buf)
{
    struct UTIL_tlv_desc tlv_desc;
    int probe_id;
    void *data;
    unsigned len;
    int status;

    /* Begin recusive TLV encoding, top-level type is PROBE_DATA. */
    UTIL_tlv_start_write(&tlv_desc, tlv_buf, sizeof(tlv_buf->v),
			 MP_TLV_PROBE_DATA);

    /* First item is current tick. */
    uint32_t tick = mp_get_current_tick();
    UTIL_tlv_write(&tlv_desc,
		   MP_TLV_CURRENT_TICK, sizeof(uint32_t), &tick);

    /* Following items are probe values... */
    probe_id = MP_NULL_PROBE_ID;

    while ((probe_id = mp_prepare_next_enabled_probe(client_id, probe_id, &data, &len)) != 
	   MP_NULL_PROBE_ID) {

	UTIL_tlv_write(&tlv_desc, probe_id, len, data);
    }

    /* Finish recursive encoding. */
    status = UTIL_tlv_finish_write(&tlv_desc);

    /* If we have attempted to write too many things into the buffer,
       we'll drop past the last one that fit.  Generate a warning, but
       pass the other data onto the client.  */

    if (status != 0) {
	MP_WARNING("probes overflowed message buffer");
    }
}

static void do_write_probes(struct UTIL_tlv_msg_buf *tlv_buf)
{
    struct UTIL_tlv_desc tlv_desc;
    int l;
    void *v;
    int probe_id;

    UTIL_tlv_start_read(&tlv_desc, tlv_buf);
    
    /* For probe write message, we take the type to be the probe ID */
    while (UTIL_tlv_read(&tlv_desc, &probe_id, &l, &v) == 0) {

	/* write the probe value */
	MP_write_probe(probe_id, v, l, "mp");
    }
}

#ifndef MP_EMBEDDED
void mp_rt_process_connection(struct UTIL_tlv_msg_buf *tlv_buf)
{
    struct UTIL_tlv_msg_buf tlv_reply_buf;
    struct UTIL_tlv_desc tlv_reply_desc;
    int32_t status;
    char *pipe_name = NULL;
    int client_id;

    MP_WARNING("processing connection request\n");

    if (tlv_buf->t != MP_INT_TLV_CONNECT) {
	MP_WARNING("Received unexpected TLV type %d on connection pipe\n", tlv_buf->t);
	return;
    }

    /* Find an empty client slot. */
    for (client_id = 0; client_id < MP_MAX_RT_CLIENTS; client_id++) {
	if (clients[client_id].pipe_opened == 0)
	    break;
    }

    /* Process the connect request. */
    if (client_id >= MP_MAX_RT_CLIENTS) {
	/* Too many clients currently connected. */
	status = ECONNREFUSED;
	MP_WARNING("connect failed - no free client slots\n");
    } else {
	pipe_name = clients[client_id].pipe_name;

	status = 0;
	clients[client_id].pipe_opened = 1;

	MP_WARNING("connected as client%d\n", client_id);
    }

    /* Build and send reply. */
    UTIL_tlv_start_write(&tlv_reply_desc, &tlv_reply_buf, sizeof(tlv_reply_buf.v), MP_INT_TLV_CONNECT_REPLY);
    UTIL_tlv_write(&tlv_reply_desc, MP_INT_TLV_CONNECT_RESULT, sizeof(status), &status);
    if (status == 0) {
	UTIL_tlv_write(&tlv_reply_desc, MP_INT_TLV_PIPE_NAME, strlen(pipe_name) + 1, pipe_name);
    }
    UTIL_tlv_finish_write(&tlv_reply_desc);

    RTS_rt_pipe_write_message(&connection_pipe, &tlv_reply_buf, tlv_reply_buf.l + UTIL_TLV_HEADER_LEN);

    MP_WARNING("connect reply written back to client\n");
}

static void mp_rt_close_client(int client_id)
{
    clients[client_id].should_be_closed = 1;
}
#endif



static void send_client_protocol_version(int client_id)
{
    uint16_t protocol_version = MP_PROTOCOL_VERSION;

    send_client_tlv(client_id, MP_TLV_PROTOCOL_VERSION, sizeof(protocol_version),
		    &protocol_version);
}

static void send_client_build_version(int client_id)
{
    struct UTIL_tlv_msg_buf msg_buf;
#ifdef MP_EMBEDDED
    static char process_build_info[SHA1_DIGEST_LENGTH*2 + 1];
    static int build_info_initialized = 0;

    if (!build_info_initialized) {
	int i;
	extern uint8_t sha_digest[SHA1_DIGEST_LENGTH];

	for (i=0; i<SHA1_DIGEST_LENGTH; i++) {
	    sprintf(process_build_info + i*2, "%02x", sha_digest[i]);
	}
	build_info_initialized = 1;
    }
#else
    static char *process_build_info = HAI_BUILD_LABEL_TEXT;
#endif

    prepare_debug_message(&msg_buf, "\n\n\n\n");
    send_tlv(client_id, &msg_buf);
    prepare_debug_message(&msg_buf, "Connected to MindProbe server");
    send_tlv(client_id, &msg_buf);
    prepare_debug_message(&msg_buf, "MindProbe server build info: %s",
                          process_build_info);
    send_tlv(client_id, &msg_buf);
}

#ifndef MP_EMBEDDED
static void send_client_probe_discovery(int client_id)
{
    if (!clients[client_id].sending_probe_discovery) {
	clients[client_id].sending_probe_discovery = 1;
	clients[client_id].probe_discovery_index = 0;
    }
}

#define MAX_NAME_LEN 256
static void do_send_client_probe_discovery(int client_id)
{
    struct UTIL_tlv_msg_buf msg_buf;
    struct UTIL_tlv_desc tlv_desc;
    enum MP_probe_type type;
    unsigned length;
    const char *name;
    int tlv_len, status;
    int probe_id;

    struct {
	uint16_t id;
	uint16_t type;
	uint16_t length;
	char name[MAX_NAME_LEN];
    } probe_def;
    
    int sent_empty_terminator = 1;

    if (!clients[client_id].sending_probe_discovery)
	return;

    probe_id = clients[client_id].probe_discovery_index;

    /* Begin recursive TLV encoding, top-level type is DISCOVER_PROBES.
     *
     * NOTE: we purposely do not try to fill the buffer - it is likely
     * there is not enough room in the pipe, so send more smaller
     * packets to increase the chances of success each tick.
     */
    UTIL_tlv_start_write(&tlv_desc, &msg_buf, sizeof(msg_buf.v)/4, MP_TLV_DISCOVER_PROBES);
    while (MP_get_probe_info(probe_id, &type, &length, &name) == 0) {
	/* Is there space remaining for this probe_def? */
	if( tlv_desc.remaining <= sizeof( probe_def ) ) {
#ifndef MP_EMBEDDED
	    /* no space - carry over into the next DISCOVER_PROBES response */
	    RCP_log( LOG_WARNING, "DISCOVER_PROBES response exceeded a single TLV." );
#endif
	    break;
	}
        
	probe_def.id = probe_id;
	probe_def.type = type;
	probe_def.length = length;
        
	/* Copy the name, truncating if necessary. */
	strncpy(probe_def.name, name, MAX_NAME_LEN);
	probe_def.name[MAX_NAME_LEN - 1] = '\0';
	
	/* Length of probe def in TLV includes ID, type, length, and
	   name string (including null terminator). */
	tlv_len = 3 * sizeof(uint16_t) + strlen(probe_def.name) + 1;
        
	UTIL_tlv_write(&tlv_desc, MP_TLV_PROBE_DEF, tlv_len, &probe_def);
	sent_empty_terminator = 0;
        
	probe_id++;
    }
        
    /* Finish recursive encoding. */
    status = UTIL_tlv_finish_write(&tlv_desc);
        
    /* Warn if we truncated the encoding (dropping probe defs from the
       end): */
    if (status != 0) {
	MP_WARNING("probe discovery overflowed message buffer\n");
    }
        
    /* Now write the whole message buffer out to the client. */
    if (send_tlv(client_id, &msg_buf) == 0) {
	/* Send succeeded - advance state machine. */
	if (sent_empty_terminator) {
	    clients[client_id].sending_probe_discovery = 0;
	} else {
	    clients[client_id].probe_discovery_index = probe_id;
	}
    } else {
	/* Send failed - leave state machine in this state so we can
	 * retry this on the next tick.
	 */
    }
}
#endif

static void send_client_hz(int client_id)
{
    uint16_t hz = MP_HZ;

    send_client_tlv(client_id, MP_TLV_HZ, sizeof(hz), &hz);
}

void mp_rt_process_command(struct UTIL_tlv_msg_buf *tlv_buf, int client_id)
{
    uint16_t t;
    int l;
    void *v;
    uint16_t probe_id;

    t = tlv_buf->t;
    l = tlv_buf->l;
    v = tlv_buf->v;

    switch (t) {
    case MP_TLV_NULL:
	break;
    case MP_TLV_CONNECTED:
	send_client_protocol_version(client_id);
	send_client_build_version(client_id);
        send_client_host_info(client_id);
	break;
    case MP_TLV_PROTOCOL_VERSION:
	send_client_protocol_version(client_id);
	break;
#ifndef MP_EMBEDDED
    case MP_TLV_DISCOVER_PROBES:
	send_client_probe_discovery(client_id);
	break;
#endif
    case MP_TLV_HZ:
	send_client_hz(client_id);
	break;
    case MP_TLV_DISCONNECTED:
	mp_disable_all_probes(client_id);
	clients[client_id].probing = 0;
	break;
    case MP_TLV_ENABLE_PROBES:
	/* Disable previous set. */
	mp_disable_all_probes(client_id);

	/* v is array of probe IDs */
	while (l > 0) {
	    memcpy(&probe_id, v, sizeof(uint16_t));
	    mp_enable_probe(client_id, probe_id, 1);

	    v += sizeof(uint16_t);
	    l -= sizeof(uint16_t);
	}
        mp_update_enabled_probes(client_id);
	break;
    case MP_TLV_START_PROBES:
	clients[client_id].probing = 1;
	break;
    case MP_TLV_STOP_PROBES:
	clients[client_id].probing = 0;
	break;
    case MP_TLV_WRITE_PROBES:
	do_write_probes(tlv_buf);
	break;
    case MP_INT_TLV_KEEPALIVE:
	/* Send it right back. */
	send_tlv(client_id, tlv_buf);
	break;
    default:
	MP_WARNING("unknown message t = %d\n", t);
	/* ignore things we don't know about */
	break;
    }
}

#ifdef MP_EMBEDDED
/* Stub out RTS_profiling_{begin,end}_event() */
#define RTS_profiling_begin_event(a)
#define RTS_profiling_end_event(a)
#endif

static int send_tlv(int client_id, struct UTIL_tlv_msg_buf *tlv_buf)
{
#ifdef MP_EMBEDDED
    return mp_send_client(tlv_buf);
#else
    if (mp_client_is_alive(client_id)) {
        return RTS_rt_pipe_write_message(&clients[client_id].pipe, tlv_buf,
                                         tlv_buf->l + UTIL_TLV_HEADER_LEN) == 0;
    } else {
        return 0; /* XXX Should that be -EINVAL or something? */
    }
#endif
}

static int send_client_tlv(int client_id, enum mp_tlv_type t, unsigned int l, void *v)
{
    struct UTIL_tlv_msg_buf msg_buf;

    MP_ASSERT(l < UTIL_TLV_LEN_LIMIT);
    msg_buf.t = t;
    msg_buf.l = l;
    if (l) memcpy(msg_buf.v, v, l);

    return send_tlv(client_id, &msg_buf);
}

/*
 * MP_do_tick(): called at the end of every tick to do probing.
 *
 * Reads commands from the Mindprobe client (via the listener),
 * updates probing state, sends probe data to the client.
 */
void MP_do_tick()
{
#ifndef MP_EMBEDDED
    int status;
#endif
    struct UTIL_tlv_msg_buf tlv_buf;
    int client_id;

#ifdef MP_EMBEDDED
    /* In the embedded version, all processing happens in this tick,
     * so we first check the receive buffer for any messages from the
     * mindprobe client application.  In the RCP version, this portion
     * is handled in the non-real-time server thread.
     */
    while (mp_client_receive_ready())
	mp_receive_from_client();
#endif

#if DEBUG_MP
    if (test_messages) {
	MP_puts("Lorem ipsum dolor sit amet, consectetur adipisicing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.\n");
	test_messages--;
    }
#endif

#ifndef MP_EMBEDDED
    RTS_profiling_begin_event(RTS_EVENT_MP_TICK_RECEIVE);

    /* Process any connection requests. */
    while (1) {
	status = RTS_rt_pipe_read_message(&connection_pipe, &tlv_buf,
					  sizeof(tlv_buf));

	/* Stop if there is nothing to read. */
	if (status == 0)
	    break;

	MP_ASSERT(status == UTIL_TLV_HEADER_LEN + tlv_buf.l);

	mp_rt_process_connection(&tlv_buf);
    }

    /* Process all incoming commands from the Mindprobe clients. */
    for (client_id = 0; client_id < MP_MAX_RT_CLIENTS; client_id++) {
	while (mp_client_is_alive(client_id)) {
	    status = RTS_rt_pipe_read_message(&clients[client_id].pipe, &tlv_buf,
					      sizeof(tlv_buf));

	    /* Stop if there is nothing to read. */
	    if (status == 0)
		break;

	    MP_ASSERT(status == UTIL_TLV_HEADER_LEN + tlv_buf.l);

	    mp_rt_process_command(&tlv_buf, client_id);
	}
    }
    RTS_profiling_end_event(RTS_EVENT_MP_TICK_RECEIVE);
#endif

    /* If there were missed messages or missed probes, attempt to notify
       the Mindprobe client of that. */
    RTS_profiling_begin_event(RTS_EVENT_MP_MISSED_MESSAGES);
    for (client_id = 0; client_id < MP_MAX_RT_CLIENTS; client_id++) {
	if (clients[client_id].missed_messages) {
	    if (send_dropped_count(client_id, clients[client_id].missed_messages,
				   MP_TLV_MISSED_DEBUG_MESSAGES) == 0) {
		clients[client_id].missed_messages = 0;
	    }
	}
    }
    RTS_profiling_end_event(RTS_EVENT_MP_MISSED_MESSAGES);

#ifdef DEBUG_MP
    for (client_id = 0; client_id < MP_MAX_RT_CLIENTS; client_id++) {
	if (clients[client_id].probing && skip_probes[client_id]) {
	    /* pretend the pipe was full */
	    clients[client_id].missed_probe_updates++;
	    skip_probes[client_id]--;
	    return;
	}
    }
#endif

    RTS_profiling_begin_event(RTS_EVENT_MP_MISSED_PROBE_UPDATES);
    for (client_id = 0; client_id < MP_MAX_RT_CLIENTS; client_id++) {
#ifndef MP_EMBEDDED
	if (!mp_client_is_alive(client_id)) continue;
	if (clients[client_id].missed_probe_updates > MAX_MISSED_PROBE_UPDATES) {
	    MP_WARNING("Closing client %d due to missed_probe_updates exceeding MAX_MISSED_PROBE_UPDATES", client_id);
	    mp_rt_close_client(client_id);
	    continue;
	}
#endif
        if (clients[client_id].missed_probe_updates
	    && send_dropped_count(client_id, clients[client_id].missed_probe_updates, MP_TLV_MISSED_DATA) == 0) {
	    clients[client_id].missed_probe_updates = 0;
	}
    }
    RTS_profiling_end_event(RTS_EVENT_MP_MISSED_PROBE_UPDATES);

    /* If probing is enabled, attempt to send the probed data (as a single data
       block). */
    RTS_profiling_begin_event(RTS_EVENT_MP_TICK_PROBES);
    for (client_id = 0; client_id < MP_MAX_RT_CLIENTS; client_id++) {
#ifndef MP_EMBEDDED
	if (clients[client_id].sending_probe_discovery) {
	    do_send_client_probe_discovery(client_id);
	}
#endif
	if (clients[client_id].probing) {

	    /* If we could not send the missed data message, don't
	       attempt to send this probe data.  Otherwise the
	       missed-data message could come out of order.  */

	    if (clients[client_id].missed_probe_updates) {
		clients[client_id].missed_probe_updates++;
	    } else {
		encode_probes(client_id, &tlv_buf);

		if (send_tlv(client_id, &tlv_buf)) {
		    /* pipe is full */
		    clients[client_id].missed_probe_updates++;
		}
	    }
	}
    }
    RTS_profiling_end_event(RTS_EVENT_MP_TICK_PROBES);
}

/*
 * Prepare a text message to be sent to mindprobe.
 *
 * The message is encoded in a TLV with type MP_TLV_DEBUG_MESSAGE.
 */
int prepare_debug_message_va(struct UTIL_tlv_msg_buf *tlv_buf, const char *message, va_list *app) {
    struct UTIL_tlv_desc tlv_desc;
    char *message_text;
    int buffer_size, text_size;
    int status;

    UTIL_tlv_start_write(&tlv_desc, tlv_buf, sizeof(tlv_buf->v),
			 MP_TLV_DEBUG_MESSAGE);

    /* First item is current tick. */
    uint32_t tick = mp_get_current_tick();
    UTIL_tlv_write(&tlv_desc,
		   MP_TLV_CURRENT_TICK, sizeof(uint32_t), &tick);

    /* Next item is the message text, which we'll sprintf directly
       into the TLV buffer. */
    message_text = UTIL_tlv_write_loc(&tlv_desc, &buffer_size);

    if (app) {
	text_size = vsnprintf(message_text, buffer_size, message, *app);

	if (text_size < 0) return text_size;  /* printf error */

	text_size += 1;  /* include null terminator */

	if (text_size >= buffer_size) {
	    /* message was truncated, force null-termination */
	    message_text[buffer_size - 1] = '\0';
	    text_size = buffer_size;
	}
    } else {
	/* Called from MP_puts/do_printf()... */

	text_size = strlen(message);
	if (text_size >= buffer_size) {
	    text_size = buffer_size - 1;
	}
	memcpy(message_text, message, text_size);
	message_text[text_size] = '\0';  /* force null termination */
	text_size += 1;  /* include null terminator */
    }

    UTIL_tlv_write(&tlv_desc, MP_TLV_MESSAGE_TEXT, text_size, NULL);

    status = UTIL_tlv_finish_write(&tlv_desc);

    MP_ASSERT(status == 0);  /* size is limited by our message_text size */
    
    return text_size;
}

/*
 * Prepare a text message to be sent to mindprobe.
 *
 * Delegates to prepare_debug_message_va above.
 */
int prepare_debug_message(struct UTIL_tlv_msg_buf *tlv_buf, const char *message, ...) {
    va_list ap;
    int status;

    va_start(ap, message);
    status = prepare_debug_message_va(tlv_buf, message, &ap);
    va_end(ap);
    return status;
}

/*
 * Send a text message to the Mindprobe client.
 *
 * The message gets written to the MP listener pipe immediately,
 * If the pipe is full, then we drop the message.  We will report
 * a count of dropped messages to the MP client before another
 * message is sent.
 */
int MP_puts(const char *message)
{
    struct UTIL_tlv_msg_buf msg_buf;
    int client_id;

    prepare_debug_message_va(&msg_buf, message, NULL);

    for (client_id = 0; client_id < MP_MAX_RT_CLIENTS; client_id++) {
	if (clients[client_id].missed_messages) {
	    /* Attempt to send a dropped count message.  If that is
	       not sent successfully, don't attempt to send this
	       message. */

	    if (send_dropped_count(client_id, clients[client_id].missed_messages,
				   MP_TLV_MISSED_DEBUG_MESSAGES) == 0) {
		clients[client_id].missed_messages = 0;
	    } else {
		clients[client_id].missed_messages++;
		continue;
	    }
	}

	if (send_tlv(client_id, &msg_buf)) {
	    /* pipe is full */
	    clients[client_id].missed_messages++;
	}
    }
    return 0;
}

/*
 * Send a text message to the Mindprobe client.
 *
 * The message gets written to the MP listener pipe immediately,
 * If the pipe is full, then we drop the message.  We will report
 * a count of dropped messages to the MP client before another
 * message is sent.
 */
void MP_printf (const char *format, ...)
{
    va_list ap;

    va_start (ap, format);
    MP_vprintf (format, ap);
    va_end (ap);
}

void MP_vprintf (const char *format, va_list ap)
{
#ifndef MP_EMBEDDED
    RCP_logv (LOG_DEBUG, format, ap);
#else /* MP_EMBEDDED */
    struct UTIL_tlv_msg_buf msg_buf;
    int client_id;

    prepare_debug_message_va(&msg_buf, format, &ap);

    for (client_id = 0; client_id < MP_MAX_RT_CLIENTS; client_id++) {
	if (clients[client_id].missed_messages) {
	    /* Attempt to send a dropped count message.  If that is
	       not sent successfully, don't attempt to send this
	       message. */

	    if (send_dropped_count(client_id, clients[client_id].missed_messages,
				   MP_TLV_MISSED_DEBUG_MESSAGES) == 0) {
		clients[client_id].missed_messages = 0;
	    } else {
		clients[client_id].missed_messages++;
		continue;
	    }
	}

	if (send_tlv(client_id, &msg_buf)) {
	    /* pipe is full */
	    clients[client_id].missed_messages++;
	}
    }
#endif
}

/*
 * Send a message to the MP client indicating that either messages
 * or probe updates were not sent due to a full pipe.  If the
 * message gets sent, clear the drop counter.  We do this before
 * sending other messages, so that the MP client knows that there
 * was something skipped.
 */
static int send_dropped_count(int client_id, int count, enum mp_tlv_type t)
{
    struct UTIL_tlv_msg_buf msg_buf;

    msg_buf.t = t;
    msg_buf.l = sizeof(uint32_t);
    memcpy(msg_buf.v, &count, sizeof(count));

    return send_tlv(client_id, &msg_buf);
}

static char mp_hostname[128] = "";
static char mp_nickname[128] = "";

void MP_set_host_info(char const *hostname, char const *nickname) {
    strncpy( mp_hostname, hostname, sizeof( mp_hostname ) );
    strncpy( mp_nickname, nickname, sizeof( mp_nickname ) );
}

void send_client_host_info(int client_id)
{
    struct UTIL_tlv_msg_buf msg_buf;

    if( mp_hostname[0] ) {
        prepare_debug_message(&msg_buf, "Server hostname: %s", mp_hostname);
        send_tlv(client_id, &msg_buf);
    }
    if( mp_nickname[0] ) {
        prepare_debug_message(&msg_buf, "Robot nickname: %s", mp_nickname);
        send_tlv(client_id, &msg_buf);
    }
    if( mp_hostname[0] ) {
	msg_buf.t = MP_TLV_HOSTNAME;
	msg_buf.l = strlen(mp_hostname)+1;
	memcpy(msg_buf.v, mp_hostname, msg_buf.l);
	send_tlv(client_id, &msg_buf);
    }
    if( mp_nickname[0] ) {
	msg_buf.t = MP_TLV_NICKNAME;
	msg_buf.l = strlen(mp_nickname)+1;
	memcpy(msg_buf.v, mp_nickname, msg_buf.l);
	send_tlv(client_id, &msg_buf);
    }
}
