

/*
 * mp_listener_common -- Mindprobe listener common routines
 *
 * Contains routines that are shared by mp_listener_tcp.c and
 * mp_listener_uart.c
 */

#include <stdio.h>
#include <stdint.h>

#ifndef MP_EMBEDDED
#include <syslog.h>
#include "log/log.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/fcntl.h>
#include <unistd.h>
#endif

#include "hai_build_version.h"
#ifdef MP_EMBEDDED
#undef HAI_BUILD_LABEL_PATH
#endif

#include "mp_internal.h"

#include "hwspecs.h"
#include "util/util.h"


/* Input bufer, data received from the MP client: */
static struct UTIL_tlv_msg_buf client_input_buf;
static int client_input_pos;

static void send_client_or_close(struct UTIL_tlv_msg_buf *tlv_buf) {
    if (mp_send_client(tlv_buf)) {
	/* Error sending to the client.  Close the connection and
	   clean up. */
	mp_close_client();
	MP_WARNING("Mindprobe client write failed\n");
    }
}

static void send_client_tlv(enum mp_tlv_type t, unsigned int l, void *v)
{
    struct UTIL_tlv_msg_buf msg_buf;

    MP_ASSERT(l < UTIL_TLV_LEN_LIMIT);
    msg_buf.t = t;
    msg_buf.l = l;
    if (l) memcpy(msg_buf.v, v, l);

    send_client_or_close(&msg_buf);
}

void mp_send_client_protocol_version(void)
{
    uint16_t protocol_version = MP_PROTOCOL_VERSION;

    send_client_tlv(MP_TLV_PROTOCOL_VERSION, sizeof(protocol_version),
		    &protocol_version);
}

void mp_send_client_build_version(void)
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
#ifdef HAI_BUILD_LABEL_PATH
    static char sys_buildinfo[512];
    static char *sys_buildinfo_p = NULL;
#endif
#endif

    prepare_debug_message(&msg_buf, "\n\n\n\n");
    send_client_or_close(&msg_buf);
    prepare_debug_message(&msg_buf, "Connected to MindProbe server");
    send_client_or_close(&msg_buf);
    prepare_debug_message(&msg_buf, "MindProbe server build info: %s",
                          process_build_info);
    send_client_or_close(&msg_buf);

#if defined(HAI_BUILD_LABEL_PATH) && !defined(MP_EMBEDDED)
    if (sys_buildinfo_p == NULL) {
        int buildinfo_fd;
        ssize_t nbytes_read;

        sys_buildinfo_p = sys_buildinfo;

        if ((buildinfo_fd = open (HAI_BUILD_LABEL_PATH, O_RDONLY)) < 0) {
            goto done;
        }

        /* Read until EOF or until our buffer is full--
           whichever comes first: */
        while ((nbytes_read = read(buildinfo_fd, sys_buildinfo_p,
                                   sizeof(sys_buildinfo)
                                   - (sys_buildinfo_p - sys_buildinfo)
                                   - 1))
               > 0)
        {
            sys_buildinfo_p += nbytes_read;
        }

        close(buildinfo_fd);

    done:
        *sys_buildinfo_p = '\0';
    }

    prepare_debug_message(&msg_buf, "System build info: %s", sys_buildinfo);
    send_client_or_close(&msg_buf);
#endif
}

#ifndef MP_HZ
#define MP_HZ HW_TICK_HZ
#endif

static void send_client_hz(void)
{
    uint16_t hz = MP_HZ;

    send_client_tlv(MP_TLV_HZ, sizeof(hz), &hz);
}

#define MAX_NAME_LEN 256
static void send_client_probe_discovery(void)
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
    
    int sent_empty_terminator = 0;
    probe_id = 0;
    do {
        /* Begin recusive TLV encoding, top-level type is DISCOVER_PROBES. */
        UTIL_tlv_start_write(&tlv_desc, &msg_buf, sizeof(msg_buf.v), MP_TLV_DISCOVER_PROBES);
        sent_empty_terminator = 1;
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
        tlv_len = UTIL_TLV_HEADER_LEN + msg_buf.l;
        
        send_client_or_close(&msg_buf);
    } while( !sent_empty_terminator );
}

void mp_reset_client_input(void)
{
    client_input_pos = 0;
}

int mp_receive_from_client(void)
{
    int n, read_len;

    /*
     * The amount available may not be a complete message, and we
     * don't want to read beyond this message (lest our
     * client_input_buf be out of whack), so we first read the T and L
     * fields and then read the V data.
     */
    if (client_input_pos < UTIL_TLV_HEADER_LEN) {
	read_len = UTIL_TLV_HEADER_LEN - client_input_pos;
    } else {
	read_len = client_input_buf.l + UTIL_TLV_HEADER_LEN - client_input_pos;
    }

    n = mp_client_read((void *) &client_input_buf + client_input_pos, read_len);

    if (n < 0) {
#ifdef MP_EMBEDDED
	/* This indicates we should restart reading a TLV over from the beginning due to a framing error. */
	client_input_pos = 0;
#else
	MP_WARNING("Mindprobe client read failed\n");
#endif
    }

    if (n <= 0) {
	/* Client closed the connection, or error.  Notify caller. */
	return -1;
    }

    client_input_pos += n;

    if ((client_input_pos < UTIL_TLV_HEADER_LEN) ||
	(client_input_pos != client_input_buf.l + UTIL_TLV_HEADER_LEN)) {

	return 0;  /* message not yet complete */
    }

    /* printf("sending received message from client: "); */
    /* dump_tlv(&client_input_buf); */

    switch(client_input_buf.t) {

	/* Some messages are handled locally by the listener: */

    case MP_TLV_CONNECTED:
	mp_send_client_protocol_version();
	mp_send_client_build_version();
	break;
    case MP_TLV_PROTOCOL_VERSION:
	mp_send_client_protocol_version();
	break;
    case MP_TLV_DISCOVER_PROBES:
	send_client_probe_discovery();
	break;
    case MP_TLV_HZ:
	send_client_hz();
	break;

	/* Other messages are handled in mp_realtime.c. */

    default:

	/* Pass this message through the pipe.  This may block until
	   there is space available in the pipe. */

	mp_handle_command(&client_input_buf);
	break;
    }
    client_input_pos = 0;

    return 0;
}

