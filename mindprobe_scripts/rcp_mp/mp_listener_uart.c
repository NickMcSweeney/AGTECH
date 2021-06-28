

/*
 * mp_listener_uart.c -- Mindprobe listener.
 *
 * A simplified mindprobe listener interface for an embedded
 * microcontroller that listens on a UART.
 */

#include <stdio.h>
#include <stdint.h>

#include "mp_internal.h"

#include "util/util.h"

static struct mp_ops *mp_ops = NULL;

void mp_close_client(void)
{
    /* Nothing to do in UART version. */
}

static int frame_and_byte_stuff(uint8_t *src, int src_len, uint8_t *dst, int dst_len)
{
    uint8_t *src_end = src + src_len;
    uint8_t *dst_orig = dst;
    uint8_t *dst_end = dst + dst_len;

    /* Start with the sync byte. */
    if ((dst + 1) > dst_end)
	return -1;
    *dst++ = MP_SYNC;

    /* Walk through the source one byte at a time. */
    while (src < src_end) {
	if ((*src == MP_SYNC) || (*src == MP_ESCAPE)) {
	    if ((dst + 2) > dst_end)
		return -1;
	    *dst++ = MP_ESCAPE;
	    *dst++ = (*src++) ^ MP_ESCAPE;
	} else {
	    if ((dst + 1) > dst_end)
		return -1;
	    *dst++ = *src++;
	}
    }

    return dst - dst_orig;
}

int mp_send_client(struct UTIL_tlv_msg_buf *tlv_buf) {
    int l;
    static uint8_t outbuf[sizeof(struct UTIL_tlv_msg_buf) * 2 + 1];

    if (!mp_ops)
	return -1;

    l = frame_and_byte_stuff((uint8_t *)tlv_buf, tlv_buf->l + UTIL_TLV_HEADER_LEN, outbuf, sizeof(outbuf));
    if ((l < 0) || (mp_ops->uart_write_room() < l))
	return -1;
    return mp_ops->uart_write(outbuf, l) != l;
}

int mp_client_read(void *bufp, int len)
{
    int nread = 0;
    char in;
    char *buf = bufp;
    static int escape_next = 0;

    if (!mp_ops)
	return 0;

    while (nread < len) {
	if (mp_ops->uart_read(&in, 1) == 0)
	    return nread;

	if (in == MP_SYNC) {
	    return -1; /* Tell caller to restart reading TLV. */
	} else if (escape_next) {
	    *buf++ = in ^ MP_ESCAPE;
	    nread++;
	    escape_next = 0;
	} else if (in == MP_ESCAPE) {
	    escape_next = 1;
	} else {
	    *buf++ = in;
	    nread++;
	}
    }

    return nread;
}

void mp_handle_command(struct UTIL_tlv_msg_buf *tlv_buf)
{
    /* In the embedded version it's all realtime, so we process the
     * command inline rather than sending it through a pipe.
     */
    mp_rt_process_command(tlv_buf, 0);
}

uint32_t mp_get_current_tick(void)
{
    if (mp_ops)
	return mp_ops->get_current_tick();
    else
	return 0;
}

void mp_fatal_error(const char *file, int line, const char *format, ...)
{
    va_list ap;

    va_start (ap, format);
    MP_vprintf (format, ap);
    va_end (ap);

    if (mp_ops)
	mp_ops->fatal_error(file, line);
    else
	while (1);
}

int mp_client_receive_ready(void)
{
    if (!mp_ops)
	return 0;

    return mp_ops->uart_read_ready();
}

void MP_init()
{
    mp_realtime_init();  /* create pipes */
}

void MP_begin(struct mp_ops *ops)
{
    mp_ops = ops;

    mp_set_ready(1);
}


