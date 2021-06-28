

/*
 * mp_listener_tcp.c -- Mindprobe listener.
 *
 * The Mindprobe listener is a thread that listens on a socket, allows
 * a Mindprobe client to connect, and realays messages between the
 * client and the RCP via a Xenomai pipe.
 *
 * The socket bits of code were originally cribbed from the Comer
 * examples.  Copyright notice for that at the end of this file.
 */ 

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stddef.h>

#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sched.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <syslog.h>
#ifndef BTB
#include <rtdk.h>
#endif

#include <netinet/in.h>
#include <netinet/tcp.h>

#include <netdb.h>

#include <pthread.h>

#include "mp_internal.h"

#include "util/util.h"

#define CLIENT_NAME "mp_listener_tcp"

#define MAX_UNANSWERED_KEEPALIVES 5

static int server_fd;  /* bound to TCP port accepting connections */
static int realtime_pipe_connected;
static RTS_pipe realtime_pipe; /* the Xenomai pipe communicating with RCP */
static int realtime_read_fd;   /* the associated read FD */
static int client_fd;  /* the Mindprobe client socket, -1 if not connected */

static void mp_realtime_disconnect(void);

__attribute__((unused))  /* allow this to be unreferenced */
static void dump_tlv(struct UTIL_tlv_msg_buf *mb)
{
    int i;
    printf("t = %d, l = %d, v = ", mb->t, mb->l);
    for(i = 0; i < mb->l; ++i)
    {
	printf("0x%02x", mb->v[i]);
	if(i < mb->l - 1) printf(":");
    }
    printf("\n");
}


static void send_rcp_tlv(enum mp_tlv_type t, unsigned int l, void *v)
{
    struct UTIL_tlv_msg_buf msg_buf;

    MP_ASSERT(l < UTIL_TLV_LEN_LIMIT);
    msg_buf.t = t;
    msg_buf.l = l;
    if (l) memcpy(msg_buf.v, v, l);

    l += UTIL_TLV_HEADER_LEN;

    /* printf("sending to rcp: "); */
    /* dump_tlv(&msg_buf); */

    if (RTS_pipe_write_message_timeout(realtime_pipe, &msg_buf, l, 5000000) != l) {
	MP_WARNING("Mindprobe pipe write failed\n");
	mp_realtime_disconnect();
    }
}

static int open_server_socket(int port, int qlen)
{
    struct sockaddr_in sin;	/* an Internet endpoint address		*/
    int	s;	/* socket file descriptor */
    int on;

    memset(&sin, 0, sizeof(sin));
    sin.sin_family = AF_INET;
    sin.sin_addr.s_addr = INADDR_ANY;
    sin.sin_port = htons((unsigned short) port);


    /* Allocate a socket */
    s = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s < 0) {
	MP_FATAL("Could not allocate server socket: %s\n", strerror(errno));
    }

    on = 1;
    if (setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) < 0) {
	MP_FATAL("Could not set reuseaddr socket option: %s\n", strerror(errno));
    }

    /* Bind the socket */
    if (bind(s, (struct sockaddr *)&sin, sizeof(sin)) < 0) {
	MP_FATAL("Could not bind to port %d: %s\n", port, strerror(errno));
    }
    if (listen(s, qlen) < 0) {
	MP_FATAL("Could not listen on port %d: %s\n", port, strerror(errno));
    }
    return s;
}

static void accept_client(void)
{
    struct sockaddr_in fsin;	/* the from address of a client	*/
    unsigned int alen;		/* from-address length		*/
    int	new_client_fd;

    alen = sizeof(fsin);
    new_client_fd = accept(server_fd, (struct sockaddr *)&fsin, &alen);

    if (new_client_fd >= 0) {
	client_fd = new_client_fd;

	/* Tell the RCP that a client has connected. */
	send_rcp_tlv(MP_TLV_CONNECTED, 0, NULL);
    }
}

void mp_close_client(void)
{
    if (client_fd != -1) {
	/* kill existing client */
	close(client_fd);
	client_fd = -1;

	/* Tell the RCP that the client disconnected. */
	send_rcp_tlv(MP_TLV_DISCONNECTED, 0, NULL);
    }
}

static void mp_close_server_socket(void)
{
    if (server_fd != -1) {
        close(server_fd);
        server_fd = -1;
    }
}


ssize_t send_with_timeout(int fd, const void *buffer, size_t length, struct timeval *timeout)
{
    struct timeval start, now, delta;
    ssize_t retval, nwritten = 0;

    gettimeofday(&start, NULL);

    do {
	retval = send(fd, buffer, length, MSG_DONTWAIT);

	if (retval > 0) {
	    nwritten += retval;
	    buffer += retval;
	    length -= retval;
	    if (length == 0)
		break;
	}

	usleep(1000);

	gettimeofday(&now, NULL);
	timersub(&now, &start, &delta);
    } while (timercmp(&delta, timeout, <));

    if (length == 0)
	retval = nwritten;
    else
	retval = -1;

    return retval;
}

int mp_send_client(struct UTIL_tlv_msg_buf *tlv_buf) {
    int l;
    struct timeval timeout = { 10 , 0 };
    
    if (client_fd == -1)
	return -1;
    
    l = tlv_buf->l + UTIL_TLV_HEADER_LEN;
    return send_with_timeout(client_fd, tlv_buf, l, &timeout) != l;
}

int mp_client_read(void *buf, int len)
{
    return read(client_fd, buf, len);
}

static void receive_from_pipe(void)
{
    struct UTIL_tlv_msg_buf output_buf;
    int n;

    /* Message from the pipe.  These messages come complete, and read
       one message at a time (Xenomai pipe semantics). */

    /* To test overflow, I uncomment the line below, which causes the listener
       thread to sleep for 20 ms. after reading a message. */
    /* usleep(20000); */

    n = RTS_pipe_read_message(realtime_pipe, &output_buf, sizeof(output_buf));
    if (n <= 0) {
	MP_WARNING("Mindprobe read pipe failed: n=%d %s\n", (int)n, strerror(errno));
	mp_realtime_disconnect();
	return;
    }

    if ((n < UTIL_TLV_HEADER_LEN) ||
	(n != output_buf.l + UTIL_TLV_HEADER_LEN)) {
	MP_WARNING("Bad length from RCP pipe: n=%d, output_buf.l=%d, output_buf.t=%d\n",
		   (int)n, (int)output_buf.l, (int)output_buf.t);
	mp_realtime_disconnect();
	return;
    }

    if (output_buf.t == MP_INT_TLV_KEEPALIVE) {
	// This was just a keepalive - drop it and don't send it to the client.
	return;
    }

    // printf("sending to client: ");
    // dump_tlv(&output_buf);

    if (client_fd != -1) {
	struct timeval timeout = { 10, 0 };

	if (send_with_timeout(client_fd, &output_buf, n, &timeout) != n) {
	    /* Error sending to the client.  Close the connection
	       and clean up. */
	    MP_WARNING("Mindprobe client write failed: %s\n",
		       strerror(errno));
	    mp_close_client();
	}
    }
}

void mp_handle_command(struct UTIL_tlv_msg_buf *tlv_buf)
{
    if (RTS_pipe_write_message_timeout(realtime_pipe, tlv_buf, tlv_buf->l + UTIL_TLV_HEADER_LEN, 5000000) != (tlv_buf->l + UTIL_TLV_HEADER_LEN)) {
	MP_WARNING("Mindprobe pipe write failed\n");
	mp_realtime_disconnect();
    }
}

static int mp_realtime_connect(void)
{
    int status = -1;
    int connection_pipe_opened = 0;
    RTS_pipe connection_pipe;
    struct UTIL_tlv_msg_buf tlv_buf;
    struct UTIL_tlv_desc tlv_desc;
    int type, length;
    void *valueptr;
    int32_t result;
    int nwritten;
    int nread;
    char *pipe_name = NULL;

    /* Open the pipe to the RT mind-probe side */
    connection_pipe = RTS_pipe_open(RTS_MINDPROBE_PIPE);
    if (connection_pipe < 0) {
	MP_WARNING("Could not open mindprobe connection pipe: %s\n", strerror(errno));
	status = -1;
	goto done;
    }
    connection_pipe_opened = 1;

    /* Establish the connection. */
    UTIL_tlv_start_write(&tlv_desc, &tlv_buf, sizeof(tlv_buf.v), MP_INT_TLV_CONNECT);
    UTIL_tlv_write(&tlv_desc, MP_INT_TLV_CLIENT_NAME, sizeof(CLIENT_NAME), CLIENT_NAME);
    status = UTIL_tlv_finish_write(&tlv_desc);
    MP_ASSERT(status == 0);

    nwritten = RTS_pipe_write_message(connection_pipe, &tlv_buf, tlv_buf.l + UTIL_TLV_HEADER_LEN);
    if (nwritten != (tlv_buf.l + UTIL_TLV_HEADER_LEN)) {
	if (nwritten < 0) {
	    MP_WARNING("Error writing to connection pipe: %s\n", strerror(errno));
	} else {
	    MP_WARNING("Incomplete write to connection pipe, expected to write %u, but actually wrote %d\n",
		       (unsigned) (tlv_buf.l + UTIL_TLV_HEADER_LEN), nwritten );
	}
	status = -1;
	goto done;
    }

    nread = RTS_pipe_read_message(connection_pipe, &tlv_buf, sizeof(tlv_buf));

    if (nread <= 0) {
	MP_WARNING("Failed to read from connection pipe: %s\n", strerror(errno));
	status = -1;
	goto done;
    }
    if ((nread < UTIL_TLV_HEADER_LEN) ||
	(nread != tlv_buf.l + UTIL_TLV_HEADER_LEN) ) {
	MP_WARNING("Bad length from connection pipe.\n");
	status = -1;
	goto done;
    }

    /* We are now done with the connection pipe - close it. */
    RTS_pipe_close(connection_pipe);
    connection_pipe_opened = 0;

    type = UTIL_tlv_start_read(&tlv_desc, &tlv_buf);
    if (type != MP_INT_TLV_CONNECT_REPLY) {
	MP_FATAL("Received connect reply of type %d, expected %d\n", type, MP_INT_TLV_CONNECT_REPLY);
	status = -1;
	goto done;
    }
    while (UTIL_tlv_read(&tlv_desc, &type, &length, &valueptr) == 0) {
	switch (type) {
	case MP_INT_TLV_PIPE_NAME:
	    pipe_name = valueptr;
	    break;

	case MP_INT_TLV_CONNECT_RESULT:
	    memcpy(&result, valueptr, sizeof(result));
	    if (result != 0) {
		MP_WARNING("Connect failed with code %d\n", result);
		status = -1;
		goto done;
	    }
	    break;
	}
    }
    if (pipe_name == NULL) {
	MP_WARNING("Connect failed - did not receive a pipe name in connect reply\n");
	status = -1;
	goto done;
    }

    realtime_pipe = RTS_pipe_open(pipe_name);
    if (realtime_pipe < 0) {
	MP_WARNING("Could not open mindprobe pipe: %s\n", strerror(errno));
	status = -1;
	goto done;
    }

    realtime_read_fd = RTS_pipe_get_read_fd(realtime_pipe);

    status = 0;
    realtime_pipe_connected = 1;

done:
    if (connection_pipe_opened) {
	/* Close the connection pipe if it was open when we got here. */
	RTS_pipe_close(connection_pipe);
	connection_pipe_opened = 0;
    }
    return status;
}

static void mp_realtime_disconnect(void)
{
    RTS_pipe_close(realtime_pipe);
    realtime_pipe_connected = 0;
}

static void do_server(void)
{
    fd_set	fds;
    int	nfds;
    struct timeval timeout;
    int nready;
    int unanswered_keepalives = 0;
    
    mp_set_ready(1);
    while (mp_is_ready ()) {

	while (!realtime_pipe_connected) {
	    /* TBD: should we disconnect a TCP client if RCP disconnects us? */
	    if (mp_realtime_connect()) {
		MP_WARNING("Failed to connect realtime pipe, will retry.\n");
		sleep(1);
	    }
	}

	timeout.tv_sec = 1;
	timeout.tv_usec = 0;

	FD_ZERO(&fds);
	FD_SET(server_fd, &fds);
	FD_SET(realtime_read_fd, &fds);

	if (client_fd != -1) FD_SET(client_fd, &fds);

	nfds = server_fd;
	if (realtime_read_fd > nfds) nfds = realtime_read_fd;
	if (client_fd > nfds) nfds = client_fd;
	nfds += 1;

	nready = select(nfds, &fds, NULL, NULL, &timeout);

	if (nready < 0) {
	    MP_FATAL("select: %s\n", strerror(errno));
	}

	if (nready == 0) {
	    /* If everything is idle, check that we are still
	     * connected to the real-time side.
	     */
	    if (unanswered_keepalives > MAX_UNANSWERED_KEEPALIVES) {
		mp_realtime_disconnect();
	    } else {
		/* Send a keepalive request to the realtime side. */
		send_rcp_tlv(MP_INT_TLV_KEEPALIVE, 0, NULL);
		unanswered_keepalives++;
	    }
	}

	if (FD_ISSET(server_fd, &fds)) {
	    /* incoming connect */

	    /* If a client was already connected, disconnect it. */
	    mp_close_client();

	    /* Accept the new client connection. */
	    accept_client();

	    /* Reset client input buffer state. */
	    mp_reset_client_input();

	    /* Send the client a message indicating the protocol version: */
	    mp_send_client_protocol_version();
	}
	if ((client_fd != -1) && (FD_ISSET(client_fd, &fds))) {
	    if (mp_receive_from_client()) {
		/* Client closed the connection, or error.  Clean up. */
		mp_close_client();
	    }
	}

	if (FD_ISSET(realtime_read_fd, &fds)) {
	    /* We received data on the realtime pipe, realtime
	     * connection is alive.
	     */
	    unanswered_keepalives = 0;

	    receive_from_pipe();
	}
    }
}

static void *mp_comm_main(void *port_as_void)
{
    RCP_log_thread_init( "mp_comm_main" );

    mp_call_init_hooks();
    
    // Tell roboweb about available probe names
    mp_dump_probe_names( "/tmp/probes.txt" );

    intptr_t port = (intptr_t)port_as_void;
    server_fd = open_server_socket(port, 2);

    client_fd = -1;

    do_server();

    mp_realtime_disconnect();
    mp_close_client();
    mp_close_server_socket();

    pthread_exit(NULL);
}

uint32_t mp_get_current_tick(void)
{
    return RTS_getCurrentTick();
}

void MP_init()
{
    mp_realtime_init();  /* create pipes */
}

void MP_begin(int port)
{
    static pthread_t mp_comm_thread;
    int status;

    status = RTS_pthread_create(&mp_comm_thread, NULL, mp_comm_main, (void *)(intptr_t)port);

    if (status != 0) {
	MP_FATAL("Could not create Mindprobe listener thread\n");
    }

    // TODO - handle this timeout differently.  With the new
    // multi-client MP, the mindprobe listener can't start until it
    // can interact with the realtime RCP; so if we sit here waiting
    // for the MP listener to start it never will because we are
    // holding up the realtime thread...
#if 0
    // Give mp_comm_main a generous time to come up - 
    // we're only interested in not permanently hanging the system
    int64_t timeout_usec = 5*1000*1000;
    int64_t start_time_usec = RTS_get_monotonic_time_usec();
    do {
        sched_yield();
    } while( !mp_is_ready()
             && ( RTS_get_monotonic_time_usec() - start_time_usec) < timeout_usec );
    if( !mp_is_ready() ) {
        MP_WARNING( "mp_comm_main failed to initialize after %f.2 sec; not waiting any longer",
                    (float)( RTS_get_monotonic_time_usec() - start_time_usec ) * 1e-6f ); 
    }
#endif
}

void
mp_fatal_error (const char *file, int line, const char *format, ...)
{
    /* Fatal errors, by definition, cannot be reported to monitoring
       MP clients; so just log them directly to syslog.
    */

    va_list ap;

    mp_set_ready (0);

    va_start (ap, format);
#ifndef BTB
    rt_vsyslog (LOG_ERR, (char *) format, ap);
#elif defined (BTB)
    /* This behaviour mimicks what RTS_fatal() did
       when MP_FATAL() was using it as a back-end: */
    fflush(stdout);
    fprintf(stderr,
            "%lu Fatal error (%s:%d): ",
            (unsigned long)RTS_getCurrentTick(), 
            file, line);
    vfprintf(stderr, format, ap);
    fprintf(stderr, "\n");
#else
    vsyslog (LOG_ERR, format, ap);
#endif
    va_end (ap);
}


/*

Portions of this software are covered by the following copyright:

	Copyright (c) 1995, 1996, 2000 Douglas E. Comer, David L. Stevens, and
Prentice Hall, Inc. All rights reserved.

Redistribution and use in source and binary forms are permitted
provided that this notice is preserved and that due credit is given
to the copyright holders. The names of the copyright holders
may not be used to endorse or promote products derived from this
software without specific prior written permission. This software
is provided ``as is'' without express or implied warranty. The authors
assume no liability for damages incidental or consequential, nor is the
software warranted for correctness or suitability for any purpose.
	Portions of this software are documented in the book:

        Internetworking With TCP/IP Vol 3:
        Client-Server Programming And Applications   
        Linux/POSIX Sockets Version

        Prentice Hall, Upper Saddle River, New Jersey.
        ISBN: 0-13-032071-4

This software may not be sold or published in printed form without written
permission from the copyright holders.

*/
