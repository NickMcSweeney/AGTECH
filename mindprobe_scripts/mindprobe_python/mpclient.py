#
# mp-client.py -- dummy mindprobe client for testing
#

import socket
import select
import struct
import code
import threading
import Queue
import time
import os
import sys
import subprocess

MP_TLV_NULL = 0
MP_TLV_CONNECTED = 1
MP_TLV_DISCONNECTED = 2
MP_TLV_PROTOCOL_VERSION = 3
MP_TLV_DISCOVER_PROBES = 4
MP_TLV_HZ = 5
MP_TLV_PROBE_DEF = 6
MP_TLV_ENABLE_PROBES = 7
MP_TLV_START_PROBES = 8
MP_TLV_STOP_PROBES = 9
MP_TLV_PROBE_DATA = 10
MP_TLV_CURRENT_TICK = 11
MP_TLV_MISSED_DATA = 12
MP_TLV_DEBUG_MESSAGE = 13
MP_TLV_MESSAGE_TEXT = 14
MP_TLV_MISSED_DEBUG_MESSAGES = 15
MP_TLV_WRITE_PROBES = 16

# Type names for probe type enumerations, and pack/unpack directives:

type_table = {
    1: ("uint8", "<B"),  # 1-byte unsigned integer
    2: ("int8", "<b"),  # 1-byte signed integer
    3: ("uint16", "<H"),  # 2-byte unsigned integer
    4: ("int16", "<h"),  # 2-byte signed integer
    5: ("uint32", "<I"),  # 4-byte unsigned integer
    6: ("int32", "<i"),  # 4-byte signed integer
    7: ("uint64", "<Q"),  # 8-byte unsigned integer
    8: ("int64", "<q"),  # 8-byte signed integer
    9: ("float", "<f"),  # 4-byte float (IEEE 754)
    10: ("double", "<d"),  # 8-byte float (IEEE 754)
    11: ("tlv", ""),  # Variable-length TLV data
    12: ("bool", "<B"),  # 1-byte boolean integer (1 or 0)
    13: ("string", ""),  # variable-size null-terminated char string
}

capture_buffer = ()


def connect(host, port=4400):
    global s, recvbuf, probe_defs, hz, version
    global capture_buffer_lock

    capture_buffer_lock = threading.Semaphore()

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host, port))
    recvbuf = ""
    print "Connected"

    start_listener()

    # Expect the server to send us the protocol version, unsolicited:
    (t, l, v) = get_next_message()
    if (t != MP_TLV_PROTOCOL_VERSION) or (l != 2):
        raise RuntimeError("unexpected message t=%d l=%d" % (t, l))
    version = struct.unpack("<H", v)[0]

    # Query the tick rate:
    send_message(make_tlv(MP_TLV_HZ))
    (t, l, v) = get_next_message()
    if (t != MP_TLV_HZ) or (l != 2):
        raise RuntimeError("unexpected message t=%d l=%d" % (t, l))
    hz = struct.unpack("<H", v)[0]

    # Discover the probes:
    discover_probes()

    print "Protocol version = 0x%04x, sample rate is %d Hz, %d probes" % (
        version,
        hz,
        len(probe_defs.keys()),
    )


def disconnect():
    global s
    stop_listener()
    s.close()


def discover_probes():
    global probe_defs, probe_names, version

    new_probe_defs = {}  # dictionary of id: (name, type)
    new_probe_names = {}  # dictionary of name: id

    send_message(make_tlv(MP_TLV_DISCOVER_PROBES))
    while True:
        (t, l, def_items) = get_next_message()
        if t != MP_TLV_DISCOVER_PROBES:
            raise RuntimeError("unexpected message t=%d l=%d" % (t, l))

        while len(def_items):
            (t, l, v, def_items) = next_tlv(def_items)
            if t != MP_TLV_PROBE_DEF:
                raise RuntimeError("unexpected message t=%d l=%d" % (t, l))

            (probe_id, probe_type, probe_length) = struct.unpack("<HHH", v[0:6])

            # Name has null terminator, which Python doesn't need.
            probe_name = v[6 : len(v) - 1]

            new_probe_names[probe_name] = probe_id
            new_probe_defs[probe_id] = (probe_name, probe_type, probe_length)
            pass

        if len(def_items) == 0 or version < 0x0104:
            probe_defs = new_probe_defs
            probe_names = new_probe_names
            break

        pass

    return


def show_probes():
    global probe_defs

    for id in probe_defs.keys():
        (name, type, length) = probe_defs[id]
        type_name = type_table[type][0]
        print "probe id %d: %s type %s length %d" % (id, name, type_name, length)


def make_tlv_uint8(t, v):
    return struct.pack("<HHH", t, 1, v)


def make_tlv_uint16(t, v):
    return struct.pack("<HHH", t, 2, v)


def make_tlv_uint32(t, v):
    return struct.pack("<HHI", t, 4, v)


def make_tlv(t, v=""):
    return struct.pack("<HH", t, len(v)) + v


def tlv_len(message):
    (t, l) = struct.unpack("<HH", message[0:4])
    return l


def next_tlv(message):
    (t, l) = struct.unpack("<HH", message[0:4])
    v = message[4 : 4 + l]
    return (t, l, v, message[4 + l :])


def send_message(message):
    global s
    # print "sending: ",
    # print struct.unpack("%dB" % len(message), message)
    s.send(message)


def lookup_probe_id(probe):
    global probe_names

    if type(probe) == str:
        if not probe in probe_names:
            raise ValueError("unknown probe %s" % probe)
        probe_id = probe_names[probe]
    elif type(probe) == int:
        probe_id = probe
    else:
        raise ValueError("probe must be int or string")

    return probe_id


def enable_probes(probes):
    # allow for singleton argument
    if type(probes) != tuple:
        probes = ((probes),)
    v = ""
    for probe in probes:
        probe_id = lookup_probe_id(probe)
        v += struct.pack("<H", probe_id)
    send_message(make_tlv(MP_TLV_ENABLE_PROBES, v))


def start_probes():
    global capturing
    capturing = True
    send_message(make_tlv(MP_TLV_START_PROBES))


def stop_probes():
    global capturing
    send_message(make_tlv(MP_TLV_STOP_PROBES))
    capturing = False


def do_debug_message(message):
    (t, l, v, message) = next_tlv(message)
    if t != MP_TLV_CURRENT_TICK:
        raise RuntimeError("unexpected type = %d" % t)
    tick = struct.unpack("<I", v)[0]
    (t, l, v, message) = next_tlv(message)
    if t != MP_TLV_MESSAGE_TEXT:
        raise RuntimeError("unexpected type = %d" % t)
    message_text = v[0 : len(v) - 1]
    if message_text[-1] == "\n":
        message_text = message_text[0 : len(message_text) - 1]
    print ("Debug message (t = %d): %s" % (tick, message_text))


def do_async_message(t, l, v):
    if t == MP_TLV_PROBE_DATA:
        capture_data(v)
        return True
    elif t == MP_TLV_DEBUG_MESSAGE:
        do_debug_message(v)
        return True
    elif t == MP_TLV_MISSED_DATA:
        print ("Missed %d probe messages" % struct.unpack("<I", v))
        return True
    elif t == MP_TLV_MISSED_DEBUG_MESSAGES:
        print ("Missed %d debug messages" % struct.unpack("<I", v))
        return True
    else:
        return False  # not an asynchronous message


# Receive one message from the socket:
def receive_next_message():
    global recvbuf, s

    while (len(recvbuf) < 4) or (len(recvbuf) < tlv_len(recvbuf)):
        recvbuf += s.recv(65536)
        # print "recvbuf is: ",
        # print struct.unpack("%dB" % len(recvbuf), recvbuf)

    (t, l, v, recvbuf) = next_tlv(recvbuf)

    return (t, l, v)


def get_next_message():
    global listener_q
    (t, l, v) = listener_q.get()
    return (t, l, v)


def capture_buffer_len():
    global capture_buffer
    capture_buffer_lock.acquire()
    l = len(capture_buffer)
    capture_buffer_lock.release()
    return l


def capture(t):
    global capture_buffer, hz

    n = int(round(t * hz))

    capture_buffer_lock.acquire()
    capture_buffer = ()
    capture_buffer_lock.release()

    start_probes()
    time.sleep(t)  # let the listener thread do the capturing
    stop_probes()


def capture_data(data):
    global capturing
    global capture_buffer

    # Add the data an item in the caputre buffer.
    if capturing:
        capture_buffer_lock.acquire()
        capture_buffer += (data,)
        capture_buffer_lock.release()


def show_capture(interval=None):
    global capture_buffer, hz
    global probe_defs

    if interval == None:
        interval = 1.0 / hz

    step = int(round(hz * interval))

    capture_buffer_lock.acquire()
    for i in range(0, len(capture_buffer), step):
        item = capture_buffer[i]

        if len(item) == 0:
            print "empty capture buffer item"
            continue

        (t, l, v, item) = next_tlv(item)
        if t != MP_TLV_CURRENT_TICK:
            print "bad capture item, first element not tick"
            continue

        (val,) = struct.unpack("<I", v)
        print "tick =", val,

        while len(item):
            (t, l, v, item) = next_tlv(item)

            name = probe_defs[t][0]
            type = probe_defs[t][1]
            val = decode_probe_data(type, v)

            print name, "=", val,
        print ""
    capture_buffer_lock.release()


def write_probe(probe, value):
    global probe_defs

    probe_id = lookup_probe_id(probe)
    probe_type = probe_defs[probe_id][1]

    v = make_tlv(probe_id, encode_probe_value(probe_type, value))
    send_message(make_tlv(MP_TLV_WRITE_PROBES, v))


def write_probes(probevals):
    global probe_defs

    v = ""

    for (probe, value) in probevals:
        probe_id = lookup_probe_id(probe)
        probe_type = probe_defs[probe_id][1]

        v += make_tlv(probe_id, encode_probe_value(probe_type, value))
    send_message(make_tlv(MP_TLV_WRITE_PROBES, v))


def encode_probe_value(probe_type, value):
    global type_table
    if type_table[probe_type][0] == "tlv":
        return value  # assume it's already encoded.
    elif type_table[probe_type][0] == "string":
        return value + "\0"  # add a null terminator
    else:
        return struct.pack(type_table[probe_type][1], value)


def decode_probe_data(probe_type, data):
    global type_table
    if type_table[probe_type][0] == "tlv":
        return tlv_to_text(data)
    elif type_table[probe_type][0] == "string":
        #        return len(data)
        return data[:-1]  # lose the null terminator
    else:
        return struct.unpack(type_table[probe_type][1], data)[0]


def tlv_to_text(data):
    (t, l) = struct.unpack("<HH", data[0:4])
    v = data[4 : 4 + l]
    # If we knew which t values were recursive encodings, we could
    # be clever here.
    string = "t=%d:l=%d:v=" % (t, l)
    for i in range(len(v)):
        string += "%02x" % ord(v[i])
        if i < len(v):
            string += ":"
    return string


def send_script(scriptname):
    # Assume that script-compile is in the same directory as this script.
    compiler = sys.path[0] + "/script-compile"

    compiled_script = subprocess.Popen(
        [compiler, "--stdout", scriptname], stdout=subprocess.PIPE
    ).communicate()[0]
    write_probe("rcp_script", compiled_script)


class listen(threading.Thread):
    def __init__(self, q):
        threading.Thread.__init__(self)
        self.q = q
        self.setDaemon(True)

    def run(self):
        global s, listener_pipe
        while True:
            # Wait for incoming data from the RCP or for the listener
            # pipe to close, indicating that we are
            (r, w, x) = select.select((s, listener_pipe[0]), (), ())
            if listener_pipe[0] in r:
                # listener pipe signaled the thread to exit
                os.close(listener_pipe[0])
                return
            if s in r:
                (t, l, v) = receive_next_message()
                if not do_async_message(t, l, v):
                    self.q.put((t, l, v))
                # time.sleep(0.1)


def start_listener():
    global listener, listener_q, listener_pipe

    listener_pipe = os.pipe()

    listener_q = Queue.Queue()

    listener = listen(listener_q)
    listener.start()


def stop_listener():
    global listener, listener_pipe
    os.write(listener_pipe[1], "bye")
    os.close(listener_pipe[1])
    listener.join()
