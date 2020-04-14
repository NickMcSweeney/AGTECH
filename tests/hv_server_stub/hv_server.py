import SocketServer
import struct

MP_TLV_NULL                  = 0
MP_TLV_CONNECTED             = 1
MP_TLV_DISCONNECTED          = 2
MP_TLV_PROTOCOL_VERSION      = 3
MP_TLV_DISCOVER_PROBES       = 4
MP_TLV_HZ                    = 5 
MP_TLV_PROBE_DEF             = 6 
MP_TLV_ENABLE_PROBES         = 7 
MP_TLV_START_PROBES          = 8 
MP_TLV_STOP_PROBES           = 9 
MP_TLV_PROBE_DATA            = 10
MP_TLV_CURRENT_TICK          = 11
MP_TLV_MISSED_DATA           = 12
MP_TLV_DEBUG_MESSAGE         = 13
MP_TLV_MESSAGE_TEXT          = 14
MP_TLV_MISSED_DEBUG_MESSAGES = 15
MP_TLV_WRITE_PROBES          = 16

# Type names for probe type enumerations, and pack/unpack directives:

type_table = {
    1:  ('uint8',   '<B'), # 1-byte unsigned integer
    2:  ('int8',    '<b'), # 1-byte signed integer
    3:  ('uint16',  '<H'), # 2-byte unsigned integer
    4:  ('int16',   '<h'), # 2-byte signed integer
    5:  ('uint32',  '<I'), # 4-byte unsigned integer
    6:  ('int32',   '<i'), # 4-byte signed integer
    7:  ('uint64',  '<Q'), # 8-byte unsigned integer
    8:  ('int64',   '<q'), # 8-byte signed integer
    9:  ('float',   '<f'), # 4-byte float (IEEE 754)
    10: ('double', '<d'),  # 8-byte float (IEEE 754)
    11: ('tlv',    ''),    # Variable-length TLV data
    12: ('bool',   '<B'),  # 1-byte boolean integer (1 or 0)
    13: ('string', ''),    # variable-size null-terminated char string
}

def make_tlv_uint8(t, v):
    return struct.pack("<HHH", t, 1, v)

def make_tlv_uint16(t, v):
    return struct.pack("<HHH", t, 2, v)

def make_tlv_uint32(t, v):
    return struct.pack("<HHI", t, 4, v)

def make_tlv(t, v = ''):
    return struct.pack("<HH", t, len(v)) + v

def tlv_len(message):
    (t, l) = struct.unpack("<HH", message[0:4])
    return l

def next_tlv(message):
    (t, l) = struct.unpack("<HH", message[0:4])
    v = message[4:4+l]
    return (t, l, v)

class ServerStub(SocketServer.BaseRequestHandler):
    def setup(self):
        print("serving... sending initial msg")
        self.request.sendall(make_tlv_uint16(MP_TLV_PROTOCOL_VERSION,1))
    def handle(self):
        (t, l, v) = next_tlv(self.request.recv(1024))
        print "{} wrote:".format(self.client_address[0])
        if(t == MP_TLV_HZ):
            v = 200
        if(t == MP_TLV_DISCOVER_PROBES):
            v = ''
        print (t, v)
        return self.request.sendall(make_tlv_uint16(t,v))

if __name__ == "__main__":
    print("starting server stub")
    #HOST, PORT = "hai-1095.local", 4400
    HOST, PORT = "localhost", 4400
    print("serving at: {}").format(HOST)

    server = SocketServer.TCPServer((HOST,PORT), ServerStub)
    print("server created")
    server.serve_forever()

    print("ending server")
    server.shutdown()
    server.close()

