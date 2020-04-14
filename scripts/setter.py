import rospy
from mpclient import *
import code
import sys

# global variables

class Setter: # parent class for setting values
    # def __init__(self, host, port): 
        # # constructor for settings class
        # self.host_name = host
        # self.port = port

    def init(self, code): 
        self.code = code
        # initialize client connection
        # connect(self.host_name, self.port)
        # code.interact("", local=globals())
        enable_probes(self.code)
        start_probes()

    def callback(self, msg): 
        # subscriber callback function
        if(type(self.code) != tuple):
            write_probe(self.code, msg)
        elif(len(msg) > 1):
            #if(msg.length != self.code.length) throw "message size missmatch"
            command = []
            i = 0
            for c in self.code:
                command.append((c,msg[i]))
                i = i + 1
            write_probes(command)
        else:
            command = []
            for c in self.code:
                command.append((c,msg))
            write_probes(command)
