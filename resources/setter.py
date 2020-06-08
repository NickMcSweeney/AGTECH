#!/usr/bin/env python

import mp

class Setter: # parent class for setting values
    def __init__(self, mp): 
        # constructor for settings class
        self.mp = mp

    def init(self, code): 
        self.code = code
        # initialize client connection
        self.mp.include_probes(self.code)

    def callback(self, msg): 
        # subscriber callback function
        if(type(self.code) != tuple):
            # if the message is a single item value pair
            self.mp.write_probe(self.code, msg)
        elif(len(msg) > 1):
            # handle an array of messages
            # each message coresponds to a probe (1 to 1)
            #if(msg.length != self.code.length) throw "message size missmatch"
            command = []
            i = 0
            for c in self.code:
                command.append((c,msg[i]))
                i = i + 1
            self.mp.write_probes(command)
        else:
            # publish the same message to each of the probes
            command = []
            for c in self.code:
                command.append((c,msg))
            self.mp.write_probes(command)
