#!/usr/bin/env python

import mp

class Getter: # parent class for setting values
    def __init__(self, mp): 
        # constructor for settings class
        self.mp = mp
        self.q = []

    def init(self, code): 
        self.code = code
        # initialize client connection
        self.mp.enable_probes(self.code)

    def listen(self): 
        # call mp collect reading
        self.mp.capture(0.05)
        # call mp to get data, put it into the queue
        capture = self.mp.return_capture()
        for elem in capture:
            print(elem)
            self.q.append(elem)

