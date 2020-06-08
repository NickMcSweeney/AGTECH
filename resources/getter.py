#!/usr/bin/env python

import mp

class Getter: # parent class for setting values
    def __init__(self, name, codes, mp): 
        # constructor for settings class
        self.mp = mp
        self.codes = codes
        self.name = name

    def init(self): 
        # initialize client connection
        self.mp.include_probes(self.codes)

    def run(self, data):
        ## Publish data
        self.pub.publish(data)
