#!/usr/bin/env python

import mp

class Getter: # parent class for setting values
    def __init__(self, name, codes, mp): 
        # constructor for settings class
        self.mp = mp
        self.codes = codes
        self.name = name

    ### TODO: because this is single connection based, probes need to be enabled together, otherwise only the last enabled probe will do be captured in data capture
    # def init(self): 
        # # initialize client connection
        # self.mp.enable_probes(self.codes)

    def run(self, data):
        ## Publish data
        self.pub.publish(data)
