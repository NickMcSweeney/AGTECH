#!/usr/bin/env python

import mp
import asyncio

class Getter: # parent class for setting values
    def __init__(self, name, codes, mp): 
        # constructor for settings class
        self.mp = mp
        self.codes = codes
        self.name = name

    def init(self): 
        # initialize client connection
        self.mp.enable_probes(self.code)

    def run(self, data):
        ## Publish data
        self.pub(data)
