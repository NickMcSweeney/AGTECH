#!/usr/bin/env python
#import rospy
from hv_bridge.srv import mp_toggle, mp_toggleResponse
from hv_bridge.srv import mp_set_target, mp_set_targetResponse

#probe id 3486: behaviors.spacing.pick.pickTarget.x type float length 4
#probe id 3487: behaviors.spacing.pick.pickTarget.y type float length 4
#probe id 3304: behaviors.spacing.runCollection type bool length 1
#probe id 2438: loc.actual.x type float length 4
#probe id 2439: loc.actual.y type float length 4
RUN_SPACING_CODE = 3304
SET_X_CODE = 3486
SET_Y_CODE = 3487
SET_X_LOC_CODE = 2438
SET_Y_LOC_CODE = 2439
SET_H_LOC_CODE = 2440
PSEUDO_START_CODE = 1125
BOT_GOT_POT_CODE = 1100


class PickTargetService():
    def __init__(self, mp, ros):
        # contstruct basics for 
        self.mp = mp
        self.serv_set = ros.Service('SetPickTarget', mp_set_target, self.set_pick_target)
        self.serv_run = ros.Service('RunPick', mp_toggle, self.toggle_pick)

    def stop(self):
        try:
            self.serv_set.shutdown('end of life') 
            self.serv_run.shutdown('end of life') 
            print("pick target service ended")
            return True
        except:
            print("pick target service shutdown failed")
            return False

    def set_pick_target(self,req):
        # set values for target
        self.mp.write_probe(BOT_GOT_POT_CODE, 0)
        self.mp.write_probes([(SET_X_CODE, req.x),(SET_Y_CODE, req.y)])

        return "suceess: set target to (" + str(req.x)+","+str(req.y)+")"
        
    def toggle_pick(self, req):
        # toggle follow person on/off
        if req.val == 1:
            #self.mp.write_probe(PSEUDO_START_CODE,0)
            self.mp.write_probe(RUN_SPACING_CODE,1)
        else:
            self.mp.write_probe(RUN_SPACING_CODE,0)
            #self.mp.write_probe(PSEUDO_START_CODE,1)
            
        return "success: value set to " + ("true" if req.val == 1 else "false")
