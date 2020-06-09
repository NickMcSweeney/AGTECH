#!/usr/bin/env python
#import rospy
from hv_bridge.srv import mp_toggle, mp_toggleResponse

# probe id 3601: behaviors.scriptAvoidCollision.isEnabled type bool length 1
# probe id 3602: behaviors.script.run type bool length 1

AVOID_CODE = 3601
PSEUDO_START_CODE = 1125

class AvoidService():
    def __init__(self, mp, ros):
        # contstruct basics for 
        self.mp = mp
        self.serv = ros.Service('AvoidCollision', mp_toggle, self.toggle_avoid)

    def stop(self):
        try:
            self.serv.shutdown('end of life') 
            print("avoid collision service ended")
            return True
        except:
            print("avoid collision service shutdown failed")
            return False

    def toggle_avoid(self, req):
        # toggle follow person on/off
        if req.val == 1:
            #self.mp.write_probe(PSEUDO_START_CODE,0)
            self.mp.write_probe(AVOID_CODE,1)
        else:
            self.mp.write_probe(AVOID_CODE,0)
            #self.mp.write_probe(PSEUDO_START_CODE,1)
            
        return "success: value set to " + ("true" if req.val == 1 else "false")
