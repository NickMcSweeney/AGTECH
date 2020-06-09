#!/usr/bin/env python
#import rospy
from hv_bridge.srv import mp_toggle, mp_toggleResponse

# probe id 3842: behaviors.followMe.run type bool length 1

FOLLOW_ME_CODE = 3842
PSEUDO_START_CODE = 1125

class FollowService():
    def __init__(self, mp, ros):
        # contstruct basics for 
        self.mp = mp
        self.serv = ros.Service('FollowMe', mp_toggle, self.toggle_follow)

    def stop(self):
        try:
            self.serv.shutdown('end of life') 
            print("follow me service ended")
            return True
        except:
            print("follow me service shutdown failed")
            return False

    def toggle_follow(self, req):
        # toggle follow person on/off
        if req.val == 1:
            #self.mp.write_probe(PSEUDO_START_CODE,0)
            self.mp.write_probe(FOLLOW_ME_CODE,1)
        else:
            self.mp.write_probe(FOLLOW_ME_CODE,0)
            #self.mp.write_probe(PSEUDO_START_CODE,1)
            
        return "success: value set to " + ("true" if req.val == 1 else "false")
