import rospy
import code
import sys
from mp import *
import resources

# main fn
if __name__ == '__main__':
    try:
        if len(sys.argv) < 2:
            host_name = "hai-1095.local"
        else:
            host_name = sys.argv[1]
        print("connected to {}").format(host_name)

        resources.mp.init(host_name)
        while True:
            code.interact("", local=globals())
        resources.mp.close()
    except rospy.ROSInterruptException:
        pass
