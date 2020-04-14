import math

### various utility functions shared by multiple modules ###

def twist_to_vel(twist):
    # assumes x is forward/backward and y is lateral.
    r = 0.25
    L = 1.0
    # x = twist.linear.x
    # y = twist.linear.y
    # v = math.sqrt(math.exp(x)+math.exp(y))
    # if(x == 0):
        # th = y
    # else:
        # th = math.atan(y/x)
    v = twist.linear.x
    th = twist.angular.z
    left = (2*v - L*th)/2*r 
    right = (2*v + L*th)/2*r
    # if (left > right):
        # right = 0
    # elif (right > left):
        # left = 0
    return [left,right]

def resource_name(name):
    return ((name.replace("-", "")).replace(".","_")).lower()
