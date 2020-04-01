### various utility functions shared by multiple modules ###

def twist_to_vel(twist):
    return 0.5

def resource_name(name):
    return ((name.replace("-", "")).replace(".","_")).lower()
