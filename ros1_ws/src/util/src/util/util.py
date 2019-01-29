import time
import rospy

def usec_since_epoch():
    return int(time.time() * 10**6)

def ms_since_epoch():
    return int(time.time() * 10**3)

def mph_to_ms(mph):
    return 0.44704 * mph

def ms_to_mph(ms):
    return ms / 0.44704
