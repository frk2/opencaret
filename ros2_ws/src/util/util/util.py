import time
from builtin_interfaces.msg import Time

def time_stamp(t=None):
    t=time.time() if t is None else t
    return Time(sec=int(t), nanosec=int((t-int(t))*1000000000))

def usec_since_epoch():
    return int(time.time() * 10**6)

def ms_since_epoch():
    return int(time.time() * 10**3)

def mph_to_ms(mph):
    return 0.44704 * mph

def ms_to_mph(ms):
    return ms / 0.44704
