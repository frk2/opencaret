import time

def usec_since_epoch():
    return int(time.time() * 10**6)


def ms_since_epoch():
    return int(time.time() * 10**3)

def mph_to_ms(mph):
    return 0.44704 * mph