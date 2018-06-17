import time


def usec_since_epoch():
    return int(time.time() * 10**6)


def ms_since_epoch():
    return int(time.time() * 10**3)