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


class SimpleTimedDiff():
    def __init__(self, maxtime=0.1):
        self.items = []
        self.maxtime = maxtime

    def append(self, item, time):
        self.items.append((item, time))
        timediff = self.items[-1][1] - self.items[0][1]
        if timediff > self.maxtime:
            self.items.pop(0)

    def get_diff(self):
        timediff = self.items[-1][1] - self.items[0][1]
        if timediff > 0.:
            return (self.items[-1][0] - self.items[0][0]) / timediff
        else:
            return None