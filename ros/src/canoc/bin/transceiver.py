#!/usr/bin/python

import rospy
import can
import sys
from canoc.can_identifier import CanIdentifier

RATE = 50.0

def main():
    rospy.init_node('transceiver', anonymous=False, log_level=rospy.DEBUG)
    interfaces = rospy.get_param('~can-interfaces').split(',')
    CanIdentifier(param_name=CanIdentifier.CONTROL_INTERFACE_PARAM, can_interfaces=interfaces, matching_ids=CanIdentifier.CONTROL_IDS_MATCH)

if __name__ == '__main__':
    main()
