 #!/usr/bin/env python
'''Custom'''

import time, os

from MAVProxy.modules.lib import mp_module
from pymavlink import mavutil
import sys, traceback

class CustomModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(CustomModule, self).__init__(mpstate, "Custom", "Custom module")
        '''initialisation code'''

    def mavlink_packet(self, m):
        'handle a MAVLink packet'''
        if m.get_type() == 'MY_CUSTOM_PACKET':
            print "My Int: %(x).2f" % \
                {"x" : m.intField}

def init(mpstate):
    '''initialise module'''
    return CustomModule(mpstate)