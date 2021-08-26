 #!/usr/bin/env python
'''Srcloc'''

import time, os

from MAVProxy.modules.lib import mp_module
from pymavlink import mavutil
import sys, traceback

class SrclocModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(SrclocModule, self).__init__(mpstate, "srcloc", "srcloc module")
        '''initialisation code'''

    def mavlink_packet(self, m):
        'handle a MAVLink packet'''
        if m.get_type() == 'GLOBAL_POSITION_INT':
            print("My Int: %.7f %.7f" % (m.lat*1.0e-7, m.lon*1.0e-7))

def init(mpstate):
    '''initialise module'''
    return SrclocModule(mpstate)