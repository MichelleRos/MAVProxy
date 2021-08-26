 #!/usr/bin/env python
'''Srcloc'''

import time, os

from MAVProxy.modules.lib import mp_module
from pymavlink import mavutil
import sys, traceback
import numpy as np

hlon = -353632.621   #home location, in (approx) metres
hlat = 1491652.374
slon = -353632.261   #source location
slat = 1491652.321
    #[ab;bc] "is essentially 0.5 over the covariance matrix", A is the amplitude, and (x0, y0) is the center
def gauss2d(x, y, amp, x0, y0, a, b, c, hlat, hlon):
    x = x - hlat
    y = y - hlon
    x0 = x0 - hlat
    y0 = y0 - hlon
    inner = a * (x - x0)**2 
    inner += 2 * b * (x - x0) * (y - y0)    #b is diagonal variance
    inner += c * (y - y0)**2
    return amp * np.exp(-inner)

class SrclocModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(SrclocModule, self).__init__(mpstate, "srcloc", "srcloc module")
        '''initialisation code'''

    def mavlink_packet(self, m):
        'handle a MAVLink packet'''
        if m.get_type() == 'GLOBAL_POSITION_INT':
            #0.0000001 deg =~ 1 cm, i.e. m.lat & m.lon are approx in cm, thus divide by 1000 to get m
            #print("My Int: %.7f %.7f" % (m.lat*1.0e-7, m.lon*1.0e-7))
            stre = gauss2d(m.lat*1.0e-3, m.lon*1.0e-3, 1, slon, slat, 0.25, 0, 0.5, hlat, hlon)
            self.master.mav.plume_send(stre)

def init(mpstate):
    '''initialise module'''
    return SrclocModule(mpstate)