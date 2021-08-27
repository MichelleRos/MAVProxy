 #!/usr/bin/env python
'''Srcloc'''

import time, os

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import mp_menu
from MAVProxy.modules.mavproxy_map import mp_slipmap
from pymavlink import mavutil
import sys, traceback
import numpy as np

hlat = -353632621   #home location, in (approx) centimetres
hlon = 1491652374
slat = -353632261   #source location
slon = 1491652321
    #[ab;bc] "is essentially 0.5 over the covariance matrix", A is the amplitude, and (x0, y0) is the center
def gauss2d(x, y, amp, x0, y0, a, b, c, hlat, hlon):
    x = (x - hlat)*1e-3
    y = (y - hlon)*1e-3
    x0 = (x0 - hlat)*1e-3
    y0 = (y0 - hlon)*1e-3
    inner = a * (x - x0)**2 
    inner += 2 * b * (x - x0) * (y - y0)    #b is diagonal variance
    inner += c * (y - y0)**2
    return amp * np.exp(-inner)

def showIcon(self, id, lat, lon, img):
     for mp in self.module_matching('map*'):
                icon = mp.map.icon(img)
                #print("Icon at: %.1f %.1f" % (m.lat, m.lon))
                mp.map.add_object(mp_slipmap.SlipIcon(id, (lat * 1e-7, lon * 1e-7),
                                icon, layer=3, rotation=0, follow=False, 
                                trail=mp_slipmap.SlipTrail(colour=(0, 255, 255))))

class SrclocModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(SrclocModule, self).__init__(mpstate, "srcloc", "srcloc module")
        '''initialisation code'''

    def mavlink_packet(self, m):
        'handle a MAVLink packet'''
        if m.get_type() == 'GLOBAL_POSITION_INT':
            #0.0000001 deg =~ 1 cm, i.e. m.lat & m.lon are approx in cm, thus divide by 1000 to get m
            stre = gauss2d(m.lon, m.lat, 1, slon, slat, 0.25, 0, 0.5, hlat, hlon)
            self.master.mav.plume_send(stre)
            showIcon(self, 4, slat, slon, 'redstar.png')

def init(mpstate):
    '''initialise module'''
    return SrclocModule(mpstate)