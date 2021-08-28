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

class SrclocModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(SrclocModule, self).__init__(mpstate, "srcloc", "srcloc module")
        '''initialisation code'''
        self.hlat = -353632621   #home location, in (approx) centimetres
        self.hlon = 1491652374
        self.slat = -353632261   #source location
        self.slon = 1491652321
        self.elat = 0 #estimated source location
        self.elon = 0 
        # self.console.set_status('SRLoc', 'SRLoc %.7f %.7f' % (self.slat*1e-7, self.slon*1e-7), row=5)
        # self.console.set_status('SELoc', 'SELoc --- ---', row=5)
        self.console.set_status('PlSt', 'Plume Strength ---', row=5)

    #[ab;bc] "is essentially 0.5 over the covariance matrix", A is the amplitude, and (x0, y0) is the center
    def gauss2d(self, x, y, amp, x0, y0, a, b, c):
        x = (x - self.hlat)*1e-3
        y = (y - self.hlon)*1e-3
        x0 = (x0 - self.hlat)*1e-3
        y0 = (y0 - self.hlon)*1e-3
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

    def mavlink_packet(self, m):
        'handle a MAVLink packet'''
        if m.get_type() == 'GPS_GLOBAL_ORIGIN':
            # message is sent when origin is initially set but not after that.
            # this means that currently the module must be started before the EKF sets its origin if the vehicle is started anywhere other than CMAC.
            self.hlat = m.latitude
            self.hlon = m.longitude
            print("Origin lat lon set to: %.0f %.0f" % (self.hlat, self.hlon))
        if m.get_type() == 'GLOBAL_POSITION_INT':
            #0.0000001 deg =~ 1 cm, i.e. m.lat & m.lon are approx in cm, thus divide by 1000 to get m
            stre = self.gauss2d(m.lat, m.lon, 1, self.slat, self.slon, 0.25, 0, 0.5)
            self.master.mav.plume_strength_send(stre)
            self.showIcon(4, self.slat, self.slon, 'redstar.png')
            self.console.set_status('PlSt', 'Plume Strength %.7f' % (stre), row=5)
        if m.get_type() == 'PLUME_EST_LOC':
            self.elat = self.hlat + m.x*111
            self.elon = self.hlon + m.y*111
            self.showIcon(5, self.elat, self.elon, 'bluestar.png')

#latitude means north
#Latitude: -35.282001 
def init(mpstate):
    '''initialise module'''
    return SrclocModule(mpstate)