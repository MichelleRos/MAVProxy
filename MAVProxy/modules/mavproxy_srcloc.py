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

'''
<message id="49" name="GPS_GLOBAL_ORIGIN">
      <description>Publishes the GPS co-ordinates of the vehicle local origin (0,0,0) position. Emitted whenever a new GPS-Local position mapping is requested or set - e.g. following SET_GPS_GLOBAL_ORIGIN message.</description>
      <field type="int32_t" name="latitude" units="degE7">Latitude (WGS84)</field>
      <field type="int32_t" name="longitude" units="degE7">Longitude (WGS84)</field>
      <field type="int32_t" name="altitude" units="mm">Altitude (MSL). Positive for up.</field>
      <extensions/>
      '''

hlat = -353632621   #home location, in (approx) centimetres
hlon = 1491652374
slat = -353632261   #source location
slon = 1491652321
elat = 0 #estimated source location
elon = 0 
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
            stre = gauss2d(m.lat, m.lon, 1, slat, slon, 0.25, 0, 0.5, hlat, hlon)
            self.master.mav.plume_strength_send(stre)
            showIcon(self, 4, slat, slon, 'redstar.png')
        if m.get_type() == 'PLUME_EST_LOC':
            elat = hlat + m.x*111
            elon = hlon + m.y*111
            #print("Icon at: %.0f %.1f" % (elat, elon))
            showIcon(self, 5, elat, elon, 'bluestar.png')
#latitude means north
#Latitude: -35.282001 
def init(mpstate):
    '''initialise module'''
    return SrclocModule(mpstate)