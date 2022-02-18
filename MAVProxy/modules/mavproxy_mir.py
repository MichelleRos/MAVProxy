#!/usr/bin/env python
'''Mir'''

import time, os

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import mp_menu
from MAVProxy.modules.mavproxy_map import mp_slipmap
from pymavlink import mavutil
import sys, traceback
import numpy as np
import scipy.optimize as opt
import math

class MirModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(MirModule, self).__init__(mpstate, "mir", "mir module")
        '''initialisation code'''
        self.hlat = -353632621   #home location, in (approx) centimetres
        self.hlon = 1491652374
        self.TarX = 0
        self.TarY = 0

    def showIcon(self, id, lat, lon, img):
        for mp in self.module_matching('map*'):
            icon = mp.map.icon(img)
            #print("Icon at: %.1f %.1f" % (m.lat, m.lon))
            mp.map.add_object(mp_slipmap.SlipIcon(id, (lat * 1e-7, lon * 1e-7),
                            icon, layer=3, rotation=0, follow=False,
                            trail=mp_slipmap.SlipTrail(colour=(0, 255, 255))))

    def toll(self, x, y): #to lat, long
        dlat = float(x)*89.83204953368922
        scl = max(math.cos((self.hlat+dlat/2)* (1.0e-7 * (3.141592653589793/180.0))), 0.01)
        dlon = (float(y) * 89.83204953368922) / scl
        lat = self.hlat + dlat
        lon = self.hlon + dlon
        return lat, lon

    def mavlink_packet(self, m):
        'handle a MAVLink packet'''
        if m.get_type() == 'GPS_GLOBAL_ORIGIN':
            # message is sent when origin is initially set and when arming, apparently. Unfortunately no apparent way of triggering it.
            # this means that currently the module must be started before the EKF sets its origin if the vehicle is started anywhere other than CMAC.
            self.hlat = m.latitude
            self.hlon = m.longitude
            print("Origin lat lon set to: %.0f %.0f" % (self.hlat, self.hlon))
        if m.get_type() == 'NAMED_VALUE_FLOAT':
            if m.name == 'BSCxz':
                self.console.set_status('BSCxz', 'BSCxz %.3f' % m.value, row=7)
            elif m.name == 'BSCyyaw':
                self.console.set_status('BSCyyaw', 'BSCyyaw %.3f' % m.value, row=7)
            if m.name == 'BSCxz_n':
                self.console.set_status('BSCxz_n', 'BSCxz_n %.3f' % m.value, row=7)
            elif m.name == 'BSCyyaw_n':
                self.console.set_status('BSCyyaw_n', 'BSCyyaw_n %.3f' % m.value, row=7)
            elif m.name ==  'TarX':
                self.TarX = m.value
            elif m.name ==  'TarY':
                self.TarY = m.value




    def idle_task(self):
    #     '''called on idle'''
            tlat, tlon = self.toll(self.TarX,self.TarY)
            self.showIcon('tar', tlat, tlon, 'redstar.png')
            self.console.set_status('tar', 'Tar X %.2f, Tar Y %.2f' % (self.TarX, self.TarY), row=7)

def init(mpstate):
    '''initialise module'''
    return MirModule(mpstate)