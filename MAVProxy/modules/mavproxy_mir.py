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
import re

class MirModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(MirModule, self).__init__(mpstate, "mir", "mir module")
        '''initialisation code'''
        self.add_command('flyto', self.cmd_flyto, "Toggle fly to clicked location", ['flyto'])
        self.add_command('showtar', self.cmd_showtar, "Toggle show target with red star", ['showtar'])
        self.hlat = -353632621   #home location, in (approx) centimetres
        self.hlon = 1491652374
        self.orst = True
        self.tlat = 0
        self.tlon = 0
        self.TarX = 0
        self.TarY = 0
        self.flyto = 0
        self.showtar = True
        self.alt = 600 #current altitude
        self.LLMINV = 89.83204953368922 #lat lon to m inv
        self.DEGTORAD = 3.141592653589793 / 180.0

    def showIcon(self, id, lat, lon, img):
        for mp in self.module_matching('map*'):
            icon = mp.map.icon(img)
            #print("Icon at: %.1f %.1f" % (m.lat, m.lon))
            mp.map.add_object(mp_slipmap.SlipIcon(id, (lat * 1e-7, lon * 1e-7),
                            icon, layer=3, rotation=0, follow=False,
                            trail=mp_slipmap.SlipTrail(colour=(0, 255, 255))))

    def toll(self, x, y): #m to lat, long
        dlat = float(x)*self.LLMINV
        scl = self.lonscl(self.hlat) #just using hlat as it's complex to do avg of both. No issue for small distances
        dlon = (float(y) * self.LLMINV) / scl
        lat = self.hlat + dlat
        lon = self.hlon + dlon
        #x2, y2 = self.tom(lat,lon)
        #print("back to m "+str(x2)+" "+str(y2)) #for testing accuracy
        return lat, lon

    def lonscl(self, lat):
        scale = math.cos(lat * (1.0e-7 * self.DEGTORAD))
        return max(scale, 0.01)

    def tom(self, lat, lon): # to metres
        x = (lat-self.hlat) * (1/self.LLMINV)
        y = (lon-self.hlon) * (1/self.LLMINV) * self.lonscl((lat+self.hlat)/2)
        return x, y

    def mavlink_packet(self, m):
        'handle a MAVLink packet'''
        if m.get_type() == 'GPS_GLOBAL_ORIGIN' and self.orst:
            # message is sent when origin is initially set and when arming, apparently. Unfortunately no apparent way of triggering it.
            self.hlat = m.latitude
            self.hlon = m.longitude
            print("MIR: Origin lat lon set to: %.0f %.0f" % (self.hlat, self.hlon))
            self.orst = False
        if m.get_type() == 'GLOBAL_POSITION_INT':
            self.alt = m.alt
        if m.get_type() == 'NAMED_VALUE_FLOAT':
            if m.name == "PLUR":
                self.console.set_status(m.name, m.name + ' %0.7f' % m.value, row=8)
            elif m.name == "PLUS":
                self.console.set_status(m.name, m.name + ' %0.7f' % m.value, row=8)
            elif re.match(r'^FINI', m.name):
                self.console.set_status(m.name, m.name + ' %.2f' % m.value, row=7)
            elif re.match(r'^BSC', m.name): #generally no longer sent
                 self.console.set_status(m.name, m.name + ' %.2f' % m.value, row=8)
            elif m.name ==  'TarX':
                self.TarX = m.value
            elif m.name ==  'TarY':
                self.TarY = m.value
            elif re.match(r'^ADRC', m.name):
                na0 = ' %.2f' % m.value
                na1 = na0.rjust(10)
                na = m.name + na1
                if re.match(r"^ADRC...1", m.name):
                    self.console.set_status(m.name, na, row=9)
                elif re.match(r"^ADRC...2", m.name):
                    self.console.set_status(m.name, na, row=10)
                else:
                    self.console.set_status(m.name, na, row=11)

    def cmd_flyto(self, args):
        if self.flyto == 1:
            self.flyto = 0
            print("Flyto off")
        else:
            self.flyto = 1
            print("Flyto on")

    def cmd_showtar(self, args):
        if self.showtar == False:
            self.showtar = True
            print("Showtar on")
        else:
            self.showtar = False
            print("Showtar off")

    def idle_task(self):
#     '''called on idle'''
        tlat, tlon = self.toll(self.TarX,self.TarY)
        self.showIcon('tar', tlat, tlon, 'redstar.png')
        self.console.set_status('tar', 'Tar X %.2f, Tar Y %.2f' % (self.TarX, self.TarY), row=7)

        # tnow_ms = int((time.time() - self.mpstate.start_time_s)*1000)
        # name = "testing"
        # value = float(335)
        # self.master.mav.named_value_float_send(tnow_ms, name.encode("utf-8"), value)

        if self.flyto == 1:
            latlon = self.mpstate.click_location
            if latlon is not None:
                tlat = int(latlon[0]*1.0e7)
                tlon = int(latlon[1]*1.0e7)
                if tlat != self.tlat or tlon != self.tlon:
                    self.tlat = tlat
                    self.tlon = tlon
                    # print("Sending target loc: %d %d %d %d" % (self.tlat, self.tlon, tlat, tlon))
                    self.master.mav.plume_est_loc_send(self.tlat, self.tlon, self.alt, 0.999) #actually just sending the target location
                    self.showIcon('sl5', self.tlat, self.tlon, 'bluestar.png')

def init(mpstate):
    '''initialise module'''
    return MirModule(mpstate)