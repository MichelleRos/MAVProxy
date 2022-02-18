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
import scipy.optimize as opt
import math

class SrclocModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(SrclocModule, self).__init__(mpstate, "srcloc", "srcloc module")
        '''initialisation code'''
        self.hlat = -353632621   #home location, in (approx) centimetres
        self.hlon = 1491652374
        self.slat = self.hlat+200   #(only used for Gaussian) North-South #source location (currently just an offset from home for start point)
        self.slon = self.hlon+100  #East-West
        self.elat = 0 #estimated source location
        self.elon = 0
        self.upto = 0
        self.done10 = 0
        self.numsave = 200
        self.dosl = 0
        self.xyarr = np.ones((2, self.numsave))*20e7 #for now, initialising to 20deg lat long & 0 strength - i.e. very far from the reading
        self.strearr = np.zeros(self.numsave)
        self.now = 0
        self.prev = 0
        self.gauEPar = np.array([0.5,0,0.5])
        self.gauTPar = np.array([0.25, 0, 0.5])
        self.cov = 0
        self.stre = 0
        self.add_command('sl', self.cmd_sl, "Set source location", ['sl x y'])#['<%s|all>' % x])
        self.console.set_status('PlSt', '', row=6)
        self.showIcon('sl5', 0, 0, 'bluestar.png')
        self.console.set_status('PlEL', '', row=6)
        self.showIcon('sl4', 0, 0, 'redstar.png')
        self.console.set_status('PlTL', '', row=6)
        self.console.set_status('PlUs', '', row=6)
        self.pompy = np.loadtxt('/home/miche/pompy/ppo/datax.csv', delimiter=',', dtype="float32")
        self.pompy = np.flipud(self.pompy.T)
        self.datasx = 1000
        self.datasy = 500
        self.mult = 1
        self.maxstr = np.amax(self.pompy)
        #self.maxstr = 1
        print("Max strength is", self.maxstr)

    #[ab;bc] "is essentially 0.5 over the covariance matrix", A is the amplitude, and (x0, y0) is the center
    def gauss2d(self, xy, amp, x0, y0, a, b, c):
        x, y = xy
        x = (x - self.hlat)*1e-3
        y = (y - self.hlon)*1e-3
        x0 = (x0 - self.hlat)*1e-3
        y0 = (y0 - self.hlon)*1e-3
        inner = a * (x - x0)**2
        inner += 2 * b * (x - x0) * (y - y0)    #b is diagonal variance
        inner += c * (y - y0)**2
        return amp * np.exp(-inner)

    def pompy2d(self, xy):
        x, y = xy
        x = x - self.hlat #work in approx cm per pixel, from home point
        y = y - self.hlon
        x = round(x/self.mult)
        y = round(y/self.mult)
        px = x + 100
        py = y + 250
        self.console.set_status('PlUs', 'PlUs %d %d' % (px, py), row=6)
        if px > self.datasx-1:
            px = self.datasx-1
        if px < 0:
            px = 0
        if py > self.datasy-1:
            py = self.datasy-1
        if py < 0:
            py = 0
        self.console.set_status('Deb', 'Deb x%.0f y%.0f s-px%.0f s-py %.0f' % (x,y,px,py), row=6)
        return self.pompy[px,py]

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
        if m.get_type() == 'GLOBAL_POSITION_INT':
            #0.0000001 deg =~ 1 cm, i.e. m.lat & m.lon are approx in cm, thus divide by 100 to get m
            self.now = m.time_boot_ms
            #self.stre = self.gauss2d((m.lat, m.lon), 1, self.slat, self.slon, self.gauTPar[0], self.gauTPar[1], self.gauTPar[2])
            self.stre = self.pompy2d((m.lat, m.lon))
            self.master.mav.plume_strength_send(self.stre)

            if (self.now - self.prev) > 0.7e3:
                #print("Running", self.now - self.prev)
                self.xyarr[:,self.upto] = m.lat, m.lon
                self.strearr[self.upto] = self.stre
                self.upto = self.upto + 1
                if self.upto > 19 & self.done10 == 0:
                    self.done10 = 1
                    print("Ready for SrcLoc Est")
                if self.upto > (self.numsave-1):
                    self.upto = 0
                self.prev = self.now
            if self.done10 == 1 & self.dosl == 1: #This only runs if target loc hasn't been sent
                i = self.strearr.argmax()
                guess = [1, self.xyarr[0,i], self.xyarr[1,i], self.gauEPar[0],self.gauEPar[1], self.gauEPar[2]]
                pred_params, uncert_cov = opt.curve_fit(self.gauss2d, self.xyarr, self.strearr, p0=guess)
                self.elat = pred_params[1]
                self.elon = pred_params[2]
                self.cov = np.mean(abs(uncert_cov))
                #print("Sending plume loc: %.2f %.2f %.2f %.2f" % (self.elat, self.elon, m.alt, self.cov))
                self.master.mav.plume_est_loc_send(int(self.elat), int(self.elon), int(m.alt), self.cov)
                # self.master.mav.plume_par_send(self.gauEPar[0], self.gauEPar[1], self.gauEPar[2], self.gauTPar[0], self.gauTPar[1], self.gauTPar[2]) #removed this mavlink message
        if m.get_type() == 'PLUME_EST_LOC':
            #for use with onboard source estimation
            self.elat, self.elon = self.toll(m.x,m.y)
            self.showIcon('sl5', self.elat, self.elon, 'bluestar.png')
            self.console.set_status('PlEL', 'PlEL %.7f %.7f cov %.4f' % (self.elat*1e-7, self.elon*1e-7, self.cov), row=6)

    def cmd_sl(self, args):
        '''handle Source Location setting'''
        if len(args) == 0:
             print("Usage: set x y | tpar a b c | epar a b c | ppar | setp lat lon | flyto on/off | dosrcloc on/off")
        else:
            if args[0] == 'set':
                self.slat, self.slon = self.toll(args[1],args[2])
                self.xyarr = np.ones((2, self.numsave))*20e7
                self.strearr = np.zeros(self.numsave)
                self.upto = 0
                #print('Set source to %.7f %.7f' % (self.slat*1e-7, self.slon*1e-7))
            if args[0] == 'tpar':
                self.gauTPar[0] = float(args[1])
                self.gauTPar[1] = float(args[2])
                self.gauTPar[2] = float(args[3])
                print('Set source true params to %.2f %.2f %0.2f' % tuple(self.gauTPar))
            if args[0] == 'epar':
                self.gauEPar[0] = float(args[1])
                self.gauEPar[1] = float(args[2])
                self.gauEPar[2] = float(args[3])
                print('Set source estimated params to %.2f %.2f %0.2f' % tuple(self.gauEPar))
            if args[0] == 'ppar':
                print('Source estimated params are %.2f %.2f %0.2f' % tuple(self.gauEPar))
                print('Source true params are %.2f %.2f %0.2f' %  tuple(self.gauTPar))
            if args[0] == 'setp':
                tlat, tlon = self.toll(args[1],args[2])
                self.dosl = 0
                print("Sending target loc: %.2f %.2f" % (tlat, tlon))
                self.master.mav.plume_est_loc_send(int(tlat), int(tlon), 700, 0.999)
                self.showIcon('sl5', tlat, tlon, 'bluestar.png')
            if args[0] == 'setmult':
                    self.mult = float(args[1])
            if args[0] == 'dosrcloc': #fly to estimated source position
                if args[1] == 'on':
                    self.dosl = 1
                if args[1] == 'off':
                    self.dosl = 0
                    self.showIcon('sl5', 0, 0, 'bluestar.png')
                    self.console.set_status('PlEL', '', row=6)
                    self.showIcon('sl4', 0, 0, 'redstar.png')
                    self.console.set_status('PlTL', '', row=6)

    def idle_task(self):
    #     '''called on idle'''
    #     # update status for detected plume strength
        self.console.set_status('PlSt', 'PS %.7f ut%d' % (self.stre/self.maxstr, self.upto), row=6)
    #     # update icon & status for estimated location of source
        if self.dosl == 1:
            self.showIcon('sl5', self.elat, self.elon, 'bluestar.png')
            self.console.set_status('PlEL', 'PlEL %.7f %.7f cov %.4f' % (self.elat*1e-7, self.elon*1e-7, self.cov), row=6)
        #     # update icon & status for true location of source
            self.showIcon('sl4', self.slat, self.slon, 'redstar.png')
            self.console.set_status('PlTL', 'PlTL %.7f %.7f' % (self.slat*1e-7, self.slon*1e-7), row=6)

#latitude means north
#Latitude: -35.282001
def init(mpstate):
    '''initialise module'''
    return SrclocModule(mpstate)
