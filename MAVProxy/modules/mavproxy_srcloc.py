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

class SrclocModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(SrclocModule, self).__init__(mpstate, "srcloc", "srcloc module")
        '''initialisation code'''
        self.hlat = -353632621   #home location, in (approx) centimetres
        self.hlon = 1491652374
        self.slat = self.hlat+400   #source location, ~4m north
        self.slon = self.hlon-200   # ~2m west
        self.elat = 0 #estimated source location
        self.elon = 0 
        self.upto = 0
        self.done10 = 0
        self.numsave = 200
        self.xyarr = np.ones((2, self.numsave))*20e7 #for now, initialising to 20deg lat long & 0 strength - i.e. very far from the reading
        self.strearr = np.zeros(self.numsave)
        self.now = 0
        self.prev = 0
        self.gauEPar = np.array([0.5,0,0.5])
        self.gauTPar = np.array([0.25, 0, 0.5])
        self.cov = 0
        self.stre = 0
        self.add_command('sl', self.cmd_sl, "Set source location", ['sl x y'])#['<%s|all>' % x])
        # self.console.set_status('SRLoc', 'SRLoc %.7f %.7f' % (self.slat*1e-7, self.slon*1e-7), row=5)
        # self.console.set_status('SELoc', 'SELoc --- ---', row=5)
        self.console.set_status('PlSt', 'Plume Strength ---', row=5)
        self.showIcon(4, self.slat, self.slon, 'redstar.png') #true location of source
        self.console.set_status('PlTL', 'PlTL %.7f %.7f' % (self.slat*1e-7, self.slon*1e-7), row=5)

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
            # message is sent when origin is initially set and when arming, apparently. Unfortunately no apparent way of triggering it.
            # this means that currently the module must be started before the EKF sets its origin if the vehicle is started anywhere other than CMAC.
            self.hlat = m.latitude
            self.hlon = m.longitude
            print("Origin lat lon set to: %.0f %.0f" % (self.hlat, self.hlon))
        if m.get_type() == 'GLOBAL_POSITION_INT':
            #0.0000001 deg =~ 1 cm, i.e. m.lat & m.lon are approx in cm, thus divide by 100 to get m
            self.now = m.time_boot_ms
            self.stre = self.gauss2d((m.lat, m.lon), 1, self.slat, self.slon, self.gauTPar[0], self.gauTPar[1], self.gauTPar[2])
            if (self.now - self.prev) > 0.7e3:
                #print("Running", self.now - self.prev)
                self.xyarr[:,self.upto] = m.lat, m.lon
                self.strearr[self.upto] = self.stre
                self.upto = self.upto + 1
                if self.upto > 19 & self.done10 == 0:
                    self.done10 = 1
                    print("Starting SrcLoc")
                if self.upto > (self.numsave-1):
                    self.upto = 0
                self.prev = self.now
            self.master.mav.plume_strength_send(self.stre)
            if self.done10 == 1:
                i = self.strearr.argmax()
                guess = [1, self.xyarr[0,i], self.xyarr[1,i], self.gauEPar[0],self.gauEPar[1], self.gauEPar[2]]
                pred_params, uncert_cov = opt.curve_fit(self.gauss2d, self.xyarr, self.strearr, p0=guess)
                #print('Uncert_cov ', np.mean(uncert_cov), 'max ',np.nanmax(uncert_cov), 'min ',np.amin(uncert_cov))
                self.elat = pred_params[1]
                self.elon = pred_params[2]
                self.cov = np.mean(abs(uncert_cov))
                self.master.mav.plume_est_loc_send(self.elat, self.elon, m.alt, self.cov)
                # self.master.mav.plume_par_send(self.gauEPar[0], self.gauEPar[1], self.gauEPar[2], self.gauTPar[0], self.gauTPar[1], self.gauTPar[2])
        # if m.get_type() == 'PLUME_EST_LOC':
        #     self.elat = self.hlat + m.x*111
        #     self.elon = self.hlon + m.y*111

    def cmd_sl(self, args):
        '''handle Source Location setting'''
        if len(args) == 0:
             print("Usage: set x y | tpar a b c | epar a b c | ppar")
        else:
            if args[0] == 'set':
                lato = int(float(args[1])*89.83204953368922)
                lono = int(float(args[2])*89.83204953368922)
                self.slat = self.hlat + lato
                self.slon = self.hlon + lono
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
    
    def idle_task(self):
    #     '''called on idle'''
    #     # update status for detected plume strength
        self.console.set_status('PlSt', 'Plume Strength %.7f ut%d' % (self.stre, self.upto), row=5)
    #     # update icon & status for estimated location of source
        self.showIcon(5, self.elat, self.elon, 'bluestar.png')
        self.console.set_status('PlEL', 'PlEL %.7f %.7f cov %.4f' % (self.elat*1e-7, self.elon*1e-7, self.cov), row=5)
    #     # update icon & status for true location of source
        self.showIcon(4, self.slat, self.slon, 'redstar.png')
        self.console.set_status('PlTL', 'PlTL %.7f %.7f' % (self.slat*1e-7, self.slon*1e-7), row=5)

#latitude means north
#Latitude: -35.282001 
def init(mpstate):
    '''initialise module'''
    return SrclocModule(mpstate)