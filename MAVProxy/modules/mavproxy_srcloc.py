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
        self.orst = True
        self.xyarr = np.ones((2, self.numsave))*20e7 #for now, initialising to 20deg lat long & 0 strength - i.e. very far from the reading
        self.strearr = np.zeros(self.numsave)
        self.now = 0
        self.prev = 0
        self.gauEPar = np.array([0.5,0,0.5])
        self.gauTPar = np.array([0.25, 0, 0.5])
        self.cov = 0
        self.stre = 0
        self.add_command('sl', self.cmd_sl, "Set source location", ['sl x y'])#['<%s|all>' % x])
        self.add_command('slp', self.cmd_load_pompy, "Load pompy data by number", ['slp no'])#['<%s|all>' % x])
        # self.console.set_status('PlSt', '', row=6)
        # self.showIcon('sl5', 0, 0, 'bluestar.png')
        # self.console.set_status('PlEL', '', row=6)
        # self.showIcon('sl4', 0, 0, 'redstar.png')
        #self.console.set_status('PlTL', '', row=6)
        #self.console.set_status('PlUs', '', row=6)
        self.pompyuse = 0
        self.pompy = np.flipud(np.loadtxt('/home/miche/pompy/ppo/blank.csv', delimiter=',', dtype="float32").T)
        print("No Pompy data loaded.")
        self.datasx = 1000
        self.datasy = 1000
        self.offx = 100
        self.offy = 500
        self.cenx = 875
        self.ceny = 500
        self.maxstr = 1
        self.LLMINV = 89.83204953368922 #lat lon to m inv
        self.DEGTORAD = 3.141592653589793 / 180.0
        plat, plon = self.toll((self.cenx-self.offx)/100,(self.ceny-self.offy)/100)
        self.showIcon('sl5', plat, plon, 'bluestar.png')

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

    def pompy2d(self, lat, lon):
        x, y = self.tom(lat, lon)
        px = x*100 + self.offx #work in cm per "pixel"
        py = y*100 + self.offy
        #self.console.set_status('PlUs', 'PlUs %d %d' % (px, py), row=6)
        if px > self.datasx-1:
            px = self.datasx-1
        if px < 0:
            px = 0
        if py > self.datasy-1:
            py = self.datasy-1
        if py < 0:
            py = 0
        self.console.set_status('Deb', 'Deb x%.0f y%.0f s-px%.0f s-py %.0f' % (x,y,px,py), row=6)
        return self.pompy[int(px),int(py)]

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
            print("SL: Origin lat lon set to: %.0f %.0f" % (self.hlat, self.hlon))
            self.orst = False
        if m.get_type() == 'GLOBAL_POSITION_INT':
            #0.0000001 deg =~ 1 cm, i.e. m.lat & m.lon are approx in cm, thus divide by 100 to get m
            self.now = m.time_boot_ms
            # cx, cy = self.tom(m.lat,m.lon)
            # if abs(cx-((self.cenx-self.offx)/100))<1.0 and abs(cy-((self.ceny-self.offy)/100))<1.0:
            #     print("Within 1m of source.")
            #self.stre = self.gauss2d((m.lat, m.lon), 1, self.slat, self.slon, self.gauTPar[0], self.gauTPar[1], self.gauTPar[2])
            self.stre = self.pompy2d(m.lat, m.lon)
            self.master.mav.plume_strength_send(self.stre/self.maxstr)

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
    def cmd_load_pompy(self,args):
        loaded = True
        if len(args) > 0:
            self.pompyuse = int(args[0])
        else:
            print("No num given. Currently loaded is "+int(self.pompyuse))
        if len(args) > 1:
            self.offx = int(args[1])
            plat, plon = self.toll((self.cenx-self.offx)/100,(self.ceny-self.offy)/100)
            self.showIcon('sl5', plat, plon, 'bluestar.png')
        else:
            print("No offset given. Using default")
        if self.pompyuse == 1:
            print("Loading ori but 1000x1000")
            self.pompy = np.flipud(np.loadtxt('/home/miche/pompy/ppo/leng20_dt0.01_spdup5_wx2_wy0_px5_py0_sprd10_pfrel30_pfsp0.5_pfmax2000_pfmo20000000000.0_ng20_nd0.1_nb0.2_sx700_six1000_scx1000_ar1.csv', delimiter=',', dtype="float32").T)
        elif self.pompyuse == 2:
            print("Loading a slight challenge")
            self.pompy = np.flipud(np.loadtxt('/home/miche/pompy/ppo/leng20_dt0.01_spdup5_wx2_wy0_px5_py0_sprd10_pfrel30_pfsp0.5_pfmax2000_pfmo20000000000.0_ng40_nd0.1_nb0.22_sx700_six1000_scx1000_ar1.csv', delimiter=',', dtype="float32").T)
        elif self.pompyuse == 3:
            print("Loading spotty easy")
            self.pompy = np.flipud(np.loadtxt('/home/miche/pompy/ppo/leng20_dt0.01_spdup5_wx2_wy0_px5_py0_sprd10_pfrel30_pfsp0.05_pfmax2000_pfmo900000000.0_ng20_nd0.1_nb0.2_sx700_six1000_scx1000_ar1.csv', delimiter=',', dtype="float32").T)
        elif self.pompyuse == 4:
            print("Loading spotty slight challenge")
            self.pompy = np.flipud(np.loadtxt('/home/miche/pompy/ppo/leng20_dt0.01_spdup5_wx2_wy0_px5_py0_sprd10_pfrel30_pfsp0.05_pfmax2000_pfmo900000000.0_ng20_nd0.1_nb0.25_sx700_six1000_scx1000_ar1.csv', delimiter=',', dtype="float32").T)
        elif self.pompyuse == 5:
            print("Loading spotty challenge")
            self.pompy = np.flipud(np.loadtxt('/home/miche/pompy/ppo/leng20_dt0.01_spdup5_wx2_wy0_px5_py0_sprd10_pfrel30_pfsp0.5_pfmax2000_pfmo20000000000.0_ng40_nd0.1_nb0.3_sx700_six1000_scx1000_ar1.csv', delimiter=',', dtype="float32").T)
        elif self.pompyuse == 99:
            print("Loading custom from /home/miche/pompy/ppo/")
            self.pompy = np.flipud(np.loadtxt('/home/miche/pompy/ppo/'+args[1], delimiter=',', dtype="float32").T)
        else:
            print("Unknown data number.")
            loaded = False
        if loaded == True:
            self.maxstr = np.amax(self.pompy)
            print("Max strength is", self.maxstr)

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
