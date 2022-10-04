 #!/usr/bin/env python
'''Srcloc'''

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.mavproxy_map import mp_slipmap
import numpy as np
import math
import re

#use self.say("Flight battery warning") to print in console

#note: this module is for all simulated plume stuff.
class SrclocModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(SrclocModule, self).__init__(mpstate, "srcloc", "srcloc module", multi_vehicle=True)
        '''initialisation code'''
        self.LLMINV = 89.83204953368922 #lat lon to m inv
        self.DEGTORAD = 3.141592653589793 / 180.0
        self.hlat = -352802523   #home location, in (approx) centimetres
        self.hlon = 1490058409
        self.orst = False #change to true to set origin on first GPS_GLOBAL_ORIGIN message
        self.gauTPar = np.array([0.25, 0, 0.5])
        self.add_command('sl', self.cmd_sl, "Set source location", ['sl x y'])#['<%s|all>' % x])
        self.add_command('slp', self.cmd_load_pompy, "Load pompy data by number", ['slp no'])#['<%s|all>' % x])
        self.pompyuse = 0 # 0 means use Gaussian
        self.maxstr = 1
        self.pompy = np.flipud(np.loadtxt('/home/miche/pompy/ppo/blank.csv', delimiter=',', dtype="float32").T)
        print("No Pompy data loaded. Using gaussian.")
        self.datasx = 1000
        self.datasy = 1000
        self.offx = 100 #x is North
        self.offy = 500
        self.cenx = 920
        self.ceny = 500
        self.slat, self.slon = self.toll((self.cenx-self.offx)/100,(self.ceny-self.offy)/100)
        self.gbests = np.zeros(11)

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
        #assumes lat & lon in int form, not standard
        for mp in self.module_matching('map*'):
            icon = mp.map.icon(img)
            #print("Icon at: %.1f %.1f" % (m.lat, m.lon))
            mp.map.add_object(mp_slipmap.SlipIcon(id, (lat * 1e-7, lon * 1e-7),
                            icon, layer=3, rotation=0, follow=False,
                            trail=mp_slipmap.SlipTrail(colour=(0, 255, 255))))

    def changeHome(self, lat, lon):
        self.hlat = lat
        self.hlon = lon
        self.orst = False #just to make sure GPS_GLOBAL_ORIGIN doesn't change it if it is manually set
        self.slat, self.slon = self.toll((self.cenx-self.offx)/100,(self.ceny-self.offy)/100)
        print("SL: Home/origin lat lon set to: %.0f %.0f, source lat lon set to %.0f %.0f" % (self.hlat, self.hlon, self.slat, self.slon))

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
            self.changeHome(m.latitude, m.longitude)
        if m.get_type() == 'NAMED_VALUE_FLOAT':
            if m.name == 'GBEST':
                sysid = m.get_srcSystem()
                self.console.set_status('gbest%d' % sysid, 'GBest acc-to %d is %0.0f' % (sysid, m.value), row=6+int((sysid-1)/3))
                self.gbests[sysid] = m.value
        if m.get_type() == 'DEBUG_LOC':
            if re.match(r'^PBEST', m.name):
                id = int(m.name[5:])
                sysid = m.get_srcSystem()
                # Disabled the line below as the redstars work well for display
                # self.console.set_status('pbest%d%d' % (m.get_srcSystem(),id), 'PBest acc-to %0.0f for %d is %d %d ' % (m.get_srcSystem(), id, m.lat, m.lon), row=(7+id))
                if int(self.gbests[sysid]) == id:
                    self.showIcon('pbestst%d%d' % (sysid,id), m.lat, m.lon, 'orangestar.png')
                else: 
                    self.showIcon('pbestst%d%d' % (sysid,id), m.lat, m.lon, 'redstar.png')
                    # print('Red star shown because gbest for sysid%d is gbest%d, not id%d' % (sysid, self.gbests[sysid], id))
        if m.get_type() == 'GLOBAL_POSITION_INT':
            #0.0000001 deg =~ 1 cm, i.e. m.lat & m.lon are approx in cm
            sysid = m.get_srcSystem()
            if self.pompyuse == 0: stre = self.gauss2d((m.lat, m.lon), 1, self.slat, self.slon, self.gauTPar[0], self.gauTPar[1], self.gauTPar[2])
            else: stre = self.pompy2d(m.lat, m.lon)
            self.master.mav.plume_strength_send(sysid, stre/self.maxstr)
            self.console.set_status('sysid%d' % sysid, 'PLUS %0.7f sysid %d ' % (stre/self.maxstr, sysid), row=6+int((sysid-1)/3))

    def cmd_sl(self, args):
        '''handle Source Location setting'''
        if len(args) == 0:
             print("Usage: set x y | sethome lat lon | tpar a b c | ppar | flyto")
        else:
            if args[0] == 'set':
                if self.pompyuse != 0:
                    if len(args) == 3:
                        self.slat, self.slon = self.toll(args[1],args[2])
                    else: 
                        latlon = self.mpstate.click_location
                        if latlon is not None:
                            self.slat = int(latlon[0]*1.0e7)
                            self.slon = int(latlon[1]*1.0e7)
                        else:
                            print('Click on map before running command to set location')
                    print('Set gauss source location to %.7f %.7f.' % (self.slat*1e-7, self.slon*1e-7))
                else: print('Moving source only possible with Gaussian. For Pompy, must move home instead.')
            if args[0] == 'sethome':
                if len(args) == 3:
                    lat = int(float(args[1])*1.0e7)
                    lon = int(float(args[2])*1.0e7)
                    self.changeHome(lat, lon)
                else:
                    latlon = self.mpstate.click_location
                    if latlon is not None:
                        lat = int(latlon[0]*1.0e7)
                        lon = int(latlon[1]*1.0e7)
                        self.changeHome(lat, lon)
                    else:
                        print('Click on map before running command to set location')
            if args[0] == 'tpar':
                self.gauTPar[0] = float(args[1])
                self.gauTPar[1] = float(args[2])
                self.gauTPar[2] = float(args[3])
                print('Set source true params to %.2f %.2f %0.2f' % tuple(self.gauTPar))
            if args[0] == 'ppar':
                print('Source true params are %.2f %.2f %0.2f' %  tuple(self.gauTPar))
    def cmd_load_pompy(self,args):
        if len(args) > 0:
            self.pompyuse = int(args[0])
        else:
            print("No num given. Currently loaded is", self.pompyuse)
        if len(args) > 1:
            if self.pompyuse == 99:
                self.offx = int(args[2])
            else: 
                self.offx = int(args[1])
            self.slat, self.slon = self.toll((self.cenx-self.offx)/100,(self.ceny-self.offy)/100)
        else:
            print("No offset given. Using default")
        filegiven = True
        if self.pompyuse == 0:
            print("Using Gaussian")
            filegiven = False
            self.maxstr = 1
        elif self.pompyuse == 1:
            print("Loading ori but 1000x1000")
            filename = 'leng20_dt0.01_spdup5_wx2_wy0_px5_py0_sprd10_pfrel30_pfsp0.5_pfmax2000_pfmo20000000000.0_ng20_nd0.1_nb0.2_sx700_six1000_scx1000_ar1'
        elif self.pompyuse == 2:
            print("Loading a slight challenge")
            filename = 'leng20_dt0.01_spdup5_wx2_wy0_px5_py0_sprd10_pfrel30_pfsp0.5_pfmax2000_pfmo20000000000.0_ng40_nd0.1_nb0.22_sx700_six1000_scx1000_ar1'
        elif self.pompyuse == 3:
            print("Loading spotty easy")
            filename = 'leng20_dt0.01_spdup5_wx2_wy0_px5_py0_sprd10_pfrel30_pfsp0.05_pfmax2000_pfmo900000000.0_ng20_nd0.1_nb0.2_sx700_six1000_scx1000_ar1'
        elif self.pompyuse == 4:
            print("Loading spotty slight challenge")
            filename = 'leng20_dt0.01_spdup5_wx2_wy0_px5_py0_sprd10_pfrel30_pfsp0.05_pfmax2000_pfmo900000000.0_ng20_nd0.1_nb0.25_sx700_six1000_scx1000_ar1'
        # elif self.pompyuse == 5:
        #     print("Loading spotty challenge")
        #     filename = 'leng20_dt0.01_spdup5_wx2_wy0_px5_py0_sprd10_pfrel30_pfsp0.5_pfmax2000_pfmo20000000000.0_ng40_nd0.1_nb0.3_sx700_six1000_scx1000_ar1'
        elif self.pompyuse == 6:
            print("Loading a challenge")
            filename = 'leng20_dt0.01_spdup5_wx2_wy0_px5_py0_sprd10_pfrel30_pfsp0.5_pfmax2000_pfmo20000000000.0_ng40_nd0.1_nb0.3_sx700_six1000_scx1000_ar1'
        elif self.pompyuse == 7:
            print("Loading spotty2 easy")
            filename = 'leng20_dt0.01_spdup5_wx3_wy0_px5_py0_sprd10_pfrel30_pfsp0.004_pfmax2000_pfmo900000000.0_crd3_ng20_nd0.1_nb0.2_sx1000_six1000_scx1000_ar1_f20'
        elif self.pompyuse == 8:
            print("Loading spotty2 medium")
            filename = 'leng20_dt0.01_spdup5_wx3_wy0_px5_py0_sprd10_pfrel30_pfsp0.004_pfmax2000_pfmo900000000.0_crd5_ng20_nd0.1_nb0.2_sx1000_six1000_scx1000_ar1_f20'
        elif self.pompyuse == 9:
            print("Loading spotty2 hard")
            filename = 'leng20_dt0.01_spdup5_wx3_wy0_px5_py0_sprd10_pfrel30_pfsp0.002_pfmax2000_pfmo900000000.0_crd5_ng20_nd0.1_nb0.2_sx1000_six1000_scx1000_ar1_f20'
        elif self.pompyuse == 10:
            print("Loading spotty2 very hard")
            filename = 'leng20_dt0.01_spdup5_wx3_wy0_px5_py0_sprd10_pfrel30_pfsp0.002_pfmax2000_pfmo900000000.0_crd10_ng20_nd0.1_nb0.2_sx1000_six1000_scx1000_ar1_f20'
        elif self.pompyuse == 99:
            print("Loading custom from /home/miche/pompy/ppo/ (no extension, must give offset)")
            filename = args[1]
        else:
            print("Unknown data number.")
            filegiven = False
        if filegiven == True: #0 = no file, 1 = file given but not loaded
            self.pompy = np.flipud(np.loadtxt('/home/miche/pompy/ppo/'+filename+'.csv', delimiter=',', dtype="float32").T)
            self.maxstr = np.amax(self.pompy)
            self.cenx, self.ceny = np.unravel_index(np.argmax(self.pompy, axis=None), self.pompy.shape)
            print("Max strength is", self.maxstr, "centre pos is", self.cenx, self.ceny)
            self.slat, self.slon = self.toll((self.cenx-self.offx)/100,(self.ceny-self.offy)/100)

    def idle_task(self):
        '''called on idle'''
        #this is displayed from the idle task to ensure it is always updated to the current slat, slon variables.
        self.showIcon('srcpos', self.slat, self.slon, 'bluestar.png')

#latitude means north
#Latitude: -35.282001
def init(mpstate):
    '''initialise module'''
    return SrclocModule(mpstate)
