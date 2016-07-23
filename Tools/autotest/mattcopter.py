# fly ArduCopter in SITL
# Flight mode switch positions are set-up in arducopter.param to be
#   switch 1 = Circle
#   switch 2 = Land
#   switch 3 = RTL
#   switch 4 = Auto
#   switch 5 = Loiter
#   switch 6 = Stabilize

import util, pexpect, sys, time, math, shutil, os
from common import *
from arducopter import land, setup_rc, hover, arm_motors, disarm_motors, takeoff, change_alt
from pymavlink import mavutil, mavwp
import random

# get location of scripts
testdir=os.path.dirname(os.path.realpath(__file__))

FRAME='+'
HOME=mavutil.location(-33.8869497,151.1921042,30,180)

homeloc = None
num_wp = 0
speedup_default = 5

# fly a set of manoeuvres
def fly_manoeuvres(mavproxy, mav, override_mode=0, timeout=300):
    '''Fly a series of twitches to see what happens'''
    tstart = get_sim_time(mav)
    success = False

    # ensure all sticks in the middle
    mavproxy.send('rc 1 1500\n')
    mavproxy.send('rc 2 1500\n')
    mavproxy.send('rc 3 1500\n')
    mavproxy.send('rc 4 1500\n')

    # switch to loiter mode to attain test height
    mavproxy.send('mode loiter\n')
    wait_mode(mav, 'LOITER')

    # fly to 10 m
    if not change_alt(mavproxy, mav, 10):
        failed_test_msg = "failed to attain test height"
        print(failed_test_msg)
        failed = True

    # switch to manoeuvre mode 
    mavproxy.send('mode 19\n')
    wait_mode(mav, 'Mode(19)'); # it never gets this because it's not a named mode yet

    # Make sure everything is centred so that manoeuvres can start
    mavproxy.send('rc 1 1500\n')
    mavproxy.send('rc 2 1500\n')
    mavproxy.send('rc 3 1500\n')
    mavproxy.send('rc 4 1500\n')
    
    # wait for stuff to start happening
    if override_mode:
        # Test roll override
        wait_seconds(mav, 2.5)
        mavproxy.send('rc 1 1400\n')
        wait_seconds(mav, 1)
        mavproxy.send('rc 1 1500\n')
        
        # Test pitch override
        wait_seconds(mav, 5.5)
        mavproxy.send('rc 2 1400\n')
        wait_seconds(mav, 1)
        mavproxy.send('rc 2 1500\n')
        
        # Test throttle override
        wait_seconds(mav, 2.5)
        mavproxy.send('rc 3 1600\n')
        wait_seconds(mav, 1)
        mavproxy.send('rc 3 1500\n')
        
        # Test yaw override
        wait_seconds(mav, 2.5)
        mavproxy.send('rc 4 1400\n')
        wait_seconds(mav, 1)
        mavproxy.send('rc 4 1500\n')

    # Wait for things to do their thing.  Ideally it would wait for a trigger to continue
    wait_seconds(mav, 30)
    
    # Switch to loiter mode
    mavproxy.send('mode loiter\n')
    wait_mode(mav, 'LOITER')
    
    success = True
    return success

def fly_ArduCopter(binary, viewerip=None, map=False, valgrind=False, gdb=False):
    '''fly ArduCopter in SIL

    you can pass viewerip as an IP address to optionally send fg and
    mavproxy packets too for local viewing of the flight in real time
    '''
    global homeloc

    home = "%f,%f,%u,%u" % (HOME.lat, HOME.lng, HOME.alt, HOME.heading)
    sil = util.start_SIL(binary, wipe=True, model='+', home=home, speedup=speedup_default)
    mavproxy = util.start_MAVProxy_SIL('ArduCopter', options='--sitl=127.0.0.1:5501 --out=127.0.0.1:19550 --quadcopter')
    mavproxy.expect('Received [0-9]+ parameters')

    # setup test parameters
    mavproxy.send("param load %s/copter_params_matt.parm\n" % testdir)
    mavproxy.expect('Loaded [0-9]+ parameters')
    mavproxy.send("param set LOG_REPLAY 0\n")
    mavproxy.send("param set LOG_DISARMED 0\n")
    time.sleep(3)

    # reboot with new parameters
    util.pexpect_close(mavproxy)
    util.pexpect_close(sil)

    sil = util.start_SIL(binary, model='+', home=home, speedup=speedup_default, valgrind=valgrind, gdb=gdb)
    options = '--sitl=127.0.0.1:5501 --out=127.0.0.1:19550 --quadcopter --streamrate=5'
    if viewerip:
        options += ' --out=%s:14550' % viewerip
    if map:
        options += ' --map'
    mavproxy = util.start_MAVProxy_SIL('ArduCopter', options=options)
    mavproxy.expect('Telemetry log: (\S+)')
    logfile = mavproxy.match.group(1)
    print("LOGFILE %s" % logfile)

    buildlog = util.reltopdir("../buildlogs/ArduCopter-test.tlog")
    print("buildlog=%s" % buildlog)
    copyTLog = False
    if os.path.exists(buildlog):
        os.unlink(buildlog)
    try:
        os.link(logfile, buildlog)
    except Exception:
        print( "WARN: Failed to create symlink: " + logfile + " => " + buildlog + ", Will copy tlog manually to target location" )
        copyTLog = True

    # the received parameters can come before or after the ready to fly message
    mavproxy.expect(['Received [0-9]+ parameters', 'Ready to FLY'])
    mavproxy.expect(['Received [0-9]+ parameters', 'Ready to FLY'])

    util.expect_setup_callback(mavproxy, expect_callback)

    expect_list_clear()
    expect_list_extend([sil, mavproxy])

    # get a mavlink connection going
    try:
        mav = mavutil.mavlink_connection('127.0.0.1:19550', robust_parsing=True)
    except Exception, msg:
        print("Failed to start mavlink connection on 127.0.0.1:19550" % msg)
        raise
    mav.message_hooks.append(message_hook)
    mav.idle_hooks.append(idle_hook)


    failed = False
    failed_test_msg = "None"

    try:
        mav.wait_heartbeat()
        setup_rc(mavproxy)
        homeloc = mav.location()

        # wait 10sec to allow EKF to settle
        wait_seconds(mav, 30)

        # Arm
        print("# Arm motors")
        if not arm_motors(mavproxy, mav):
            failed_test_msg = "arm_motors failed"
            print(failed_test_msg)
            failed = True

        # Take off
        print("# Takeoff")
        if not takeoff(mavproxy, mav, 10):
            failed_test_msg = "takeoff failed"
            print(failed_test_msg)
            failed = True

        # Fly a set of maneuvours for Systems ID
        print("#")
        print("########## Fly maneuvours file ##########")
        print("#")
        override_test = 0;
        if not fly_manoeuvres(mavproxy, mav, override_test):
            failed_test_msg = "fly_manoeuvres failed"
            print(failed_test_msg)
            failed = True
        else:
            print("Flew copter maneuvours OK")
            
        # Check override of maneuvours
        print("#")
        print("########## Fly maneuvours file with overrides ##########")
        print("#")
        override_test = 1;
        if not fly_manoeuvres(mavproxy, mav, override_test):
            failed_test_msg = "fly_manoeuvres override failed"
            print(failed_test_msg)
            failed = True
        else:
            print("Overrode copter maneuvours OK")

        # land aircraft to finish flight
        land(mavproxy, mav)

        # wait for disarm
        mav.motors_disarmed_wait()

        if not log_download(mavproxy, mav, util.reltopdir("../buildlogs/ArduCopter-log.bin")):
            failed_test_msg = "log_download failed"
            print(failed_test_msg)
            failed = True

    except pexpect.TIMEOUT, failed_test_msg:
        failed_test_msg = "Timeout"
        failed = True

    mav.close()
    util.pexpect_close(mavproxy)
    util.pexpect_close(sil)

    valgrind_log = sil.valgrind_log_filepath()
    if os.path.exists(valgrind_log):
        os.chmod(valgrind_log, 0644)
        shutil.copy(valgrind_log, util.reltopdir("../buildlogs/ArduCopter-valgrind.log"))

    # [2014/05/07] FC Because I'm doing a cross machine build (source is on host, build is on guest VM) I cannot hard link
    # This flag tells me that I need to copy the data out
    if copyTLog:
        shutil.copy(logfile, buildlog)
        
    if failed:
        print("FAILED: %s" % failed_test_msg)
        return False
    return True