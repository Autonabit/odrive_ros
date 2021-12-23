import odrive
from odrive.enums import *
import time
from fibre.libfibre import ObjectLostError

if False: #True:
    VEL_GAIN = 10
    VEL_INTEGRATOR_GAIN = 0.5
else:
    VEL_GAIN = 5
    VEL_INTEGRATOR_GAIN = 0

VEL_INTEGRATOR_LIMIT = 1000000
CURRENT_LIM = 30

CURRENT_CONTROL_BANDWIDTH = 100
ENCODER_BANDWIDTH = 100

def set_gains(odriv):
    odriv.axis0.controller.config.vel_gain = VEL_GAIN
    odriv.axis1.controller.config.vel_gain = VEL_GAIN

    odriv.axis0.controller.config.vel_integrator_gain = VEL_INTEGRATOR_GAIN
    odriv.axis1.controller.config.vel_integrator_gain = VEL_INTEGRATOR_GAIN

    odriv.axis0.controller.config.vel_integrator_limit = VEL_INTEGRATOR_LIMIT
    odriv.axis1.controller.config.vel_integrator_limit = VEL_INTEGRATOR_LIMIT

    odrv0.axis0.motor.config.current_lim = CURRENT_LIM
    odrv0.axis1.motor.config.current_lim = CURRENT_LIM

    odrv0.axis0.motor.config.current_control_bandwidth = CURRENT_CONTROL_BANDWIDTH
    odrv0.axis1.motor.config.current_control_bandwidth = CURRENT_CONTROL_BANDWIDTH

    odriv.axis0.controller.config.spinout_electrical_power_threshold = 20
    odriv.axis1.controller.config.spinout_electrical_power_threshold = 20

    odriv.axis0.controller.config.spinout_mechanical_power_threshold = -20
    odriv.axis1.controller.config.spinout_mechanical_power_threshold = -20

    odrv0.axis0.encoder.config.bandwidth = ENCODER_BANDWIDTH
    odrv0.axis1.encoder.config.bandwidth = ENCODER_BANDWIDTH

    #odrv0.axis0.controller.config.vel_ramp_rate = 0.5
    #odrv0.axis1.controller.config.vel_ramp_rate = 0.5
    #odrv0.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
    #odrv0.axis1.controller.config.input_mode = INPUT_MODE_VEL_RAMP

def print_axis_control_values(axis):
    print("\tvel_gain",axis.controller.config.vel_gain)
    print("\tvel_integrator_gain",axis.controller.config.vel_integrator_gain)
    print("\tvel_integrator_limit",axis.controller.config.vel_integrator_limit)
    print("\tvel_limit",axis.controller.config.vel_limit)

def print_odrive_control_values(odrv):
    print("Axis 0")
    print_axis_control_values(odrv.axis0)
    print("Axis 1")
    print_axis_control_values(odrv.axis1)

def move(rps, duration, odrv):
    odrv.axis0.controller.input_vel = rps
    odrv.axis1.controller.input_vel = rps
    time.sleep(duration)
    odrv.axis0.controller.input_vel = 0
    odrv.axis1.controller.input_vel = 0

def ramp_hold(rps, duration, odrv):
    for i in range(100):
        odrv.axis0.controller.input_vel = rps * (i/100)
        odrv.axis1.controller.input_vel = rps * (i/100)
        time.sleep(0.01)

    time.sleep(duration)

    for i in range(100):
        odrv.axis0.controller.input_vel = rps * (1-(i/100))
        odrv.axis1.controller.input_vel = rps * (1-(i/100))
        time.sleep(0.01)


def turn(rps, duration, odrv):
    odrv.axis0.controller.input_vel = -rps
    odrv.axis1.controller.input_vel = rps
    time.sleep(duration)
    odrv.axis0.controller.input_vel = 0
    odrv.axis1.controller.input_vel = 0


def torque_control(nm, duration, odrv):

    odrv.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
    odrv.axis1.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL

    print("Setting torque to %fnm" %nm)
    odrv.axis0.controller.input_torque = nm
    odrv.axis1.controller.input_torque = nm

    time.sleep(duration)

    print("Setting torque to %fnm" %(-nm))
    odrv.axis0.controller.input_torque = -nm
    odrv.axis1.controller.input_torque = -nm

    time.sleep(duration)
    print("Setting torque to 0nm")
    odrv.axis0.controller.input_torque = 0
    odrv.axis1.controller.input_torque = 0

    time.sleep(10)

    odrv.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    odrv.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    

odrv0 = odrive.find_any(timeout=10)

if odrv0:
    print("Original Gains")
    print_odrive_control_values(odrv0)

    set_gains(odrv0)

    print("New Gains")
    print_odrive_control_values(odrv0)
    time.sleep(1)

    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    #torque_control(1, 10, odrv0)
    #torque_control(10, 2, odrv0)
    ramp_hold(1, 0, odrv0)
    # ramp_hold(-1,0,odrv0)
    # turn(1, 2, odrv0)
    # turn(-1, 2, odrv0)
    #for i in range(0):
    #    move(0.5, 3, odrv0) 
    #    time.sleep(2)

    #     #turn(0.2, 3, odrv0)
    #     #time.sleep(2)

    #    move(-0.5, 3, odrv0) 
    #    time.sleep(2)

    #     #turn(-0.2, 3, odrv0)
    time.sleep(1)
    odrv0.axis0.requested_state = AXIS_STATE_IDLE
    odrv0.axis1.requested_state = AXIS_STATE_IDLE

    odrv0.save_configuration()
    print("Saved config")
