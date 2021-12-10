import odrive
from odrive.enums import *
import time
from fibre.libfibre import ObjectLostError

VEL_GAIN = 10
VEL_INTEGRATOR_GAIN = 20
VEL_INTEGRATOR_LIMIT = 500
CURRENT_LIM = 20

def set_gains(odriv):
    odriv.axis0.controller.config.vel_gain = VEL_GAIN
    odriv.axis1.controller.config.vel_gain = VEL_GAIN

    odriv.axis0.controller.config.vel_integrator_gain = VEL_INTEGRATOR_GAIN
    odriv.axis1.controller.config.vel_integrator_gain = VEL_INTEGRATOR_GAIN

    odriv.axis0.controller.config.vel_integrator_limit = VEL_INTEGRATOR_LIMIT
    odriv.axis1.controller.config.vel_integrator_limit = VEL_INTEGRATOR_LIMIT

    odrv0.axis0.motor.config.current_lim = CURRENT_LIM
    odrv0.axis1.motor.config.current_lim = CURRENT_LIM

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
    

def turn(rps, duration, odrv):
    odrv.axis0.controller.input_vel = -rps
    odrv.axis1.controller.input_vel = rps
    time.sleep(duration)
    odrv.axis0.controller.input_vel = 0
    odrv.axis1.controller.input_vel = 0

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
    turn(0.2, 4, odrv0)
    # for i in range(5):
    #     move(0.5, 3, odrv0) 
    #     time.sleep(2)

    #     #turn(0.2, 3, odrv0)
    #     #time.sleep(2)

    #     move(-0.5, 3, odrv0) 
    #     time.sleep(2)

    #     #turn(-0.2, 3, odrv0)

    odrv0.axis0.requested_state = AXIS_STATE_IDLE
    odrv0.axis1.requested_state = AXIS_STATE_IDLE