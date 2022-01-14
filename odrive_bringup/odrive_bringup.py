#!/usr/bin/env python3
"""Bringup script for provisioning an odrive."""

# Hardware Setup:
#   * TMotor U8 (?)
#   * AS5048a absolute encoder installed
#   * Power resistor installed (but not configured currently).


import odrive
import odrive.enums as en
import fibre.libfibre # for exceptions
import time

# Find ODrive:
print("finding an odrive...")
odrv0 = odrive.find_any()
print("found an odrive!")

# Apply configuration settings:
odrv0.axis0.motor.config.pole_pairs = 11 # 24N22P
odrv0.axis0.motor.config.torque_constant = 1.0 # Set torque units to Amps.
odrv0.axis0.motor.config.current_lim = 10.0
odrv0.axis0.motor.config.motor_type = en.MOTOR_TYPE_HIGH_CURRENT
odrv0.axis0.controller.config.vel_limit = 5 # RPS

odrv0.axis0.motor.config.pre_calibrated = False
odrv0.axis0.encoder.config.pre_calibrated = False
odrv0.axis0.encoder.config.use_index = True

# Increase default calibration settings for this particular gimbal motor.
odrv0.axis0.motor.config.resistance_calib_max_voltage = 5.0
odrv0.axis0.motor.config.calibration_current = 1.0

# TODO: if we enable flyback resistor, do we need to increase this value?
odrv0.config.dc_max_negative_current = -0.1

# Calibrate:
print("starting calibration...")
odrv0.axis0.requested_state = en.AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while odrv0.axis0.current_state != en.AXIS_STATE_IDLE:
    time.sleep(0.1)

# Tell calibration to persist beyond powerdown.
odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis0.encoder.config.pre_calibrated = True

# odrive.axis0.requested_state must be AXIS_STATE_IDLE first.
try:
    odrv0.save_configuration()
# Odrive will reboot after saving.
except fibre.libfibre.ObjectLostError:
    pass

odrv0 = odrive.find_any()

print("Homing...")
odrv0.axis0.requested_state = en.AXIS_STATE_ENCODER_INDEX_SEARCH
while odrv0.axis0.current_state != en.AXIS_STATE_IDLE:
    time.sleep(0.1)
print("Homed.")


## Setup Torque Control
#odrv0.axis0.controller.config.control_mode = en.CONTROL_MODE_TORQUE_CONTROL
#odrv0.axis0.controller.config.input_mode = en.INPUT_MODE_PASSTHROUGH
#odrv0.axis0.controller.config.enable_torque_mode_vel_limit = False
#odrv0.axis0.requested_state = en.AXIS_STATE_CLOSED_LOOP_CONTROL
#
## Try it:
#odrv0.axis0.controller.input_torque = 0.5
#for i in range(15):
#    time.sleep(1);
#    print(f"{odrv0.axis0.motor.current_control.Iq_measured:.3f}")
#odrv0.axis0.controller.input_torque = 0
#odrv0.axis0.requested_state = en.AXIS_STATE_IDLE
#print("done!")
