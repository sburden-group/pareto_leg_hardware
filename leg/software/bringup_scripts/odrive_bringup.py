#!/usr/bin/env python3
"""Bringup script for provisioning an odrive."""

# Hardware Setup:
#   * TMotor U8 KV100
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

# Apply general settings
odrv0.config.enable_brake_resistor = True
odrv0.config.brake_resistance = 2.0 # [Ohms]
# Current Settings:
odrv0.config.max_regen_current = 1.0 # [Amps] max current before the brake resistor kicks in.
odrv0.config.dc_max_negative_current = -10.0 # [Amps] max negative current after saturating brake resistor(?)

# Apply motor 0 configuration settings:
odrv0.axis0.motor.config.pole_pairs = 21 # 21 pole pairs ??
odrv0.axis0.motor.config.torque_constant = 1.0 # Set torque units to Amps. #leave 1 for now
odrv0.axis0.motor.config.motor_type = en.MOTOR_TYPE_HIGH_CURRENT
odrv0.axis0.controller.config.vel_limit = 500 # RPS
odrv0.axis0.motor.config.current_lim = 30.0
odrv0.axis0.motor.config.current_lim_margin = 8
odrv0.axis0.motor.config.requested_current_range = 50.0
odrv0.axis0.controller.config.pos_gain = 20.0
odrv0.axis0.controller.config.vel_gain = 0.8
odrv0.axis0.controller.config.vel_integrator_gain = 4.0
odrv0.axis0.motor.config.current_control_bandwidth = 1000.0
odrv0.axis0.controller.config.enable_torque_mode_vel_limit = False # not needed for position mode.
odrv0.axis0.controller.config.control_mode = en.CONTROL_MODE_TORQUE_CONTROL
odrv0.axis0.controller.config.input_mode = en.INPUT_MODE_PASSTHROUGH
odrv0.axis0.controller.config.enable_vel_limit = False
odrv0.axis0.controller.config.enable_overspeed_error = False
odrv0.axis0.controller.config.anticogging.anticogging_enabled = False

odrv0.axis0.motor.config.pre_calibrated = False
odrv0.axis0.encoder.config.pre_calibrated = False
odrv0.axis0.encoder.config.use_index = False # no dedicated index signal



# Apply motor 1 configuration settings (same as motor0):
odrv0.axis1.motor.config.pole_pairs = 21 # 21 pole pairs ??
odrv0.axis1.motor.config.torque_constant = 1.0 # Set torque units to Amps. #leave 1 for now
odrv0.axis1.motor.config.motor_type = en.MOTOR_TYPE_HIGH_CURRENT
odrv0.axis1.controller.config.vel_limit = 500 # RPS
odrv0.axis1.motor.config.current_lim = 30.0
odrv0.axis1.motor.config.current_lim_margin = 8
odrv0.axis1.motor.config.requested_current_range = 50
odrv0.axis1.controller.config.pos_gain = 20.0
odrv0.axis1.controller.config.vel_gain = 0.8
odrv0.axis1.controller.config.vel_integrator_gain = 4.0
odrv0.axis1.motor.config.current_control_bandwidth = 1000.0
odrv0.axis1.controller.config.enable_torque_mode_vel_limit = False # not needed for position mode
odrv0.axis1.controller.config.control_mode = en.CONTROL_MODE_TORQUE_CONTROL
odrv0.axis1.controller.config.input_mode = en.INPUT_MODE_PASSTHROUGH
odrv0.axis1.controller.config.enable_vel_limit = False
odrv0.axis1.controller.config.enable_overspeed_error = False
odrv0.axis1.controller.config.anticogging.anticogging_enabled = False

odrv0.axis1.motor.config.pre_calibrated = False
odrv0.axis1.encoder.config.pre_calibrated = False
odrv0.axis1.encoder.config.use_index = False # no dedicated index signal


# AS5048a motor0 configuration:
odrv0.config.gpio3_mode = en.GPIO_MODE_DIGITAL
odrv0.axis0.encoder.config.abs_spi_cs_gpio_pin = 3 # CS on GPIO pin 3.
odrv0.axis0.encoder.config.mode = en.ENCODER_MODE_SPI_ABS_AMS
odrv0.axis0.encoder.config.cpr = 2**14

# Increase default calibration settings for this particular gimbal motor.
odrv0.axis0.motor.config.resistance_calib_max_voltage = 12.0
odrv0.axis0.motor.config.calibration_current = 2.5 # current used to do the measurement

# AS5048a motor1 configuration:
odrv0.config.gpio4_mode = en.GPIO_MODE_DIGITAL
odrv0.axis1.encoder.config.abs_spi_cs_gpio_pin = 4 # CS on GPIO pin 4.
odrv0.axis1.encoder.config.mode = en.ENCODER_MODE_SPI_ABS_AMS
odrv0.axis1.encoder.config.cpr = 2**14

# Increase default calibration settings for this particular gimbal motor.
odrv0.axis0.motor.config.resistance_calib_max_voltage = 12.0
odrv0.axis0.motor.config.calibration_current = 2.5 # current used to do the measurement

# Increase default calibration settings for this particular gimbal motor.
odrv0.axis1.motor.config.resistance_calib_max_voltage = 12.0
odrv0.axis1.motor.config.calibration_current = 2.5 # current used to do the measurement


# TODO: get Kp, Ki, Kd and store them here.


# Calibrate:
#print("starting motor0 calibration...")
#odrv0.axis0.requested_state = en.AXIS_STATE_FULL_CALIBRATION_SEQUENCE # do the calibration
#while odrv0.axis0.current_state != en.AXIS_STATE_IDLE: # wait until idle again
#    time.sleep(0.1)
#print("starting motor1 calibration...")
#odrv0.axis1.requested_state = en.AXIS_STATE_FULL_CALIBRATION_SEQUENCE # do the calibration
#while odrv0.axis0.current_state != en.AXIS_STATE_IDLE: # wait until idle again
#    time.sleep(0.1)

# Tell calibration to persist beyond powerdown.
#odrv0.axis0.motor.config.pre_calibrated = True
#odrv0.axis0.encoder.config.pre_calibrated = True
#odrv0.axis1.motor.config.pre_calibrated = True
#odrv0.axis1.encoder.config.pre_calibrated = True

# odrive.axis0.requested_state must be AXIS_STATE_IDLE first.
# odrive.axis1.requested_state must also be AXIS_STATE_IDLE first.
try:
    odrv0.save_configuration()
# Odrive will reboot after saving.
except fibre.libfibre.ObjectLostError:
    pass

odrv0 = odrive.find_any() # find USB again
print("Done configuring!")

# print("Homing...")
# odrv0.axis0.requested_state = en.AXIS_STATE_ENCODER_INDEX_SEARCH
# while odrv0.axis0.current_state != en.AXIS_STATE_IDLE:
#     time.sleep(0.1)
# print("Homed.")


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

# Setup Position Control
# odrv0.axis0.controller.config.control_mode = en.CONTROL_MODE_POSITION_CONTROL 
# odrv0.axis0.controller.config.input_mode = en.INPUT_MODE_PASSTHROUGH
# odrv0.axis0.requested_state = en.AXIS_STATE_CLOSED_LOOP_CONTROL 

## Try it:
# odrv0.axis0.controller.input_pos = 0.0


##TODO
# how to enable power resistor
# make sure the calibration works, won't tell you if there's bug
# interative way mode to talk to Odrive 


# odrv0.axis0.controller.config.vel_gain = 0.8
# odrv0.axis0.controller.config.pos_gain = 20 # from 600
# odrv0.axis0.controller.config.vel_integrator_gain = 0.5*10*0.8



# TODO: provide the following options:
# * calibrate motor 0
# * calibrate motor 1
# * get stored configuration from odrive
# * load configuration <config_file.json> from pc
#if __name__ == "__main__":
#    pass
