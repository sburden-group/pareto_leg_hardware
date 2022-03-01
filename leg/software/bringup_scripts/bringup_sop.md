# Bringup

Run the **odrivetool** utility.

Power up the PSU and apply 24V to the board.

Check that there are no errors with `dump_errors(odrv0)`.


If there are no errors, close the odrive utility.

run `bringup.py` to upload the Dual TMotor U8 motor configuration for each axis.

Reopen the odrive utility. Check that there no errrors with `dump_errors(odrv0)`.

Run a motor0 calibration with:
````
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
````
When calibration finishes, check that there no errrors with `dump_errors(odrv0)`.

Run a motor1 calibration with:
````
odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
````
When calibration finishes, check that there no errrors with `dump_errors(odrv0)`.

If there are no errors, tell the motor to show up pre-calibrated by setting the following:

````
odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis0.encoder.config.pre_calibrated = True
odrv0.axis1.motor.config.pre_calibrated = True
odrv0.axis0.encoder.config.pre_calibrated = True
````

Close the odrive utility.

