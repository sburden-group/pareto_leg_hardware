# Debugging

Here are some useful commands that you can run with the odrive utility.

* `dump_errors(odrv0)` will dump the system state to the screen and display any errors in red.
* `odrv0.reboot()` will reboot the system

* `odrv0.axis0.encoder.shadow_count` read the raw AS5048A encoder count in ticks
* `odrv0.axis0.encoder.pos_estimate` read axis0 angle in rotations
* `odrv0.axis0.encoder.set_linear_count(0)` zero out the current angle of axis0.
