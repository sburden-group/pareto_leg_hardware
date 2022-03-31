# Troubleshooting

Here are some errors we have previously encountered.

* Before compiling and flashing custom firmware, check that the board versin in`tup.config` is set to the actual board we have in the lab. For reference, we have `CONFIG_BOARD_VERSION=v3.5-48v`
* Before flashing code, make sure the DIP switch is set to *DFU* on powerup.
* After flashing code, make sure that the DIP switch is set back to *RUN* on powerup.
* After flashing code, you need to reconfigure the robot with the bringup procedure.

## odrivetool Commands

Here are some useful commands that you can run with the odrive utility.

* `dump_errors(odrv0)` will dump the system state to the screen and display any errors in red.
* `odrv0.reboot()` will reboot the system

* `odrv0.axis0.encoder.shadow_count` read the raw AS5048A encoder count in ticks
* `odrv0.axis0.encoder.pos_estimate` read axis0 angle in rotations
* `odrv0.axis0.encoder.set_linear_count(0)` zero out the current angle of axis0.

