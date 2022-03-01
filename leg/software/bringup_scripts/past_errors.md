# Past Errors

Here are some errors we have previously encountered.

* Before compiling and flashing custom firmware, check that the board versin in`tup.config` is set to the actual board we have in the lab. For reference, we have `CONFIG_BOARD_VERSION=v3.5-48v`
* Before flashing code, make sure the DIP switch is set to *DFU* on powerup.
* After flashing code, make sure that the DIP switch is set back to *RUN* on powerup.
* After flashing code, you need to reconfigure the robot with the bringup procedure.
