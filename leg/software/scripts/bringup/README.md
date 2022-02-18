## Computer Environment Setup

1. Create a virtual environment - make sure the python package is not newer than 3.8.6
1. Enter that new environment
1. `pip install --upgrade odrive`

- manual setup: 'odrivetool'


## troubleshoot (windows)
- If getting error of "[USB] Could not open USB device: -5": download Zadig and replace the driver to libusb (v1.2.6.0 for me). (https://docs.odriverobotics.com/troubleshooting.html#usb-connectivity-issues)


## Hardware Troubleshooting
- Encoders need to be rebooted if you change the config, so everything needs to be power-cycled.
- Wait for the LED to burn out before turning power back on.


## Reflashing:
- in `tup.config`, board should be set to `ONFIG_BOARD_VERSION=v3.5-48V`
