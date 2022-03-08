# Setup

## Required Hardware:
* breadboard
* [1bitsy](https://1bitsquared.com/products/1bitsy)
* [Black Magic Probe](https://www.adafruit.com/product/3839) for flashing
* Pololu In-Line Current Sensor, [+/- 40A](https://www.pololu.com/product/4033)

## Firmware
* Install gcc-arm-embedded according to the [libopencm3 guide](https://github.com/libopencm3/libopencm3#prerequisites)
* Pull the submodule repository in with: `git submodule init` and then `git submodule update`.
* cd into the **libopencm3** directory and build the code with `make`
* with libopencm3 built, you can now invoke the Makefile for this project in this directory with `make`


## Linux Setup
You will need to add the udev rules for (1) the black magic probe, (2) the 1bitsy custom usb device and reload rules.

Install gcc-arm-embedded according to the [libopencm3 guide](https://github.com/libopencm3/libopencm3#prerequisites).

Pull the submodule repository in with: `git submodule init` and then `git submodule update`.

cd into the libopencm3 directory and build the code with `make`.

### Compiling Firmware
With libopencm3 directory, you can now invoke the Makefile for this project in this directory with `make`.

### Flashing
With the code compiled, you should be able to flash it with `make bmp-flash`.
