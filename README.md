# AudioMoth-USB-Microphone
Additional firmware for AudioMoth devices to implement a streaming USB microphone.

For more details, visit [AudioMoth USB Microphone](https://www.openacousticdevices.info/usb-microphone).

Compatible with the [AudioMoth USB Microphone App](https://github.com/OpenAcousticDevices/AudioMoth-USB-Microphone-App).

### Usage ####

Clone the contents of [AudioMoth-Project](https://github.com/OpenAcousticDevices/AudioMoth-Project).

Replace the ```src/main.c``` from AudioMoth-Project with the ```src/main.c``` from this repository. Put all the remaining ```src/*.c``` files and all the ```src/*.h``` files from this repository into the ```/src/``` and ```/inc/``` folders of the AudioMoth-Project repository. 

The firmware implements two different USB configurations with a different number of end-points in each. This is not supported within the standard Silicon Labs USB library, so open ```em_usbd.c``` inside ```emnusb/src/``` and comment out lines 433 to 437. This removes an internal validation of the number of end-points described in the USB configuration.

![alt text](https://github.com/OpenAcousticDevices/AudioMoth-USB-Microphone/blob/main/comment.png)

If building from the command line tools no further changes are necessary. However, if building using Simplicity Studio, open the project 'Properties' under the 'File' menu, navigate to the 'Linker' and 'Ordering' and make sure that the standard math library '-lm' is at the bottom of the list.

### Documentation ####

See the [Wiki](https://github.com/OpenAcousticDevices/AudioMoth-Firmware-Basic/wiki/AudioMoth) for detailed description of the example code.

### License ###

Copyright 2021 [Open Acoustic Devices](http://www.openacousticdevices.info/).

[MIT license](http://www.openacousticdevices.info/license).
