# AudioMoth-USB-Microphone
Additional firmware for AudioMoth devices to implement a streaming USB microphone.

Compatible with the [AudioMoth USB Microphone App](https://github.com/OpenAcousticDevices/AudioMoth-USB-Microphone-App).

### Usage ####

Clone the contents of [AudioMoth-Project](https://github.com/OpenAcousticDevices/AudioMoth-Project).

Replace ```src/main.c``` with this ```main.c``` from this repository. Put all the remaining ```.c``` in the ```/src/``` folder and all the ```.h``` in the ```/inc/``` folder. 

The firmware implements two different USB configurations with different numbers of end-points in each. This is not supported within the standard Silicon Labs USB library, so open ```em_usbd.c``` inside ```emnusb/src/``` and comment out lines 433 to 437. This removes an internal validation of the number of end-points described in the USB configuration.

![alt text](https://github.com/OpenAcousticDevices/AudioMoth-USB-Microphone/blob/main/comment.png)

If building from the command line tools no further changes are necessary. However, if building using Simplicity Studio, open the project 'Properties' under the 'File' menu, navigate to the 'Linker' and 'Ordering' and make sure that the standard math library '-lm' is at the bottom of the list.

### License ###

Copyright 2017 [Open Acoustic Devices](http://www.openacousticdevices.info/).

[MIT license](http://www.openacousticdevices.info/license).
