Manufactured version of this project will be made available when the project has reached an appropriate maturity.

#Introduction
The main goal of this project is to add a 5.8ghz video RSSI hardware solution to [MultiGP Lapsync](https://www.facebook.com/groups/FPVRaceTimer/). 

This project is intended to supply a DIY hardware design and associated software. 

## Table of Contents
1. [Features](#features)
2. [Usage](#usage)
3. [Hardware](#hardware)
4. [Software](#software)
5. [Contributing](#contributing)
6. [License](#license)


##Features
The following are the planned features to be incorperated into MultiGP Lapsync for use with LapRSSI. Incorperation is at the discression of the Lapsync developer. The project is still in the beta phase and so most of the features below still need to be incorperated.

- **Pilot Count** - The number of pilots for each heat will set the number of receivers used. This will be hardware limited by how many receivers are installed.
- **Pilot Frequency** - The there will be a video frequency field for each pilot in Lapsync that will then update the frequency of each receiver.
- **RSSI Trigger** - A RSSI trivver value will need to be set in Lapsync for each pilot. This can be changed to match the pilots equipment.
- **RSSI Calibrate** - A button for each pilot will allow an automatic callibration of the RSSI trigger.
- **Frequency Scanner** - A feature will be added to Lapsync that will show a graph of the RSSI value for all 40 frequency channels. 

##Usage

- **Hardware** - The assembled hardware will connect to a computer via a USB cable. A USB-CAT5-USB setup can extende the distance from the computer to the hardware assembly. A USB wireless connection can also be used.
  The receivers will require a ~12v power supply. A 3x Li-po battery works great.
- **Software** - Once the software is uploaded to the hardware it will run automatically.


##Software Installation
- ** -Download [Arduino version 1.6.12](https://www.arduino.cc/en/Main/OldSoftwareReleases#previous)
- ** -Go to the [PJRC Teensyduino installation](http://www.pjrc.com/teensy/td_download.html) page. Download Teensyduino and follow the instructions. This will setup the Arduino software to work with the Teensy.
- ** -Download the latest release of LapRSSI.ino and open it in arduino.
- ** -Click the UPLOAD button. The code will compile and upload to the Teensy.
- ** -Wire the receiver(s) to the teensy and then proceed with the initial setup.


##Initial Setup
When powering on for the first time it is best to calibrate your RSSI modules. No two modules have the same RSSI min and max readings. To calibrate follow these steps below. You can repeat this process as many times as needed.

1. Connect the hardware assembly to your computer with a USB cable.
2. Plug your ~12v power supply into the hardware assembly to power the receivers.
3. Open Device manager on your computer and see what COM port has been assigned.
4. Open Lapsync.
5. Go to Edit>Options and enter in the COM port value.
6. Click on the open COM port button or go to File>Open COM Port.

##Hardware
This project is centered around the [Tensy 3.2](https://www.pjrc.com/store/teensy32.html). It is a faster version of an Arduino.

The project also is designed to use the [rx5808 5.8ghz receiver module](https://www.foxtechfpv.com/product/5.8G%20modules/rx5808/RX5808-Spec-V1.pdf) which can be found at a number of online stores. The number of modules can be varied from 1 to 8.

##Software
The software uses SPI to allow the 8 channel receiver to be able change to any of the 40 channels that are standard usage. It reads the RSSI value from each receiver and when it detects a value higher than the associated threshold it will send a string to Lapsync triggering a lap count.

A heartbeat is sent to Lapsync on 1 second intervals so the software knows it is in constant communication with the hardware.

The lap trigger is stored until the next heartbeat and then they are sent one at a time to Lapsync in the order received in the case that triggers are received from multiple receivers.

##Contributing
Contributors are welcome. We ask that you submit your ideas to the team for review.


##License & Recognition
####Recognition
To be added.

####License
The MIT License (MIT)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
