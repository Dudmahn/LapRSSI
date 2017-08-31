# LapRSSI - RF Based Lap Timing Device
LapRSSI is an 8-channel timing device for FPV racing that utilizes the 5.8 GHz analog video signal from the aircraft to perform lap detection. It is intended to be inexpensive, easy to build, and easy to maintain. It functions as a hardware timing device in combination with a PC-based lap timing software such as [LiveTime FPV](https://www.livetimescoring.com/).

LapRSSI consists of a single PJRC Teensy 3.2 microcontroller that interfaces with 8 Boscam RX5808 receivers, and it communicates with the lap timing software via UART serial link. The preferred serial communication device is the [SiK/3DR  Telemetry Radio v2](http://ardupilot.org/copter/docs/common-sik-telemetry-radio.html), which provides a long range 915 MHz wireless link to the timing gate. However, LapRSSI supports other TTL-level UART devices as well, such as the FT232 USB-Serial adapter, or the HC-06 Bluetooth Serial module.

Peak detection is performed on the RSSI signal output on each RX5808 receiver, and the system performs an auto-calibrate sequence at the beginning of each race, in order to determine appropriate signal levels for each transmitted video signal.

## License & Recognition
### Recognition
The main inspiration for this project came from the excellent [Delta 5 Race Timer](https://github.com/scottgchin/delta5_race_timer) by Scott Chin. Credit goes to Scott for conceiving the peak detection and auto-calibration algorithms, on which LapRSSI is based.

The [rx5808-pro-diversity](https://github.com/sheaivey/rx5808-pro-diversity) project by Shea Ivey provided insight into the capability to control the RX5808 module via SPI. Credit goes to Shea for the RX5808 frequency tables, which are incorporated into LapRSSI.

Credit goes to Mike Bailey a.k.a. [Hyper-Quad](https://github.com/Hyper-Quad) for the "LapRSSI" name, and also for first conceiving the idea to control 8 RX5808 modules with a single Teensy microcontroller.

Special thanks to  [LiveTime](https://www.livetimescoring.com/) for integrating support for LapRSSI into their lap timing software.

### License
GNU GPLv3

LapRSSI - RF Based Lap Timing Device
Copyright (C) 2017 Steve Lilly

LapRSSI is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

LapRSSI is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with LapRSSI.  If not, see <http://www.gnu.org/licenses/>.