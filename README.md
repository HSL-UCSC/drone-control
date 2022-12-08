# Hybrid Systems Lab - ST Drone Interface
An interface for ST Microelectronics STEVAL-FCU001V1 drone.

#### Features
* XBOX Control
* Position setpoint generator
* Dynamic gain tuning (not implemented yet)
* Real-time data tx/rx
* Mechanism to switch between various controllers/estimators both on and off-board (not yet implemented)

## Base Station Setup
#### Optitrack Setup
1) Launch Motive, and follow the video tutorial found [here](https://drive.google.com/file/d/18uIZ4nmRBpClOAIvb6bXcjtSB3jgUkA6/view) in order to calibrate, create a rigid body, and stream pose data.
2) Once Motive has been setup to stream data to the base station, you will need to download the [NatNetSDK](https://optitrack.com/software/natnet-sdk/) from Optitrack's website. Our Matlab script can then load the library required to interface with Motive's data stream. When initializing MocapAPI class, provide a correct file path to the NatNetSDK. 

#### HC12 Setup
The HC12 is a radio transceiver with millisecond precise data transfer rates. This module comes ready with a pre-programmed microcontroller and wireless communication chip. All that's left to do, is interface with it through the simple UART protocol. By placing the HC12 into programming mode, we can send **AT** commands to adjust various setting such as the frequency to transmit/receive on, as well as the baudrate to operate UART on for data I/O. The datasheet for this module can be found [here](https://www.smart-prototyping.com/image/data/2020/09/102041%20HC-12%20Wireless%20Transceiver%20Module%20(SI4438,%20433MHz,%201km)/HC-12%20english%20datasheets.pdf), however, by following the instructions below, you can set it up for use with our drone.

![alt text](https://imgaz.staticbg.com/thumb/large/oaupload/banggood/images/2B/84/c2010326-344d-4513-98cd-879ea4f7ab16.jpg)

To begin, wire the hc12 as follows.. one time setup AT+B19200, battery supply, if 5V supply, use diode in series to drop voltage under 4.5V.

## Xbox Control
An Xbox controller was introduced to this project in order to 1) act as a form of manual override, and 2) enable the quadrotor to be controlled in unconventional ways such as by switching between positional setpoints, or even control position/velocity with the joystick itself. This extends the capabilities of the ST drone past the original intentions of just controlling attitude with a phone app. The current Xbox control configuration is provided in the figure below, however, thease features can be changed or built upon in order to meet your own experiment requirements.

![alt text](https://lh5.googleusercontent.com/ak9S9LqvmSyjND_QmrkH7fyYmUmcYyIqQMQegmAeDIY7XEuUXGje9xpXwXxIrt8zcgc=w2400)
.. Explain the various features

## Real Time Data Transfer
.. BLE callback and the scale factor
.. HC12 for attitude commands and data update requests

## Running Experiments
.. position_control_main.m

