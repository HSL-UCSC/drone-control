# Hybrid Systems Lab - ST Drone Interface
An interface for ST Microelectronics STEVAL-FCU001V1 drone.

#### Features
* XBOX Control
* Position setpoint generator
* Dynamic gain tuning (not implemented yet)
* Real-time data tx/rx
* Mechanism to switch between various controllers/estimators both on and off-board (not yet implemented)

## Base Station Setup
#### Optitrack
1) Launch Motive, and follow the video tutorial found [here](https://drive.google.com/file/d/18uIZ4nmRBpClOAIvb6bXcjtSB3jgUkA6/view) in order to calibrate, create a rigid body, and stream pose data.
2) Once Motive has been setup to stream data to the base station, you will need to download the [NatNetSDK](https://optitrack.com/software/natnet-sdk/) from Optitrack's website. Our Matlab script can then load the library required to interface with Motive's data stream. When initializing MocapAPI class, provide a correct file path to the NatNetSDK. 

#### HC12
<img align="right" src="https://imgaz.staticbg.com/thumb/large/oaupload/banggood/images/2B/84/c2010326-344d-4513-98cd-879ea4f7ab16.jpg" alt="drawing" width="200"/><!-- 
![alt text](https://imgaz.staticbg.com/thumb/large/oaupload/banggood/images/2B/84/c2010326-344d-4513-98cd-879ea4f7ab16.jpg) -->


The HC12 is a radio transceiver with millisecond precise data transfer rates. This module comes ready with a pre-programmed microcontroller and wireless communication chip. All that's left to do, is interface with it through the simple UART protocol. By placing the HC12 into programming mode, we can send **AT** commands to adjust various setting such as the frequency to transmit/receive on, as well as the baudrate to operate UART on for data I/O. Useful instructions for this module can be found on this [allaboutcircuits post](https://www.allaboutcircuits.com/projects/understanding-and-implementing-the-hc-12-wireless-transceiver-module/), and the orignal datasheet can be found [here](https://www.smart-prototyping.com/image/data/2020/09/102041%20HC-12%20Wireless%20Transceiver%20Module%20(SI4438,%20433MHz,%201km)/HC-12%20english%20datasheets.pdf). Follow the instructions below to set it up for use with the ST drone:


1) A [USB to TTL serial cable](https://www.amazon.com/HiLetgo-PL2303TA-RS232-Download-Cable/dp/B073R6XJND/ref=sr_1_2_sspa?crid=XXQ2EQ86HMBK&keywords=ttl+usb&qid=1670464205&sprefix=ttl+usb%2Caps%2C197&sr=8-2-spons&psc=1&spLa=ZW5jcnlwdGVkUXVhbGlmaWVyPUEzMEZFTDhOS1FCUkpFJmVuY3J5cHRlZElkPUEwMDc0ODgyMzIzMjdYNTFCVDU5MCZlbmNyeXB0ZWRBZElkPUEwMjkzNjg2MU5ORVJHNlJVUDY5QiZ3aWRnZXROYW1lPXNwX2F0ZiZhY3Rpb249Y2xpY2tSZWRpcmVjdCZkb05vdExvZ0NsaWNrPXRydWU=) should connect the HC12 to the base station. Plug the USB end into the computer, and connect the rest of the lines to the appropriate pins on the radio module. If using the 5V line of the cable to power the HC12, a diode should be placed in series so as to drop the voltage to below 4.5V as stated in the datasheet. Connect the SET pin to ground in order to enter programming mode. While in programming mode, the base station can send the HC12 **AT** commands to configure the module.
2) Set the baud rate to the default of 9600, and send "AT+DEFAULT". You should receive an "OK" acknowledgement which means the radio successfuly reset to default settings.
3) Now, change the operating baud rate to 19200 by sending "AT+B19200". Again, if successful, the base station should read back an "OK".
4) Finally, we need to make sure each pair of HC12's are listening to eachother. Set frequency channel with "AT+C001". If using multiple pairs of radios, stagger than channels at least 5 apart. Ie. when using a new pair of HC12's, use the command AT+C006.
5) Remove the SET pin from ground, and repeat steps 1 through 5 with the second HC12 transceiver. Only one HC12 will be connected to the base station, however, the other should be configured in the same way. 

Once two HC12 modules have been configured identically, connect one to the base station via the USB to TTL cable, and one to the ST drone via the UART pins. This radio will handle receiving attitude commands as well as data update requests from the base station. The flow of data using the HC12 is always such that data transmits from the base station, and is received on the ST drone.

Run the ... to verify communication works (MAKE MY OWN SCRIPT TO CHECK RADIO COMMS)

#### Bluetooth Low Energy (BLE)
While the HC12 handles consistently fast communication from the base station to the drone, a Bluetooth module is required to receive data being sent from the drone to the base station. 

.. can check the bluetooth mac adress via iphone app (OR I CAN MAKE MY OWN SCRIPT TO CHECK FOR THEM)

## Xbox Control
An Xbox controller was introduced to this project in order to 1) act as a form of manual override, and 2) enable the quadrotor to be controlled in unconventional ways such as by switching between positional setpoints, or even control position/velocity with the joystick itself. This extends the capabilities of the ST drone past the original intentions of just controlling attitude with a phone app. The current Xbox control configuration is provided in the figure below, however, thease features can be changed or built upon in order to meet your own experiment requirements.

![alt text](https://lh5.googleusercontent.com/ak9S9LqvmSyjND_QmrkH7fyYmUmcYyIqQMQegmAeDIY7XEuUXGje9xpXwXxIrt8zcgc=w2400)

## Real Time Data Transfer
.. BLE callback and the scale factor
.. HC12 for attitude commands and data update requests

## Running Experiments
.. position_control_main.m

