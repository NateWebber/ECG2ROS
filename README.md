# ECG2ROS

## About

This is a small script that I wrote for collecting ECG data from a Polar H10 biosensor, and transferring it to a ROS Foxy node to be published. I wrote this as part of my Summer 2022 DAAD internship at Universität zu Lübeck in Lübeck, Germany.

This script is a little bit hacky due to some dubious multithreading use (a concept whose best practices are still very new to me), but it works. 

This script could also be used to get other sorts of data from the Polar H10. To do this the byterarray **POLAR_STREAM_CONFIG** would need to be modified per the [official specification](https://github.com/polarofficial/polar-ble-sdk/blob/master/technical_documentation/Polar_Measurement_Data_Specification.pdf). There would also have to be work done on appropriately converting the received hex data into meaningful values. That process for ECG data is borrowed (stolen) from [here](https://github.com/markspan/PolarBand2lsl/blob/main/Polar2LSL.py).


**NB**: If you want to use this script, don't forget to change the value of **POLAR_MAC_ADDRESS** to the MAC address of your individual sensor's MAC address. Also keep in mind that the BLE_GATT library I used for this script *does not* have a pairing function, so make sure you pair and connect your sensor to the machine you run this script on beforehand.

## Requirements
[ROS 2 Foxy](https://docs.ros.org/en/foxy/Installation.html)

[BLE_GATT](https://github.com/ukBaz/BLE_GATT)