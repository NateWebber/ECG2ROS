# Accelerometer Scripts
These scripts interact with the accelerometer data of the Polar H10, rather than the ECG data

I have documented them to explain the methods by which acceleromter data is gathered and interpreted, as well as provided links to relevant source material that helped me figure this stuff out.

The scripts use a custom "Accelerometer" ROS message type, that consists of three Int32's for the x, y, and z axes. This message type is included in the polar2ros_interfaces packages that is also in this repository. It is the only additional dependency for these scripts, otherwise the dependencies are indentical to the ones for the ECG scripts. 