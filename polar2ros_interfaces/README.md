# Interfaces
This is a supplementary ROS package that contains tools needed for the other scripts.

Specifically, at the moment the only content in this package is a custom "Accelerometer" message type, that uses three Int32's to transmit the data (in mg, 'milligravity') for the three acceleromter axes the Polar H10 possesses.

In order to use the accelerometer scripts you will need to make sure this package is built and sourced correctly.