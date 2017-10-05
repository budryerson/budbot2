# budbot2
BudBot2 is an Arduino sketch project for a semi-autonomous, distance sensing, tracked robot.

The robot uses a pair of nRF24L01+ devices to communicate with a joystick contoller.  The radios are implemented using the Radiohead library. They employ a single pipe for TX and RX. The robot platform acts as the primary transmitter (PTX) and receives command data back as acknowledgement (ACK) packets from the joystick, which functions as the primary receiver (PRX).

The joystick also exchanges serial data with a PC based Graphical User Interface (GUI) application created in the Processing3 IDE.  The PC application displays joystick data as well as status data and navigation information from the robot platform. The application also accepts graphical button clicks, whcih are integrated into the joystick command data and sent to the robot platform.

Robot motor speed is managed by feeding wheel encoder data into a Proportional Integral Derivative (PID) controller routine.
