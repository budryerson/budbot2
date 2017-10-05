# budbot2
BudBot2 is an Arduino sketch project for a semi-autonomous, distance sensing, tracked robot.

The robot communicates with a joystick contoller using nRF24L01+ devices.  The two radios are implemented using the Radiohead library and employ a single pipe for TX and RX. The robot platform acts as the primary transmitter (PTX) and receives command data back as acknowledgement (ACK) packets from the joystick, which functions as the primary receiver (PRX).

The joystick also exchanges serial data with a PC based display and control application created in the Processing3 IDE.
