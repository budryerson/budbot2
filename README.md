# budbot2
BudBot2 is an IR distance sensing, tracked robot.  The robot platform employs an nRF24L01 radio module to communicate with a joystrick controller. The robot radio is configured as the Primary Transmitter (PTX) and the joystick is configured as the Primary Receiver (PRX).  Data packets flow from the joystick as 'acknowledgement' to data packets sent from the robot. 

The joystick also exchanges serial data with a display anmd control application written in Processing on a PC.
