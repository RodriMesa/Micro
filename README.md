# Development of embedded motor control
This repository presents the design of a master-slave system used on the "Discovery" development boards for motor control. 
This was conceived with a master board, which is responsible for coordinating the movements of the motors, receive movement instructions via USB, 
and perform, if necessary, the inverse kinematics calculations. On the other hand, each slave board will be in charge of carrying out the movement of two motors (per slave). 
two motors (per slave) with its own control. For the development of the project, the STM32Cube IDE software was used.
