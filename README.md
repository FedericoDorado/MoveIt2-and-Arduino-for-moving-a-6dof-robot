### Using MoveIt2 and Arduino for moving a 6dof robot
Code used for move a 6dof robot sending the joint's values calculated by MoveIt 2 to an Arduino Uno through a serial port.

#### ROS 2 NODE

Considering the AccelStepper library for arduino and the relationship between the motor steps and the calculated angles, a ros 2 node is created to perform the calculations and send the results to the arduino in charge of moving all the stepper motors.

![](https://github.com/FedericoDorado/MoveIt2-and-Arduino-for-moving-a-6dof-robot/blob/main/Images/Nodo_arduino.png?raw=true)

Ussing the motion planning on MoveIt 2 the joint's values are calculated and then sended to the arduino trhoug serial port making the robot moving just as is wanted.

![](https://github.com/FedericoDorado/MoveIt2-and-Arduino-for-moving-a-6dof-robot/blob/main/Images/Nodo_arduino_test1.png.jpg.png?raw=true)

![](https://github.com/FedericoDorado/MoveIt2-and-Arduino-for-moving-a-6dof-robot/blob/main/Images/Nodo_arduino_test.png.jpg?raw=true)
