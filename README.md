# Aero Open Hand Controller

The [Aero Open Hand](https://tetheria.github.io/aero-hand-open/) is a low cost, anthropomorphic robotic hand. Their control setup is a Xiao esp32s3 soldered to a control board that connects to the seven Feetech HLS3606M servo motors. The esp32s3 only connects to the serial pins of the control board, as the control board simply exposes a serial interface to the daisy chained servo motors which are TTL servo motors.

This project is a python codebase that can directly interface with the serial pins of the control board to control the servos, and offers an extendable GUI to control the hand with.

## Directly interfacing with the hand

The hardware facing side of this code uses PySerial to send commands to and receive data from the servos. It should have all of the same functionality that the pre-written code by the designers of the hand for the esp32 has, including:
 - Position and torque control modes of controlling the servo movements
    - Also applies limits (via software control) when controlling via torque
 - A thread that reads sensor data from the servos (position, velocity, temperature, current draw) whenever the serial bus isn't being used for control
 - Setting IDs of the servos (only used once, to configure each servo when they are initially connected to the control board)
 - Homing the hand, which reset it to a rest position
 - Trimming the servos, which means setting their neutral and maximum positions and remembering these settings.

This project also implements PID control when using torque control mode. The PID algorithm is be tuneable on the fly, and constants for each servo is saved to a file.

All original code that was written for the esp32 is in the arduino_code directory.
 - The firmware written to take commands from a host PC software is in the firmware.ino file. Note that these are NOT the commands that actually interface with the servos.
    - Homing.h and Homing.cpp contain the code for the homing the hand. This project does not necessarily need an abstraction that splits the Homing code away from the Hand interfacing code.
    - HandConfig.h allows the user to choose if the left or right hand is used. This project will take in left or right hand as a parameter in the constructor for the class that represents the physical hand.
- FTServo/ contains all library code used to interface with the control board over serial.

## Presenting a GUI

The user facing side of this code uses PyQt6 to present a modern GUI that has sliders to control the hand positions, change the control mode, and displays sensor readouts. It has controls to set servo IDs, home the hand, trim the servos and tune the PID constants for each arm.

The center of the GUI window is a graphical representation of the hand itself using QGraphicsScene. The graphics are simple, consisting only of lines that change their length (or in the case of the thumb, the length and angle) to represent the set position of the hand.