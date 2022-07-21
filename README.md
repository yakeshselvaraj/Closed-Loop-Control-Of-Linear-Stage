# Closed-Loop-Control-Of-Linear-Stage
Implement closed loop control of a linear stage using an ultrasonic distance sensor, stepper driver, NEMA 23 stepper motor, and Arduino Uno. Upon startup, the carriage must home itself using a hardware endstop and maintain a setpoint distance between itself and another object, using the ultrasonic distance sensor for position feedback. Three additional sensors must be used as inputs to the system and the limits of the stage must be respected in all operation modes through the use of software end stops.