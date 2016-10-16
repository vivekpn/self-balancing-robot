# Self Balancing Robot

Platform Used: Arduino Uno

Sensors: Inertial Measurement Unit. (Gyroscope and accelerometer)

The gyroscope is very precise, but tend to drift over a period of time. The accelerometer is a bit unstable, but does not drift. Hence it can be relied on a longer window. To calculate the precise angle Kalman filtering was used.

A PID (proportional–integral–derivative) control is tuned to provide a correction to minimize the error.
