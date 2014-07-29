Quadcopter Simulator
====================

What is it and why?
-----------
This is a multirotor physics simulator specifically designed to help
in development and research of PID controllers. It has been kept as
simple as possible to make the code accessible for others as well.

Almost all calculations are made in 3D, it should be relatively easy to
switch to a 3D gui at a later time. Currently a two
rotor craft is simulated, adding more rotors should be trivial.

Features
* (should have) accurate physics
* accessible code
* aircraft steerable
* reasonable drag model
* motors do not respond infinitely fast

Missing features
* Sensors (with jitter and max update frequency)
* propellers are not modelled.
* Yaw (motors generate no toque along their shaft)
* controller should not run every simulation step

Usage
-----
python quadsim.py

Then use arrowkeys to steer the aircraft and spacebar to reset motion

You will additionally need numpy and pygame packages
