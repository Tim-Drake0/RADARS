# RADARS (Rapid Altitude Determination and Response System)
**RADARS.ino is the MATLAB code integrated into SparkyVT's HPR flight computer code.**

My senior design capstone project, RADARS, provides autonomous altitude control to solid motor rockets during
ascent. The system utilizes an air braking mechanism to adjust drag, thereby controlling
altitude. MATLAB and OpenRocket simulations predict flight trajectory and apogee while
Computational Fluid Dynamics (CFD) in ANSYS and OpenFOAM assess airbrake
effectiveness. The team is currently in the manufacturing phase, integrating the avionics and
airbrakes for 5 test flights to iteratively refine the control algorithm. This paper outlines the
design process, simulation methods, and flight test procedures, emphasizing the project's
objective of achieving precise altitude control.

Airbrake_Rocet_Sim.m is used to numerically simulate the flight of a high-powered rocket using airbrakes.
Some minor changes were made to HPR_Rocket_Flight_PC_V4_6.ino and Event_Logic.ino to fully integrate RADARS.
