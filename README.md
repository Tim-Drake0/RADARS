# RADARS (Rapid Altitude Determination and Response System)
<img width="1415" alt="Screenshot 2024-03-13 181917" src="https://github.com/Tim-Drake0/RADARS/assets/84010200/2458e9f3-431c-48fd-9051-d0567ae52bd7">

**RADARS.ino is my code integrated into SparkyVT's HPR flight computer code: https://github.com/SparkyVT/HPR-Rocket-Flight-Computer**

My senior design capstone project, RADARS, provides autonomous altitude control to solid motor rockets during
ascent. The system utilizes an air braking mechanism to adjust drag, thereby controlling
altitude. MATLAB and OpenRocket simulations predict flight trajectory and apogee while
Computational Fluid Dynamics (CFD) in ANSYS and OpenFOAM assess airbrake
effectiveness. The team is currently in the manufacturing phase, integrating the avionics and
airbrakes for 5 test flights to iteratively refine the control algorithm. This paper outlines the
design process, simulation methods, and flight test procedures, emphasizing the project's
objective of achieving precise altitude control.

**Using/Running Code:**

The MATLAB code is used to size the airbrakes according to rocket specifications, such as body tube diameter, rocket length, target altitude, 
and burnout conditions (altitude and velocity). These values were taken from OpenRocket simulations. The program begins by running various simulations
at various CD values until the apogee is +/- 10ft to the target altitude. Drag values are tabulated for each time step of the simulation. The goal of the
airbrake algorithm is to match the drag values for the time step by changing how much the airbrakes are deflected based on how much drag is required. 

RADARS.ino is the equivalent of this code, but in C++ and integrated into SparkyVT's code.

The flight computer (custom-built by SparkyVT) utilizes a Teensy 3.5 as the main computer. The code is tested and uploaded using Arduino IDE to an 
SD card in the Teensy and the servo signal wires are connected to the flight computer. 

**Required Libraries**

TinyGPS++
SDfat and/or SD

**Expected Plots From MATLAB Code**
![image](https://github.com/Tim-Drake0/RADARS/assets/84010200/d04a6a09-95a8-4dbb-b7fb-888c4ab28789)

![image](https://github.com/Tim-Drake0/RADARS/assets/84010200/387df904-e6df-401f-ac19-1acaaeb9b3c4)

![image](https://github.com/Tim-Drake0/RADARS/assets/84010200/413ed0ee-fa65-4f91-82eb-dc7e23d5f88b)
