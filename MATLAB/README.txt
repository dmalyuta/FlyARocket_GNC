Fly-A-Rocket GNC

***********************************************************************************************************
******************************************** LICENSE ******************************************************
***********************************************************************************************************

The MIT License (MIT)

Copyright (c) 2015 Danylo Malyuta

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

***********************************************************************************************************
********************************************* INTRO *******************************************************
***********************************************************************************************************

This project presents the cumulative effort of an Undergraduate student team to design, build and fly
the first ever recorded attempt to actively stabilize a model rocket with a (R)eaction (C)ontrol (S)system.
The end goal is to have 3 successful demonstration flights with the RCS control activated that show
the rocket being maintained in the upright vertical orientation during the few seconds of low velocity around
apogee. The test-bed vehicle, the (F)light (A)ttitude (L)inearly (CO)ntrolled (4)th iteration (FALCO-4) rocket,
is the own design of the student team and has been made specifically to carry the RCS system and supporting
avionics at a minimum size and cost.

Contributors:
		- Danylo Malyuta (GNC and avionics)
		- Gautier Rouaze (RCS mechanical design)
		- Xavier Collaud (Rocket airframe design)
		- Nikolay Mullin (Rocket design and systems architecture supervisor)
		- Mikael Gaspar (Launch systems and ground support)
		- Raimondo Pictet (CFD analysis)
		- John Maslov (GPS tracking)

Funding : eSpace Space Engineering Center EPFL.

A special thanks goes to the following people and companies for their support:

		- Jürg Thuring from Spacetec Rocketry, Tripoli and ARGOS (Advanced Rocket Group of Switzerland)

***********************************************************************************************************
************************************ HOW TO USE MATLAB FILES **********************************************
***********************************************************************************************************

1) Open and run main.m
	* NOTE : DO _N_O_T_ DELETE ./logs/noise_log.txt as it is needed both by main.m and kalman_filter_test.m
	  for noise parameter estimation needed in the simulation.

The code is heavily commented and some places (e.g. look in rocket_dynamics.m) offer you places where you
can change the code to see different effects and behaviour of the control system. Functions r2d.m and d2r.m do
radian to degree and degree to radian conversions respectively.

The file kalman_filter_test.m is a stand-alone file allowing you to see Kalman filter performance
and tweak its parameters (which, if you want to see in action in the main.m simulation you will have to copy-paste
into main.m in the lines regarding Kalman filter parameters). The Kalman filter itself is found in kalmanFnc.m and is
called from within kalman_filter_test.m.

The simulation takes into account inertial forces only (via the inertia matrix) and uses the Tait-Bryan angle
convention (yaw ψ (z)-pitch Ө (y)-roll Φ (z) Euler angle convention) with x pointing from center of mass to the nose of
the rocket:
			
					       / \  (Rocket nose)
			  		      /   \
					     /  x  \
					     : /|\ :
					     :  |  :  _
					     :	|  :  /| z
					     :	|  : /
					     :	|  :/
					     :	|  /
					     :	| /:
					     :	|/___________________>
					     :	   :		      y 
					     :	   :
					    /:	|  :\
					   / :	|  : \
					  /  :	|  :  \
					 /   :	|  :   \
					/____:__|__:____\
					      \___/
						    (Rocket tail)

Other forces, such as aerodynamic, are modeled as disturbances that you can apply and change yourself in
rocket_dynamics.m. Examples of distrubances in orientation (psidot, phidot and others) and in moment acting
on the rocket can be seen in rocket_dynamics.m.

The result_analyzer.m file reads the log files in ./logs/ directory and generates graphs in order to instantly
visually interpret what has been logged during the rocket flight (or during a ground test or whener the
flight software has been fully executed from start to finish). Note that these log files are generates in the
./logs/ folder on your Raspberry Pi when you run the compiled C program. You can then copy-paste these files from
your Raspberry Pi into the ./logs/ directory here; simply executing result_analyzer.m does the rest!

Finally, valve_curve.m is a small tool to generate the valve thrust characteristic. The RCS valves in our
rocket were chosen to be controlled in open loop due to space constraints preventing closing the loop to
be possible (as it would require adding additional sensors, etc.). Therefore the valves must receive receive
an individually-taylored PWM to achieve what thrust one needs; even at that, we will still have errors due to
hyteresis (going up not being the same as coming down), etc. Nevertheless, valve_curve.m is a small tool which
generates for you the C code that you must paste into the GNC C source code to have a thrust curve (i.e. flow
rate vs. PWM) that is representative of the "average" valve that is used for the RCS (Reaction Control System).
Note that the valves used in our case were Parker VSO Miniature Proportional Valve (see
http://ph.parker.com/us/12051/en/vso-miniature-proportional-valve) model 5 with 100 [psid] (6.89 [bar]) pressure
differential and 0.050 [in] (1.27 [mm]) orifice diameter. The curves found in ./images/curve.png are the first
figure on page 2. For your application, you may have to take a different curve!

***********************************************************************************************************
************************************** CONTACT FOR PROBLEMS ***********************************************
***********************************************************************************************************

In case of issues, contact the code's author Danylo Malyuta at danylo.malyuta@gmail.com
