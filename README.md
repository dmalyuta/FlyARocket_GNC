# Fly-A-Rocket flight software

## The team

* Danylo Malyuta (GNC and avionics)
* Gautier Rouaze (RCS mechanical design)
* Mikael Gaspar (Launch systems and ground support)
* Nikolay Mullin (Rocket design and systems architecture supervisor)
* Raimondo Pictet (CFD analysis)
* Xavier Collaud (Rocket airframe design)

This is the Bachelor project for the above team, done at the EPFL Space Engineering Center (eSpace), and so we thank in particular the following organizations for the support they have shown in making this project a reality.

<p align="center">
<img src="https://github.com/DanyloMalyuta/FlyARocket_GNC/blob/master/doc/images/EPFL.png" width="300" alt="EPFL (Swiss Federal Institute of Technology in Lausanne)" />
</p>

<p align="center">
<img src="https://github.com/DanyloMalyuta/FlyARocket_GNC/blob/master/doc/images/eSpace.jpg" width="300" alt="eSpace Space Engineering Center" />
</p>

## Project aim

### Abstract

This project has the aim of making the first documented attempt to stabilize a model rocket via an RCS - i.e. a Reaction Control System. For this purpose, the **F**light **A**ttitude **L**inearly **CO**controlled 4(th) iteration (FALCO-4) rocket was designed by our team with the objective of being the smallest vehicle capable of flight-testing the RCS system.

### Details

This is the first in what is planned to be a series of 3 projects to create a fully reusable model rocket via powered vertical landing (such as seen in this [SpaceX test](https://www.youtube.com/watch?v=9ZDkItO-0a4)). The aim would be to provide team members at each step with invaluable experience in the design of rockets with the reusability constraint. The aim of reusing the model rocket with an actively controlled landing rather than a parachute is to serve a training purpose in preparation for work with real rockets as the presented challenges are tougher than with simple parachute deployment currently used on almost every amateur model rocket.

This project (Fly-A-Rocket) : a Bachelor project to develop and successfully test rocket FALCO-4 with an RCS control system with the aim of making the rocket maintain a perfectly vertical attitude around apogee (at low velocities).

Next project : a Master project to design and build a large (2-3 [m] long) rocket carrying an evolved version of the RCS from FALCO-4 as well as a fin-deflection based control systems. The fins will be used to control the rocket during high-speed ascent and the RCS will be turned on near apogee when velocity goes to zero. The rocket will carry a large main and a series of small motors (e.g. [D12-0](https://www.apogeerockets.com/Rocket_Motors/Estes_Motors/24mm_Motors/Estes_Motors_D12-0?zenid=7adf7364902fc1b2cc1028012ae3d976)). The main engine will be used for ascent and the small motors will fire one after another at apogee; their impulse will keep the rocket "bouncing" at the apogee altitude while the RCS maintains the rocket's vertical position. Therefore the outcome of this project would be a proof-of-concept for the feasibility of stably mainting a model rocket's altitude via an array of small motors.

Final project : a PhD thesis to design and build the first ever reusable model rocket via powered descent and vertical landing. The rocket will be an evolution of the Master project rocket and will incorporate features like landing legs. The aim will be to bring the rocket to a soft landing at a precise location. In this case, the same control systems (fins and RCS) will be used; the array of small motors will thus be strategically ignited in the final moments of descent to slow down the rocket before landing.



## Source

This repository contains the full source-code and support programs for the flight software and avionics for the Fly-A-Rocket project. Everything required software-side to fly the FALCO-4 rocket can therefore be found here. [Here](https://github.com/DanyloMalyuta/FlyARocket_GNC/blob/master/doc/avionics/veroboard_layouts.svg) you can find the actual layout of the electronics hardware used on the rocket.
