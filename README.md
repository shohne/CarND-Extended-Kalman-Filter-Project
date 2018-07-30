## Extended Kalman Filter
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

This repo contains the written code to complete the project **Extended Kalman Filter** on Udacity Self-Driving Car Nanodegree. The goal is to predict vehicle position and velocity from radar and measures. 

Prerequisites
---
To run this project, it is necessary to have a c++ 11 compiler and cmake 3.5 (minimum).

Installation
---
First, clone the repository:
```
git clone https://github.com/shohne/CarND-Extended-Kalman-Filter-Project.git
```
Change current directory:
```
cd CarND-Extended-Kalman-Filter-Project
```
Execute cmake to creata a makefile:
```
cmake .
```
Create executable file:
```
make
```
If everything was OK, there should be a executable file ExtendedKF.

Running the Application
---
Firts, start the executable:
```
./ExtendedKF
```
Now, the program should show in console:
```
Listening to port 4567
```
It is waiting for tcp/ip connection on port to 4567 to receive update measures from the car simulator.

Finally, run the simulator. An example for execution:
[ekf.m4v](ekf.m4v)

Implementation Details
---
Please visit the [report.md](report.md) for more information about the algorithm pipeline.
