# **Extended Kalman Filter**

The goals / steps of this project are the following:
* Predict position and velocity to a vehicle;
* Receive radar and lidar information from vehicle and update its state using a Extended Kalman Filter.

### Inplementation Details

There are 3 main steps in this code:
1. Initialization;
2. Prediction;
3. Update state;

All these 3 steps are in file **src/FusionEKF.cpp**, *Initialization* is on class *FusionEKF* constructor and begin of FusionEKF::ProcessMeasurement method. The *Prediction** and *Update* are in **FusionEKF::ProcessMeasurement**.

There is a auxiliary class *Tools* that computes de (root mean square error)[https://en.wikipedia.org/wiki/Root-mean-square_deviation].

