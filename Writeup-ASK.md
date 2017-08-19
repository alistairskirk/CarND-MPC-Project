# CarND-MPC-Project
Self-Driving Car Engineer Nanodegree Program
Alistair Kirk August 2017

---

## Writeup and Reflection
This project explores the creation of the Model Predictive Control (MPC) for use with the Term 2 Simulator. The goal is to automate steering of the test vehicle around the lake track loop, while dealing with prediction and latency.
The sensor input to our program provides the current steering angle, and throttle, vehicle velocity, bearing, and global x and y coordinates. 
Additional input includes a set of global coordinates representing the road centerline waypoints ahead of the vehicle which is used to calculate the Cross Track Error (CTE) and bearing error of the vehicle.

Videos of the functioning MPC project can be found on Youtube.
*[self driving car nanodegree program MPC Project: Shows the car successfully driving around the whole test track twice.](https://youtu.be/0eK-djsJhzw)

## The Model

The majority of the MPC theory and code style was taken from the Udacity course, with additional learning from resources listed below.

State

Actuators

Update Equations

The Proportional, Integral, and Derivative gains were manually tuned for this project:
Controller | Proportional | Integral | Derivative
--- | --- | --- | ---
Default Steering | 0.9 | 0.001 | 0.6 
Speed | 0.65 * 0.9 | 2.6 * 0.001 | 0.65 * 0.6

## Timestep Length and Elapsed Duration (N & dt)
Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

## Polynomial Fitting and MPC Preprocessing
A polynomial is fitted to waypoints.
If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

## Model Predictive Control with Latency
The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.


### Future considerations if one wanted to use a PID controller:
*Dynamic speed selection: go faster on straight-aways but slow down on the curves, possibly varying speed depending on DOT standard radius of curve. This would improve corner handling but would need a way to predict if you were in a tight turn or not
## References:
[webSocket Data Input](https://github.com/udacity/CarND-MPC-Project/blob/master/DATA.md)