# **MPC Control**

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/896/view)individually and describe how I addressed each point in my implementation.

---
### Implementation

#### The Model

* Student describes their model in detail. This includes the state, actuators and update equations.

The related code in `MPC.cpp` is as follows.

```
      fg[1 + x_start + t]    = x1 -    (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t]    = y1 -    (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t]  = psi1 -  (psi0 + v0 * delta0 / Lf * dt);
      fg[1 + v_start + t]    = v1 -    (v0 + a0 * dt);
```
According to the lecture, the state represent the current position, orientation, speed. The state is depend on only the previous step by Markov assumption.
`x`, `y` are position, and `psi` is orientation angle. `v` is velocity.

Actuator-related variables are `delta` steering angle. and `a` acceleration.

```
      fg[1 + cte_start + t]  = cte1 -  ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
```
And we need additional 2 elements for representing errors. The first One is cte, cross track error expressing the error between the center of the road and the vehicle's position. The second one express orientation error, epsi. These two elements are affected by the status of the elements described above and actuators.


```
    // The part of the cost based on the reference state.
    for (int t = 0; t < N; t++) {
      fg[0] += CppAD::pow(vars[cte_start + t], 2);
      fg[0] += CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; t++) {
      fg[0] += CppAD::pow(vars[delta_start + t], 2);
      fg[0] += CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; t++) {
      fg[0] += 500* CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
```
We need additional 'constraint model` to mimic the real world driver's behaviors.
Basically we need to minimize whole types of errors. cte, epsi. And we need to put model some incentive for the vehicle to go foward.
I use additional two cost functions. One for minimizing the use of actuators. Another one is for minimizing the value gap between sequential actuations.
These 2 constraints make the vehicle's movement stable.

#### Timestep Length and Elapsed Duration (N & dt)

* Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

```
 size_t N = 10;
 double dt = 0.1;
```
In my opinion T is the first parameter to fix, `T = N x dt` should be moderate time length. Too long T can make predicted path exceed out of the x range of reference path. It can corrupt all calcuration.
After several tests I fixed N = 10, dt = 0.1. `dt = 0.1` is enough short for catching the vehicle's movement.  and shorter N means fewer paramters and fast calculation of optimization process.

#### Polynomial Fitting and MPC Preprocessing

* A polynomial is fitted to waypoints.

* If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

I used third order polynomial. During the very earlier test, I use linear fitting for clearity and simplicity.
The second order polynomial sufficiently represent a curve line in one direction. So test result in simulation system is not bad.
But I think that more higher order than 2nd is needed for expressing complicated curve lines. I think third order polynomial is suitable for roads in the real world.

Before the polynomial fitting for reference path, we need the transformation from global map coordinates to local vehicle coordinates as a preprocess. Without this transformation, some input waypoints make extreme results. For example polynomial fitting to `y=constant` can cause wrong results.

#### Model Predictive Control with Latency

* The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

If we know the whole process has latency, we need to simulate the vehicle movement within the latency time. We already have the model as described before.
The following code in `main.cpp` is the implementation.
```
          // calculate new start point through simulation considering latency
          double latency = 0.1; // 100ms
          px = px + v*cos(psi)*latency;
          py = py + v*sin(psi)*latency;
```
### Simulation

* The vehicle must successfully drive a lap around the track.

The following video is the final result. [final_result.mp4](./mpc.mp4)

