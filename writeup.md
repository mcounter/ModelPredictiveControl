## Model Predictive Control (MPC)

Implements Model Predictive Control to drive the car simulator around the track.

![Model Predictive Control](./images/main.jpg)

#### 1. Model
Model is a base thing for Model Predictive Control algorithm. Complex dynamic models depends on numerous vehicle physical parameters which unknown for car simulator. Therefore I selected more simple Kinematic model, which depends mostly on parameters which could be observable or estimated.

##### 1.1 State vector

In model state vector I included vehicle position, angle, velocity, steering angle and acceleration: S = (x, y, ψ, v, d, a). Using global coordinates lead to a undesired complexity with mathematical equations. It happen because rotation angle could be ambiguous and some trigonometric functions (like tangent) are not continuous function which cause problems with derivatives and using optimization algorithms. Moving from global system of coordinates to vehicle local system resolves these problems, because initial state become S(0) = (0, 0, 0, v, d, a).

##### 1.2 State update equations

State update equations for Kinematic model:

* x(t + 1) = x(t) + v(t) \* cos(ψ) \* dt
* y(t + 1) = y(t) + v(t) \* sin(ψ) \* dt
* ψ(t + 1) = ψ(t) + v(t) \* tan(d(t - wlag)) \* dt / Lf
* v(t + 1) = v(t) + a(t - hlag) \* dt

Where:
* dt - time difference between adjacent steps of prediction model. In my model dt = 50 ms.
* Lf = 2.67 m - distance between front wheel axis of vehicle and it center of mass. This is fixed vehicle parameter.
* d(t) = D \* w(t) - steering angle, where D - maximal steering angle, w(t) - steering actuator value of vehicle simulator in range [-1, 1]. This is first control parameter we want to estimate and return back to vehicle simulator. D = 25 degrees - known value. **Note: In this model d(t) can be used directly, because tangent function near 0 could be approximated by linear function f(x)=x. But I used more accurate value in my model.**
* a(t) = A(t) \* h(t) - acceleration, where A(t) - maximal vehicle acceleration for fixed speed range and transmission value, h(t) - throttle value - second control parameter we want to estimate.
A(t) is important, but unknown function for car simulator. It will be discussed separately.
* wlag - steering lag, determines time between moment when value is received by vehicle (simulator) and moment when wheels change it position. wlag = 6 in my model (300 ms).
* hlag - acceleration lag, determines time between moment when value is received by vehicle (simulator) and moment when throttle change it position. hlag = 6 in my model (300 ms).

##### 1.2.1 Acceleration function A(t)

A(t) is important, but unknown function. In real life it depends on multiple factors and can be a constant only in narrow speed range and fixed transmission value. Because it's hardly possible calculate it precisely for simulator, I decided select constant, but reasonable value.
* Big value means vehicle can change trajectory more fast and trajectory could be more steep. And really, more steep trajectory will be predicted by MPC. And if value is too big, vehicle will leave the track.
* Low values lead to more smooth trajectory to be predicted and vehicle will cutoff corners.Tthis is not a good as well.

I selected A = 0.4 m/s which real acceleration value for average vehicle and speed near 50 miles per hour. Vehicle simulator demonstrates perfect results with this value.

##### 1.3 Loss function
Loss function is set of penalties which used by optimizer algorithm to select unknown variables. In my model it's w(t) and h(t) - steering actuator value and throttle.
Some components of loss function, like Cross Track Error could be additionally included in state vector. But these variables derive from other and don't increase accuracy of model, but decrease it performance (speed).

Loss functions components:
1. cte(t) = (cos(atan(f'(t))) \* (f(t) - y(t)))^2 \* |v(t)| \* dt - integral cross track error (iCTE), tangential distance to target trajectory multiplied by speed and prediction step time length. This loss function keep vehicle close to target trajectory as possible.
2. ev(t) = (v(t) - tspeed)^2 - difference between predicted vehicle speed and target speed. tspeed = 100 miles per hour in my model. This loss function keeps vehicle speed near target speed limit.
3. edd(t) = (d(t) - d(t-1))^2 - steering angle changing speed. This loss function prevents vehicle from rapid changes of steering angle.
4. eda(t) = (a(t) - a(t-1))^2 \* (1 + sign(a(t) - a(t-1))) - acceleration increasing speed. This loss function prevents vehicle from too rapid acceleration, but don't limit braking speed.
5. eψ(t) = sin(ψ(t) - atan(f'(t)) - sinus of angle between vehicle and target trajectory. Sinus is used to avoid angle ambiguous. This loss function helps keep vehicle in direction of target trajectory.
6. ed(t) = d(t)^2 - magnitude of steering angle, prevents steering with big angles.
7. ea(t) = a(t)^2 - magnitude of acceleration, prevent vehicle accelerate too fast.

Where:
* f(t) - target trajectory function.
* f'(t) - derivative of target trajectory function.
* tspeed - target speed.

I tried different combinations of these loss functions. But it's really non-easy select reasonable  weights for all these functions. Moreover most of these functions have no visible positive effect for model. Rapid acceleration or big steering angle is not so bad and limiting it makes vehicle less maneuverable that it is. Finally I selected first 3 loss functions and tuned parameters for it. Other functions remain with zero weights.

#### 2. Timestep Length and Elapsed Duration (N & dt)
Timestamp length (number of steps in prediction model) and duration of each step was selected empirically. Number of steps depends mostly on optimization algorithm performance and computer speed. I used **Ipopt** optimization algorithm which has good convergence in reasonable time. Decreasing number of steps do model less accurate. Increasing it increase communication lag and vehicle simulator become less manageable.
I selected reasonable value of N = 20 steps.

Duration of each step depends on communication lag and lag of actuators from one side. From other side, it's limited by path length which will be predicted. Small predicted path length don't allow predict vehicle behavior in time enough to make correct maneuver. Big prediction cause very good convergence of vehicle trajectory and target trajectory far from vehicle, but almost ignore part of trajectory near the vehicle, which more important for steering angle and acceleration estimation.
I selected dt = 100 ms.

Previously I tried different combinations of N = {10, 20, 40} and dt = {50, 100, 150} ms.

#### 3. Polynomial Fitting and MPC Preprocessing

In my vehicle model target trajectory function plays very important role in loss function calculation. I use 3rd-order polynomial (spline) to approximate nearest forward piece of target trajectory in each moment of time. For this purpose I use waypoints received from vehicle simulator.
I transform waypoint in vehicle coordinate system and after that calculate spline which fit these waypoints maximally. For this purpose I use Householder QR decomposition of rectangle matrix with help of Eigen library.

Using vehicle coordinate system allow more accurate and predictable calculations, because algorithm become independent from vehicle position in the world. And this transformation transparent for other vehicle parameters like speed and values of actuators.

I my MPC algorithm I used rough values of steering angle [-1, 1] and throttle [-1, 1] provided by simulator and return predicted values back from model without transformation as well. Sure, it's multiplied by appropriate scaling factors in vehicle model. And these values was already discussed.

But I used speed value transformation from miles per hour to meters per second. All other values in my vehicle model in meters and seconds, so it's necessary transformation for model accuracy.

#### 4. Model Predictive Control with Latency
I took in account next 3 types of latency which exists in real life and in simulator as well:
* Connection latency - difference between time when values was transferred from simulator and time when response was returned back. In my code it's artificially increased to 100 ms with delay function. But in real life and on different environments these values could be different. Therefore I calculated average connection latency during last several seconds and used this more accurate value in my algorithm. I took in account same connection latency value for whole model for all N periods and updated values of control variable with less frequency. But N and dt was always fixed. Next, I returned back not first value, but shifted by number of latency periods. latency period equals to latency value divided by dt and rounded.
* Steering actuator latency. Naturally, it takes some time to change position of vehicle wheels. And it's true for simulator as well. Adding steering latency made vehicle more predictable on higher speeds. Empirically I found that steering latency is near 300 ms. It's much bigger than connection latency and has significant impact on prediction of vehicle behavior.
* Throttle latency. Similar to Steering actuator latency, time necessary to change throttle position. This value has less impact on vehicle behavior, therefore I didn't estimate it directly and assumed it equals to Steering actuator latency value.  
