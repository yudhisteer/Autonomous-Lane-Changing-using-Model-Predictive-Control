# Autonomous Lane Changing using Model Predictive Control


## Abstract

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/160285131-79c71a9a-599a-48a7-b84b-99d649ab8dd9.gif" />
</p>



## Research Questions

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/160285501-ca5c39e0-d4ff-4703-b028-93c07eaa192a.png" />
</p>

## Methods

## Plan of Action
1. PID vs MPC
    - Controller Schematics
    - Reference Frames
    
2. Lateral Control using Equations of Motion

    - Bicycle Model
    - Ackerman Steering
    - Longitudinal and Lateral Velocities
    - Equations of Motion in Lateral Direction
    - Understanding Centripetal Acceleration
    - Modelling the Front Wheel


3. State-Space and Linear Time Invariant (LTI)
    - Equations of motion to State-space
    - Adding Extra States
    - Computing New States
    - Linear Time Invariant (LTI)
    - LTI Model with Small Angle Approximation

4. MPC of Rocket
    - Thrust Levels
    - Cost Function
    - Noise
    - Weights
    - Horizon Period
    - Kalman Filter
    - Quadratic and Other Cost Functions

5. MPC of Autonomous Car
    - Forward Euler Method
    - Predicting Future States
    - Deriving a Cost Function
    - Regulating <img src="https://latex.codecogs.com/svg.image?\delta&space;" title="https://latex.codecogs.com/svg.image?\delta " />
    - Advantages of Predicting
    - Errors as State-space equations


## Implementation

### 1. PID vs MPC
The ```problem statement``` is simple: we have an obstacle (a car at rest) on our lane, we want to achieve a lane changing maneuvre to avoid that obstacle. So how does this work? We assume we have a lidar system or a pseudo-lidar system using stereo cameras which will detect the obstacle and plan a trajectory for lane changing. The goal here is to compute <img src="https://latex.codecogs.com/svg.image?\delta&space;" title="https://latex.codecogs.com/svg.image?\delta " /> degrees which is the ```angle``` required for the front wheel to rotate in order to achieve this planned trajectory.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/160287947-1e833c1c-dca9-42cf-bf65-a0507b913145.gif"/>
</p>

So we need to design a ```controller``` which will derive automatically which <img src="https://latex.codecogs.com/svg.image?\delta&space;" title="https://latex.codecogs.com/svg.image?\delta " /> degrees is optimal. To simplify our system, we will use the line diagram in the picture below shown on the right. <img src="https://latex.codecogs.com/svg.image?\delta&space;" title="https://latex.codecogs.com/svg.image?\delta " /> is the angle between the vertical axis and the local axis of the wheel.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/160290549-9a6046e2-d1b7-49ba-86cc-df0b0fdfd4b4.png"  width = "350" height = "300"/>
</p>

We will need to make a couple of assumptions in order to simplify our system:

- We will be moving at a ```constant velocity```.
- We already have a ```trajectory pre-planned``` - it can be the optimal solution from another system.
- We assume the car is moving on a ```flat surface``` so that we can ignore the Z-axis.

From the assumptions made above, we have a ```3 degree of freedom (dof)``` system. That is, we only require ```3``` variables in order to derive the position and orientation of the car - <img src="https://latex.codecogs.com/png.image?\dpi{110}(x,y,\psi)" title="https://latex.codecogs.com/png.image?\dpi{110}(x,y,\psi)" /> where <img src="https://latex.codecogs.com/png.image?\dpi{110}\psi" title="https://latex.codecogs.com/png.image?\dpi{110}\psi" /> is the angle in radians of the orientation of the vehicle w.r.t to a reference frame.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/160291256-16616701-ed7a-481a-834e-ee5e5bebaf25.png"  width="300" height="300"/>
</p>

Our goal will be to overtake the obstacle, that is changing the lane, in ```7``` seconds. While this may seem simple, it is quite a challenging problem as we have to take into consideration the ```comfort``` of the passenger in the vehicle, the ```safety``` of the passengers in the vehicle and of other vehicles and also the ```efficiency``` of our system.

**Scenario 1: Staight Line Trajectory**

We can have a case where we can straight line trajectory for lane overtaking. Also, it can be an easy task computationally, it will not be a realistic case. In real-life, it is impossibe to have such sharp turns and suppose if we can acheive it then it will be quite uncomfortable for our passengers. Hence, we can scrape scenatio 1.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/160291980-5227a42d-d025-40ed-bfac-41cedb4b20e6.png" />
</p>


**Scenario 2: Bézier Curve Trajectory**

The reason we use a spline or more specifically a Bézier curve is in order to acheive a more realistic scenario which we would expect in real-life. We can be sure to acheive a smooth transition during overtaking which will allow for the comfort and safety of the passengers.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/160292453-f61bbdd6-b157-4441-99c6-0958440e1f10.png" />
</p>

For now, we are assuming our obstacles are constant however, if we had a system which could determine the speed at which other vehicles are moving and if we could determine the length of the vehicles then we could determine this overtaking trajectory on the go instead of pre-planned.

#### 1.1 Controller Schematics
We will start our controller schematics with an ```Open-Loop``` system. Our input will be <img src="https://latex.codecogs.com/png.image?\dpi{110}\delta&space;" title="https://latex.codecogs.com/png.image?\dpi{110}\delta " />, that is the angle of rotation of the front wheels, and our output will be <img src="https://latex.codecogs.com/png.image?\dpi{110}(x,y,\psi)" title="https://latex.codecogs.com/png.image?\dpi{110}(x,y,\psi)" />.


<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/160295046-88c9e547-dd4e-479d-9fdf-b0944c8db68a.png" width="600" height="230"/>
</p>

Once we have <img src="https://latex.codecogs.com/png.image?\dpi{110}x,y,\psi&space;" title="https://latex.codecogs.com/png.image?\dpi{110}x,y,\psi " />, we need to compare them to our reference values.

- <img src="https://latex.codecogs.com/png.image?\dpi{110}x,y,\psi&space;" title="https://latex.codecogs.com/png.image?\dpi{110}x,y,\psi " />: where we are right now
- <img src="https://latex.codecogs.com/png.image?\dpi{110}x_{R},y_{R},\psi&space;_{R}" title="https://latex.codecogs.com/png.image?\dpi{110}x_{R},y_{R},\psi _{R}" />:  where we want to be

In the open loop system above, we already have our <img src="https://latex.codecogs.com/png.image?\dpi{110}\delta&space;" title="https://latex.codecogs.com/png.image?\dpi{110}\delta " /> as the input, so now we need to build the ```closed-loop``` system which will allow us to derive the <img src="https://latex.codecogs.com/png.image?\dpi{110}\delta&space;" title="https://latex.codecogs.com/png.image?\dpi{110}\delta " />.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/160295504-910774fb-a952-4096-b49c-bc8a74ad0c1d.png" width="600" height="210"/>
</p>

- <img src="https://latex.codecogs.com/png.image?\dpi{110}\begin{bmatrix}x_{R}\\y_{R}\\\psi&space;_{R}\\\end{bmatrix}-\begin{bmatrix}x&space;\\y&space;\\\psi&space;\end{bmatrix}=\begin{bmatrix}e_{x}\\e_{y}\\e_{\psi}\\\end{bmatrix}" title="https://latex.codecogs.com/png.image?\dpi{110}\begin{bmatrix}x_{R}\\y_{R}\\\psi _{R}\\\end{bmatrix}-\begin{bmatrix}x \\y \\\psi \end{bmatrix}=\begin{bmatrix}e_{x}\\e_{y}\\e_{\psi}\\\end{bmatrix}" />

The error vector will be the ```input``` of our controller and its job, its ```output```, will be to find the best steering wheel angle, <img src="https://latex.codecogs.com/png.image?\dpi{110}\delta" title="https://latex.codecogs.com/png.image?\dpi{110}\delta" />, so that the errors go to ```0```. The errors go to zero when <img src="https://latex.codecogs.com/png.image?\dpi{110}x_{R}&space;=&space;x" title="https://latex.codecogs.com/png.image?\dpi{110}x_{R} = x" />, <img src="https://latex.codecogs.com/png.image?\dpi{110}y_{R}&space;=&space;y" title="https://latex.codecogs.com/png.image?\dpi{110}y_{R} = y" /> and <img src="https://latex.codecogs.com/png.image?\dpi{110}\psi&space;_{R}&space;=&space;\psi&space;" title="https://latex.codecogs.com/png.image?\dpi{110}\psi _{R} = \psi " />.

Suppose we are moving at 20 m/s then in 7 s, our distance covered in the y-direction will be ```140 m``` whereas the distance covered in the x-direction will be only ```4 m```. An approximate of the yaw-angle is arctan(4/140) which is ```1.64``` degrees which is very small. Hence, we deduce <img src="https://latex.codecogs.com/png.image?\dpi{110}\psi&space;" title="https://latex.codecogs.com/png.image?\dpi{110}\psi " /> depends only on the y-direction and we can ignore the x-direction.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/160296551-3963804d-16df-4f81-8337-88969e3bc087.png" />
</p>

**Note:** We do not need to control the Forward Velocity and we will simplify our system to make is 2 dof such that we now only need to variables: y and <img src="https://latex.codecogs.com/png.image?\dpi{110}\psi&space;" title="https://latex.codecogs.com/png.image?\dpi{110}\psi " />:

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/160296834-d37a07b9-390d-4eed-9fa7-018e9e66f7f2.png" width="600" height="150"/>
</p>

The reason we will be using a MPC instead of a PID is becasue a PID is a Single Input Single Output (SISO) whereas a MPS is a Multiple Input Multiple Output (MIMO). We can still use a PID controller but we have have to tune 6 constants - 3 constants <img src="https://latex.codecogs.com/png.image?\dpi{110}(K_{P},K_{D},K_{I})" title="https://latex.codecogs.com/png.image?\dpi{110}(K_{P},K_{D},K_{I})" /> for each PID. Figure below shows the schematic if we had used PID controllers:

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/160297034-2a4f753d-f5c9-47b3-88c9-9bf4bef71312.png" width="600" height="130"/>
</p>




#### 1.2 Reference Frames





### 2. Lateral Control using Equations of Motion

#### 2.1 Bicycle Model

#### 2.2 Ackerman Steering

#### 2.3 Longitudinal and Lateral Velocities

#### 2.4 Equations of Motion in Lateral Direction

#### 2.5 Understanding Centripetal Acceleration

#### 2.6 Modelling the Front Wheel



### 3. State-Space and Linear Time Invariant (LTI)



#### 3.1 Equations of motion to State-space

## Conclusion

## References
1. https://www.youtube.com/watch?v=W5Eo5EEbt0s&t=240s
2. https://www.youtube.com/watch?v=3xDBu6yXSn4
3. https://openscholarship.wustl.edu/cgi/viewcontent.cgi?article=1064&context=mems500#:~:text=Obstacle%20avoidance%2C%20especially%20lane%20change,a%20safe%20and%20stable%20driving.
4. 
