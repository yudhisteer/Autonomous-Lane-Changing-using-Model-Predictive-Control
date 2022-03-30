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
    - Acceleration in lateral direction
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
  <img src= "https://user-images.githubusercontent.com/59663734/160297505-0f4e61cf-4c95-4d0f-8c0a-8a1c88db548f.png" width="700" height="180"/>
</p>


<p align="center">
  <img src= "https://latex.codecogs.com/png.image?\dpi{110}\begin{bmatrix}x_{R}\\y_{R}\\\psi&space;_{R}\\\end{bmatrix}-\begin{bmatrix}x&space;\\y&space;\\\psi&space;\end{bmatrix}=\begin{bmatrix}e_{x}\\e_{y}\\e_{\psi}\\\end{bmatrix}" title="https://latex.codecogs.com/png.image?\dpi{110}\begin{bmatrix}x_{R}\\y_{R}\\\psi _{R}\\\end{bmatrix}-\begin{bmatrix}x \\y \\\psi \end{bmatrix}=\begin{bmatrix}e_{x}\\e_{y}\\e_{\psi}\\\end{bmatrix}"/>
</p>

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
  <img src= "https://user-images.githubusercontent.com/59663734/160297163-7b0f8a86-61a9-413b-9f48-fce8aa562ed5.png" width="600" height="130"/>
</p>


#### 1.2 Reference Frames
We will now setup our reference frames: ```Inertial Reference Frame``` and a ```Body Frame```. The reason we have 2 reference frames is because our trajectory is defined in the Inertial Frame and therefore we need to translate our Body Frame's velocity into our Global Coordinate System.

Recall that we have a constant longitudinal velocity of 20 m/s.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/160376183-4f20cbdc-1be3-4093-8b7c-66e2862ca3a1.png" width="300" height="340"/>
</p>

From the image below we have <img src="https://latex.codecogs.com/png.image?\dpi{110}\dot{x}" title="https://latex.codecogs.com/png.image?\dpi{110}\dot{x}" /> and <img src="https://latex.codecogs.com/png.image?\dpi{110}\dot{y}" title="https://latex.codecogs.com/png.image?\dpi{110}\dot{y}" /> in our Body Frame and we need to transform it into <img src="https://latex.codecogs.com/png.image?\dpi{110}\dot{X}" title="https://latex.codecogs.com/png.image?\dpi{110}\dot{X}" /> and <img src="https://latex.codecogs.com/png.image?\dpi{110}\dot{Y}" title="https://latex.codecogs.com/png.image?\dpi{110}\dot{Y}" /> respectively in the Global Reference Frame.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/160378505-df829a89-aa1e-4b12-b52a-3c0fd83c0b53.png" width="500" height="320"/>
</p>


We start by finding the components of <img src="https://latex.codecogs.com/png.image?\dpi{110}V_{x}" title="https://latex.codecogs.com/png.image?\dpi{110}V_{x}" /> and <img src="https://latex.codecogs.com/png.image?\dpi{110}V_{y}" title="https://latex.codecogs.com/png.image?\dpi{110}V_{y}" /> in the Global Reference frame then compute the resultant velocities in the X and Y direction.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/160380738-c6c073d1-3606-4384-bd88-b6c492eaa2c2.png"/>
</p>

<p align="center">
  <img src= "https://latex.codecogs.com/png.image?\dpi{110}\dot{X}=V_{x_{x}}-V_{y_{x}}=V_{x}\cdot&space;cos(\frac{\pi}{6})-V_{y}\cdot&space;sin(\frac{\pi}{6})" title="https://latex.codecogs.com/png.image?\dpi{110}\dot{X}=V_{x_{x}}-V_{y_{x}}=V_{x}\cdot cos(\frac{\pi}{6})-V_{y}\cdot sin(\frac{\pi}{6})"/>
</p>

<p align="center">
  <img src= "https://latex.codecogs.com/png.image?\dpi{110}\dot{Y}=V_{x_{y}}&plus;V_{y_{y}}=V_{x}\cdot&space;sin(\frac{\pi}{6})&plus;V_{y}\cdot&space;cos(\frac{\pi}{6})" title="https://latex.codecogs.com/png.image?\dpi{110}\dot{Y}=V_{x_{y}}+V_{y_{y}}=V_{x}\cdot sin(\frac{\pi}{6})+V_{y}\cdot cos(\frac{\pi}{6})"/>
</p>

From the two computations above, we transform the Body Frame's velocities to velocities in the Global Refrence Frame.

### 2. Lateral Control using Equations of Motion
We will now design our ```plant``` to describe motion in the ```lateral``` direction. We will use that model/plant to later design our controller (MPC). The system will try to relate the system's input, <img src="https://latex.codecogs.com/png.image?\dpi{110}\delta&space;" title="https://latex.codecogs.com/png.image?\dpi{110}\delta " /> to the system's output <img src="https://latex.codecogs.com/png.image?\dpi{110}Y,\psi&space;" title="https://latex.codecogs.com/png.image?\dpi{110}Y,\psi " />. so we will have a function that describes <img src="https://latex.codecogs.com/png.image?\dpi{110}Y,\psi&space;" title="https://latex.codecogs.com/png.image?\dpi{110}Y,\psi " /> based on <img src="https://latex.codecogs.com/png.image?\dpi{110}\delta&space;" title="https://latex.codecogs.com/png.image?\dpi{110}\delta " /> - <img src="https://latex.codecogs.com/png.image?\dpi{110}f(\delta&space;)\mapsto&space;Y,\psi&space;" title="https://latex.codecogs.com/png.image?\dpi{110}f(\delta )\mapsto Y,\psi " />.

#### 2.1 Bicycle Model
Earlier we simplified our model with the line diagram, now we will further simplify our model with one of a ```Bicycle```. We use the Bicycle model as it represents the car's dynamics well enough and allow the design of the controller reliably.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/160399129-d1020a26-baaa-4a70-8a24-d5b67885f4b1.png" width="500" height="320"/>
</p>

**Scenario 1: Two instanteneous points with old cars model**

Long time ago, cars used to turn their entire front axle and not only the wheels to make a turn. However, turning the entire axle is not very efficient. When two wheels are locked on an axle so that they are not free to turn separately, one or the other has to slide. 

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/160397648-72e532c1-dc9f-4fb7-a1b5-f1c177187b29.gif"/>
</p>

In the figure below, we have two instances where the axle is turn at 15 degrees (green) and another at 30 degrees (red). If we draw two projection lines perpendicular to the inner front and back wheels, then these two lines intersect at the ```Instanteneous Center Point```. 

_The instanteneous centre of rotation/point is a point about which a rigid body in general plane motion can be considered to have purely rotational motion at that instant._

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/160451231-c8653a79-434b-4765-8a0b-f11647057436.png" width="500" height="470"/>
</p>

 We see that a mere change of 15 degrees pushes the centre points further apart and does not allow the car for sharp turns. This can be problematic in practice specially if we need to have tight corners and paralell parking.

**Scenario 2: Two instanteneous points with pivoting wheels**

We will now see at an improvement of our car model whereby we no longer need to turn the entire axle to make a turn but only need to turn the wheels. We see that the problem with this kind of configuration that when turning atleast one wheel will be ```slipping```.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/160453693-ebcb5545-9e1a-412e-8270-e2c4343bfc84.png" width="500" height="300"/>
</p>

So the question we should ask is: how can we turn the two wheels without turning the entire axle in such a way so as to have only **one** Instanteneous Center of Rotation?

**Scenario 3: One instanteneous point with new cars model**

The goal is to have two different angle of rotation of the front wheels with the inner wheel pivoted at a bigger angle than the outer wheel. With this solution we avoid turing an entire axle and avoid tire slipping.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/160454979-1fb13d8a-3ef2-40b9-af5a-d87d65ba44a1.png" width="500" height="300"/>
</p>

However, the Bicycle model does not take this into account as it has only one front wheel. However, <img src="https://latex.codecogs.com/png.image?\dpi{110}\alpha&space;-\delta\approx&space;0" title="https://latex.codecogs.com/png.image?\dpi{110}\alpha -\delta\approx 0" />, therefore, we can take <img src="https://latex.codecogs.com/png.image?\dpi{110}\delta" title="https://latex.codecogs.com/png.image?\dpi{110}\delta" /> as the front wheel angle.

#### 2.2 Ackerman Steering
Now, the question we should ask is that if we find <img src="https://latex.codecogs.com/png.image?\dpi{110}\delta" title="https://latex.codecogs.com/png.image?\dpi{110}\delta" /> from the Bicycle model from the MPC then how do we apply it to the car model? To approximate the ideal steering (<img src="https://latex.codecogs.com/png.image?\dpi{110}\delta_{1},\delta_{2}" title="https://latex.codecogs.com/png.image?\dpi{110}\delta_{1},\delta_{2}" /> where <img src="https://latex.codecogs.com/png.image?\dpi{110}\delta_{1}\neq&space;\delta_{2}" title="https://latex.codecogs.com/png.image?\dpi{110}\delta_{1}\neq \delta_{2}" />) that will cause **one** Centre of Instanteneous Rotation, we need to use ```Ackerman Steering```.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/160466671-65251697-2577-4699-94d3-c084e45b6c4c.png"/>
</p>

The intention of Ackermann geometry is to avoid the need for tires to ```slip``` sideways when following the path around a curve. As the steering moved, the wheels turned according to Ackermann, with the inner wheel turning further. 

When making a turn each tires' path has a different radius therefore, this means that each tire is rotating at a different rate. The wheels on the outside travel further distance so the angular velocities of the outer wheels are greater than the angular velocities of the inner wheels. But the wheels on the inside turn at a greater angle than those on the outside as shown below.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/160469036-dab712c9-a91e-4a9c-a2b3-23bf45aaae99.png"/>
</p>


**Note:** Modern cars do not use pure Ackermann steering, partly because it ignores important dynamic and compliant effects, but the principle is sound for low-speed maneuvers. 

#### 2.3 Longitudinal and Lateral Velocities
Earlier we said that when the car performs a lane changing maneuvre with a constant longitudinal velocity of 20 m/s then the car will mostly drive in the Longitudinal direction and not in the Lateral direction. However, for our Bicycle model, we also have a velocity in the ```lateral``` direction which is small compared to the longitudinal velocity.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/160474891-f212f0ef-f4f1-4a90-96d7-9930db557aea.png" width="600" height="400"/>
</p>

In the diagram above we see that we have a front velocity <img src="https://latex.codecogs.com/png.image?\dpi{110}V_{F}" title="https://latex.codecogs.com/png.image?\dpi{110}V_{F}" /> but all along the body of our bicycle we have velocities ```V``` with angles <img src="https://latex.codecogs.com/png.image?\dpi{110}\theta&space;_{1}" title="https://latex.codecogs.com/png.image?\dpi{110}\theta _{1}" /> till <img src="https://latex.codecogs.com/png.image?\dpi{110}\theta&space;_{4}" title="https://latex.codecogs.com/png.image?\dpi{110}\theta _{4}" /> of magnitude less than <img src="https://latex.codecogs.com/png.image?\dpi{110}V_{F}" title="https://latex.codecogs.com/png.image?\dpi{110}V_{F}" />. And we can draw a perpendicular projection from all these velocities and all of them will intersect at the Centre of Instanteneous Rotation. The magnitude of the angles are such that:

<p align="center">
  <img src= "https://latex.codecogs.com/png.image?\dpi{110}\theta&space;_{1}<\theta&space;_{2}<\beta&space;<\theta&space;_{3}<\theta&space;_{4}" title="https://latex.codecogs.com/png.image?\dpi{110}\theta _{1}<\theta _{2}<\beta <\theta _{3}<\theta _{4}"/>
</p>

#### 2.4 Equations of Motion in Lateral Direction
We will now derive the equations of motion which connect all forces to an acceleration and connect all moments to an angular acceleration.

<p align="center">
  <img src= "https://latex.codecogs.com/png.image?\dpi{110}\sum&space;F&space;=&space;&space;m\cdot&space;a" title="https://latex.codecogs.com/png.image?\dpi{110}\sum F = m\cdot a"/>
</p>

<p align="center">
  <img src= "https://latex.codecogs.com/png.image?\dpi{110}\sum&space;M&space;=&space;&space;I\cdot&space;\ddot{\psi&space;}" title="https://latex.codecogs.com/png.image?\dpi{110}\sum M = I\cdot \ddot{\psi }"/>
</p>


<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/160564573-f600c77f-848c-41e2-9dc2-2ba285c3be81.png" width="500" height="280"/>
</p>

Hence, from the equations above we derive the equations of motion in the lateral direction:

<p align="center">
  <img src= "https://latex.codecogs.com/png.image?\dpi{110}F_{y_{r}}&plus;F_{y_{r}}=m\cdot&space;a_{y}\xrightarrow[]{}1" title="https://latex.codecogs.com/png.image?\dpi{110}F_{y_{r}}+F_{y_{r}}=m\cdot a_{y}\xrightarrow[]{}1"/>
</p>

<p align="center">
  <img src= "https://latex.codecogs.com/png.image?\dpi{110}F_{y_{f}}\cdot&space;l_{f}-F_{y_{r}}\cdot&space;l_{r}=I\cdot&space;\ddot{\psi&space;}\xrightarrow[]{}2" title="https://latex.codecogs.com/png.image?\dpi{110}F_{y_{f}}\cdot l_{f}-F_{y_{r}}\cdot l_{r}=I\cdot \ddot{\psi }\xrightarrow[]{}2"/>
</p>

where

- <img src="https://latex.codecogs.com/png.image?\dpi{110}F_{y_{r}}" title="https://latex.codecogs.com/png.image?\dpi{110}F_{y_{r}}" /> and <img src="https://latex.codecogs.com/png.image?\dpi{110}F_{y_{f}}" title="https://latex.codecogs.com/png.image?\dpi{110}F_{y_{f}}" /> are fritional forces.
- <img src="https://latex.codecogs.com/png.image?\dpi{110}\ddot{\psi&space;}" title="https://latex.codecogs.com/png.image?\dpi{110}\ddot{\psi }" /> is the angular acceleration in <img src="https://latex.codecogs.com/png.image?\dpi{110}rad&space;\cdot&space;s^{-2}" title="https://latex.codecogs.com/png.image?\dpi{110}rad \cdot s^{-2}" />
- ```I``` is the moment of inertia in <img src="https://latex.codecogs.com/png.image?\dpi{110}Kg&space;\cdot&space;m^{2}" title="https://latex.codecogs.com/png.image?\dpi{110}Kg \cdot m^{2}" />
- ```m``` is the mass of the body in <img src="https://latex.codecogs.com/png.image?\dpi{110}Kg" title="https://latex.codecogs.com/png.image?\dpi{110}Kg" />
- <img src="https://latex.codecogs.com/png.image?\dpi{110}a_{y}" title="https://latex.codecogs.com/png.image?\dpi{110}a_{y}" /> is the acceleration in the lateral direction
- <img src="https://latex.codecogs.com/png.image?\dpi{110}l_{r}" title="https://latex.codecogs.com/png.image?\dpi{110}l_{r}" /> and <img src="https://latex.codecogs.com/png.image?\dpi{110}l_{f}" title="https://latex.codecogs.com/png.image?\dpi{110}l_{f}" /> are the perpendicular distance in ```m``` from the frictional forces to the Centre of gravity of the body, <img src="https://latex.codecogs.com/png.image?\dpi{110}C_{g}" title="https://latex.codecogs.com/png.image?\dpi{110}C_{g}" />.


#### 2.5 Understanding Centripetal Acceleration
We will further derive our equations of motion but before that we need to explore what is centripetal acceleration. 

_Centripetal acceleration is the the acceleration of a body traversing a circular path. Because velocity is a vector quantity (that is, it has both a magnitude and a direction), when a body travels on a circular path, its direction constantly changes and thus its velocity changes, producing an acceleration. The acceleration is directed radially toward the centre of the circle._

The figure below shows an object moving in a circular path at constant speed. The direction of the instantaneous velocity is shown at two points along the path. Acceleration is in the direction of the change in velocity, which points directly toward the center of rotation—the center of the circular path. We call the acceleration of an object moving in uniform circular motion—resulting from a net external force—the centripetal acceleration <img src="https://latex.codecogs.com/png.image?\dpi{110}a_{c}" title="https://latex.codecogs.com/png.image?\dpi{110}a_{c}" />; centripetal means “toward the center” or “center seeking”.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/160571893-f4253c5e-c484-45a1-8255-d09f643cec10.png" width="300" height="300"/>
</p>

In the image above, we have a body moving along the arc length ```x``` from point ```A``` to ```B```. The equation relating ```x```, ```r``` and <img src="https://latex.codecogs.com/png.image?\dpi{110}\psi&space;" title="https://latex.codecogs.com/png.image?\dpi{110}\psi " /> is:

<p align="center">
  <img src= "https://latex.codecogs.com/png.image?\dpi{110}x=r\cdot&space;\psi&space;" title="https://latex.codecogs.com/png.image?\dpi{110}x=r\cdot \psi "/>
</p>

It is important to note here that the linear velocity ```V``` is always perpendicular to the curved path - it always acts tangential to the circular path. We define velocity as the rate of change of displacement, therefore:

<p align="center">
  <img src= "https://latex.codecogs.com/png.image?\dpi{110}V&space;=&space;\frac{d(x)}{dt}=\dot{x}=\frac{d(r\cdot&space;\psi&space;)}{dt}=r\cdot&space;\dot{\psi&space;}" title="https://latex.codecogs.com/png.image?\dpi{110}V = \frac{d(x)}{dt}=\dot{x}=\frac{d(r\cdot \psi )}{dt}=r\cdot \dot{\psi }" />
</p>

where <img src="https://latex.codecogs.com/png.image?\dpi{110}\dot{\psi&space;}" title="https://latex.codecogs.com/png.image?\dpi{110}\dot{\psi }" /> is the rate of change of angular displacement.

If we now differentiate the equation above a second time, we will get the acceleration which is defined as the rate of change of velocity:


<p align="center">
  <img src= "https://latex.codecogs.com/png.image?\dpi{110}a_{c}=\frac{d^{2}(x)}{dt^{2}}=\frac{d(V)}{dt}=r\cdot&space;\ddot{\psi&space;}" title="https://latex.codecogs.com/png.image?\dpi{110}a_{c}=\frac{d^{2}(x)}{dt^{2}}=\frac{d(V)}{dt}=r\cdot \ddot{\psi }" />
</p>

We can also prove that the centripetal acceleration is equal to <img src="https://latex.codecogs.com/png.image?\dpi{110}\frac{V^{2}}{r}" title="https://latex.codecogs.com/png.image?\dpi{110}\frac{V^{2}}{r}" /> thanks to this derivation from Wikipedia:

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/160661910-17f39bf9-5708-464d-8fef-f0e92e2799ea.png" width="550" height="300"/>
</p>

We can further derive the centripetal accelearion to get:

<p align="center">
  <img src= "https://latex.codecogs.com/svg.image?a_{c}=\frac{V^{2}}{r}=\frac{\dot{x}\cdot&space;\dot{x}}{r}=\frac{(r\cdot&space;\dot{\psi&space;})\cdot&space;\dot{x}}{r}=\dot{\psi}&space;\cdot&space;\dot{x}" title="https://latex.codecogs.com/svg.image?a_{c}=\frac{V^{2}}{r}=\frac{\dot{x}\cdot \dot{x}}{r}=\frac{(r\cdot \dot{\psi })\cdot \dot{x}}{r}=\dot{\psi} \cdot \dot{x}" />
</p>

where:

- <img src="https://latex.codecogs.com/svg.image?\dot{x}" title="https://latex.codecogs.com/svg.image?\dot{x}" />: change of longitudinal velocity w.r.t time

- <img src="https://latex.codecogs.com/svg.image?\dot{\psi&space;}" title="https://latex.codecogs.com/svg.image?\dot{\psi }" />: change of angular displacement w.r.t time

**Note:** In our case, our longitudinal velocity is constant but the direction of the velocity changes hence, we have an acceleration

#### 2.6 Acceleration in lateral direction
In the lane changing project, the turns are so small and therefore the slip angle <img src="https://latex.codecogs.com/png.image?\dpi{110}\beta&space;" title="https://latex.codecogs.com/png.image?\dpi{110}\beta " /> is so small that we can neglect it and instead use <img src="https://latex.codecogs.com/png.image?\dpi{110}\dot{x}" title="https://latex.codecogs.com/png.image?\dpi{110}\dot{x}" /> instead.


<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/160675904-c8f55497-c250-45f4-9c7b-cb8daeef34ad.png" width="350" height="280"/>
</p>

Thus, from the theory explained above, we will then define our acceleration in the lateral direction as :

<p align="center">
  <img src= "https://latex.codecogs.com/png.image?\dpi{110}a_{y}=\ddot{y}&plus;a_{c}=\ddot{y}&plus;\dot{x}\cdot&space;\dot{\psi&space;}" title="https://latex.codecogs.com/png.image?\dpi{110}a_{y}=\ddot{y}+a_{c}=\ddot{y}+\dot{x}\cdot \dot{\psi }"/>
</p>

We can choose to either work with the ```Body Frame``` (Fixed to the vehicle) or work with the ```Inertial Frame``` (Fixed to the ground). When the vehicle rotates, the body frame rorates with it hence, for simplicity we will choose to work with the Body Frame.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/160681476-616eda26-1d00-4448-baf5-02217a97712e.png" width="620" height="300"/>
</p>

From the image above, if we go from configuration ```1``` to configuration ```2``` and assuming <img src="https://latex.codecogs.com/png.image?\dpi{110}\dot{y}" title="https://latex.codecogs.com/png.image?\dpi{110}\dot{y}" /> remains constant then 

<p align="center">
  <img src= "https://latex.codecogs.com/png.image?\dpi{110}\frac{d(\dot{y})}{dt}=0&space;ms^{-2}" title="https://latex.codecogs.com/png.image?\dpi{110}\frac{d(\dot{y})}{dt}=0 ms^{-2}"/>
</p>

Why is that? Because in the **Inertial Frame**, <img src="https://latex.codecogs.com/png.image?\dpi{110}\dot{y}" title="https://latex.codecogs.com/png.image?\dpi{110}\dot{y}" /> changes direction but in the **Body Frame** the direction of <img src="https://latex.codecogs.com/png.image?\dpi{110}\dot{y}" title="https://latex.codecogs.com/png.image?\dpi{110}\dot{y}" /> remains constant. Hence, <img src="https://latex.codecogs.com/png.image?\dpi{110}\dot{x}" title="https://latex.codecogs.com/png.image?\dpi{110}\dot{x}" /> and <img src="https://latex.codecogs.com/png.image?\dpi{110}\dot{y}" title="https://latex.codecogs.com/png.image?\dpi{110}\dot{y}" /> rotate together with the Body Frame and <img src="https://latex.codecogs.com/png.image?\dpi{110}\ddot{y}" title="https://latex.codecogs.com/png.image?\dpi{110}\ddot{y}" /> appears in the Body Frame only when the magnitude changes.

<img src="https://latex.codecogs.com/png.image?\dpi{110}a_{c}=\dot{x}\cdot&space;\dot{\psi&space;}" title="https://latex.codecogs.com/png.image?\dpi{110}a_{c}=\dot{x}\cdot \dot{\psi }" /> appears because the Body Frame is rotating w.r.t the Inertial Frame. If the Body Frame did not rotate then the second term in <img src="https://latex.codecogs.com/png.image?\dpi{110}a_{y}" title="https://latex.codecogs.com/png.image?\dpi{110}a_{y}" /> would be ```0``` because <img src="https://latex.codecogs.com/png.image?\dpi{110}\psi&space;" title="https://latex.codecogs.com/png.image?\dpi{110}\psi " /> would be ```0```.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/160682316-bb145af8-0714-4b28-a6fe-59ec2cb8f062.png"/>
</p>

Therefore:


<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/160683452-a563f2e0-506a-4888-adcd-11c5b2e6b7a5.png"/>
</p>

We update our equations of motion as we have expanded our terms in <img src="https://latex.codecogs.com/png.image?\dpi{110}a_{y}" title="https://latex.codecogs.com/png.image?\dpi{110}a_{y}" />:

<p align="center">
  <img src= "https://latex.codecogs.com/png.image?\dpi{110}F_{y_{f}}&plus;F_{y_{r}}=m(\ddot{y}&plus;\dot{x}\cdot&space;\dot{\psi&space;})\xrightarrow[]{}1" title="https://latex.codecogs.com/png.image?\dpi{110}F_{y_{f}}+F_{y_{r}}=m(\ddot{y}+\dot{x}\cdot \dot{\psi })\xrightarrow[]{}1""/>
</p>

<p align="center">
  <img src= "https://latex.codecogs.com/png.image?\dpi{110}F_{y_{f}}\cdot&space;l_{f}-F_{y_{r}}\cdot&space;l_{r}=I\cdot&space;\ddot{\psi&space;}\xrightarrow[]{}2" title="https://latex.codecogs.com/png.image?\dpi{110}F_{y_{f}}\cdot l_{f}-F_{y_{r}}\cdot l_{r}=I\cdot \ddot{\psi }\xrightarrow[]{}2"/>
</p>

#### 2.7 Modelling the Front Wheel
Before we assumed that the velocity of each wheel was in the direction of tha twheel. However, this is only **true** when the vehicle is moving at ```low speed```.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/160754534-9c549816-cf9d-422f-afdd-7666f2a8cd19.png" width="300" height="240"/>
</p>

When the vehicle is moving at high speed, it experiences ```tyre deformation```. How this happens? The tyre are made of rubber and the friction between the tyres and the road under lateral load causes deformation. So our contact patch is not going to be facing the exact same direction as the rest of the tyre which is not experiencing friction. 



<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/160758571-50c89208-2362-4443-b3fe-d11546c7923e.png" width="520" height="330"/>
</p>
                                                                                                                                            
The velocity fo the wheel will be between the longitudinal axis and the axis of the wheel - the velocity will be faicng the same direction as the direction of the contact patch. From the diagram above, we can therefore deduce the following equations:

<p align="center">
  <img src= "https://latex.codecogs.com/png.image?\dpi{110}\alpha&space;_{f}=\delta&space;-\theta&space;_{V_{f}}" title="https://latex.codecogs.com/png.image?\dpi{110}\alpha _{f}=\delta -\theta _{V_{f}}"/>
</p>


<p align="center">
  <img src= "https://latex.codecogs.com/png.image?\dpi{110}\alpha&space;_{r}=0&space;-\theta&space;_{V_{r}}=-\theta&space;_{V_{r}}" title="https://latex.codecogs.com/png.image?\dpi{110}\alpha _{r}=0 -\theta _{V_{r}}=-\theta _{V_{r}}"/>
</p>

The typical relation between the lateral force and the slip angle can be observed in the figure below. It is shown in expriments that slip angles are ```proportional``` to the lateral forces. 

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/160759066-7188b293-ad59-4a33-a6fb-153d5def9970.png" width="430" height="250"/>
</p>

                                                                                                                   
The tire model describes a ```linear relationship``` between tire lateral force and slip angle. Note this is valid only for small slip angles as shown on the graph above - for slip angles < 8 deg. 
                                                                                                                   
<p align="center">
  <img src= "https://latex.codecogs.com/png.image?\dpi{110}F_{y_{f}}&space;\propto\alpha&space;&space;_{f}" title="https://latex.codecogs.com/png.image?\dpi{110}F_{y_{f}} \propto\alpha _{f}"/>
</p>

<p align="center">
  <img src= "https://latex.codecogs.com/png.image?\dpi{110}F_{y_{r}}&space;\propto\alpha&space;&space;_{r}" title="https://latex.codecogs.com/png.image?\dpi{110}F_{y_{r}} \propto\alpha _{r}"/>
</p>

We can then express our lateral forces in terms of cornering stiffness constants ```C```: 

<p align="center">
  <img src= "https://latex.codecogs.com/png.image?\dpi{110}F_{y_{f}}&space;=&space;2\cdot&space;C_{\alpha&space;_{f}}\cdot&space;\alpha&space;_{f}" title="https://latex.codecogs.com/png.image?\dpi{110}F_{y_{f}} = 2\cdot C_{\alpha _{f}}\cdot \alpha _{f}"/>
</p>

<p align="center">
  <img src= "https://latex.codecogs.com/png.image?\dpi{110}F_{y_{r}}&space;=&space;2\cdot&space;C_{\alpha&space;_{r}}\cdot&space;\alpha&space;_{r}" title="https://latex.codecogs.com/png.image?\dpi{110}F_{y_{r}} = 2\cdot C_{\alpha _{r}}\cdot \alpha _{r}"/>
</p>


We already know the equation for our slip angles <img src="https://latex.codecogs.com/png.image?\dpi{110}\alpha&space;_{f}" title="https://latex.codecogs.com/png.image?\dpi{110}\alpha _{f}" /> and <img src="https://latex.codecogs.com/png.image?\dpi{110}\alpha&space;_{r}" title="https://latex.codecogs.com/png.image?\dpi{110}\alpha _{r}" />:


<p align="center">
  <img src= "https://latex.codecogs.com/png.image?\dpi{110}F_{y_{f}}&space;=&space;2\cdot&space;C_{\alpha&space;_{f}}\cdot&space;(\delta&space;-\theta&space;_{V_{f}})" title="https://latex.codecogs.com/png.image?\dpi{110}F_{y_{f}} = 2\cdot C_{\alpha _{f}}\cdot (\delta -\theta _{V_{f}})"/>
</p>

<p align="center">
  <img src= "https://latex.codecogs.com/png.image?\dpi{110}F_{y_{r}}&space;=&space;2\cdot&space;C_{\alpha&space;_{r}}\cdot&space;(-\theta&space;_{V_{r}})" title="https://latex.codecogs.com/png.image?\dpi{110}F_{y_{r}} = 2\cdot C_{\alpha _{r}}\cdot (-\theta _{V_{r}})"/>
</p>

Now, we need to figure out how can we write <img src="https://latex.codecogs.com/png.image?\dpi{110}\theta&space;_{V_{f}}" title="https://latex.codecogs.com/png.image?\dpi{110}\theta _{V_{f}}" /> and <img src="https://latex.codecogs.com/png.image?\dpi{110}\theta&space;_{V_{r}}" title="https://latex.codecogs.com/png.image?\dpi{110}\theta _{V_{r}}" /> in terms of our known variables: <img src="https://latex.codecogs.com/png.image?\dpi{110}\dot{x},\dot{y},l_{f},l_{r},\dot{\psi&space;}" title="https://latex.codecogs.com/png.image?\dpi{110}\dot{x},\dot{y},l_{f},l_{r},\dot{\psi }" />

##### 2.7.1 Front Wheel's Slip Angle

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/160768246-1eccc16d-a366-4962-b534-f4b98c906ca5.png" width="280" height="350"/>
</p>

We can find <img src="https://latex.codecogs.com/png.image?\dpi{110}\theta&space;_{V_{f}}" title="https://latex.codecogs.com/png.image?\dpi{110}\theta _{V_{f}}" /> using tan. <img src="https://latex.codecogs.com/png.image?\dpi{110}V_{L}" title="https://latex.codecogs.com/png.image?\dpi{110}V_{L}" /> can be seen as the resultant velocity in the lateral direction.

<p align="center">
  <img src= "https://latex.codecogs.com/png.image?\dpi{110}tan(\theta&space;_{V_{f}})=\frac{V_{L}}{\dot{x}}" title="https://latex.codecogs.com/png.image?\dpi{110}tan(\theta _{V_{f}})=\frac{V_{L}}{\dot{x}}"/>
</p>

Similar to finding the acceleration in the lateral direction, we have two scenarios. First, we can have lateral motion of the centre of mass as shown below in the configuration ```1``` and ```2```. Our lateral velocity is then

<p align="center">
  <img src= "https://latex.codecogs.com/png.image?\dpi{110}V_{L}&space;=&space;\dot{y}" title="https://latex.codecogs.com/png.image?\dpi{110}V_{L} = \dot{y}"/>
</p>

Or, we can have rotation about the centre of mass describing a circular path of radius <img src="https://latex.codecogs.com/png.image?\dpi{110}l_{f}" title="https://latex.codecogs.com/png.image?\dpi{110}l_{f}" /> which is the length from the centre of mass till the front of the wheel. In this configuration ```3```, we have 

<p align="center">
  <img src="https://latex.codecogs.com/png.image?\dpi{110}V_{L}=\dot{\psi&space;}\cdot&space;l_{f}" title="https://latex.codecogs.com/png.image?\dpi{110}V_{L}=\dot{\psi }\cdot l_{f}"  />
</p>

In our case, we have both translation and rotation of the Centre of mass therefore the lateral velocity is:

<p align="center">
  <img src="https://latex.codecogs.com/png.image?\dpi{110}V_{L}=\dot{y}&plus;\dot{\psi&space;}\cdot&space;l_{f}" title="https://latex.codecogs.com/png.image?\dpi{110}V_{L}=\dot{y}+\dot{\psi }\cdot l_{f}"  />
</p>

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/160770721-7f72eaf0-fcf3-42eb-9d47-76506b4a8a19.png"  width="670" height="320"/>
</p>
                                                                                                                                             
We can now update our equation of slip angle for front wheel as:
                                                                                                                                             
<p align="center">
  <img src= "https://latex.codecogs.com/png.image?\dpi{110}tan(\theta&space;_{V_{f}})=\frac{V_{L}}{\dot{x}}=\frac{\dot{y}&plus;\dot{\psi&space;}\cdot&space;l_{f}}{\dot{x}}" title="https://latex.codecogs.com/png.image?\dpi{110}tan(\theta _{V_{f}})=\frac{V_{L}}{\dot{x}}=\frac{\dot{y}+\dot{\psi }\cdot l_{f}}{\dot{x}}" />
</p>                                                                                                                                       

##### 2.7.2 Rear Wheel's Slip Angle
The rear wheel does not rotate and as such this creates a slowing effect. The lateral velocity for the back wheel then becomes:

<p align="center">
  <img src= "https://latex.codecogs.com/png.image?\dpi{110}V_{L}=\dot{y}-\dot{\psi&space;}\cdot&space;l_{r}" title="https://latex.codecogs.com/png.image?\dpi{110}V_{L}=\dot{y}-\dot{\psi }\cdot l_{r}" />
</p>      

Updating our sliping angle equation for the rear wheel:

<p align="center">
  <img src= "https://latex.codecogs.com/png.image?\dpi{110}tan(\theta&space;_{V_{r}})=\frac{V_{L}}{\dot{x}}=\frac{\dot{y}-\dot{\psi&space;}\cdot&space;l_{r}}{\dot{x}}" title="https://latex.codecogs.com/png.image?\dpi{110}tan(\theta _{V_{r}})=\frac{V_{L}}{\dot{x}}=\frac{\dot{y}-\dot{\psi }\cdot l_{r}}{\dot{x}}" />
</p>      


In order to calculate <img src="https://latex.codecogs.com/png.image?\dpi{110}\theta&space;_{V_{f}}" title="https://latex.codecogs.com/png.image?\dpi{110}\theta _{V_{f}}" /> and <img src="https://latex.codecogs.com/png.image?\dpi{110}\theta&space;_{V_{r}}" title="https://latex.codecogs.com/png.image?\dpi{110}\theta _{V_{r}}" /> we could use arctan, but this will only make the equations more complex. Instead we will use the ```small angle approximation```.

Since we are changing lanes, the steering angle would be very close so we can approximate:

<p align="center">
  <img src= "https://latex.codecogs.com/png.image?\dpi{110}tan(\theta&space;)\approx&space;\theta&space;" title="https://latex.codecogs.com/png.image?\dpi{110}tan(\theta )\approx \theta " />
</p>      

 We therfore have our slipping angles as:
 <p align="center">
<img src="https://latex.codecogs.com/png.image?\dpi{110}\theta&space;_{V_{f}}=\frac{\dot{y}&plus;\dot{\psi&space;}\cdot&space;l_{f}}{\dot{x}}" title="https://latex.codecogs.com/png.image?\dpi{110}\theta _{V_{f}}=\frac{\dot{y}+\dot{\psi }\cdot l_{f}}{\dot{x}}" />
  
<p align="center">
<img src="https://latex.codecogs.com/png.image?\dpi{110}\theta&space;_{V_{r}}=\frac{\dot{y}-\dot{\psi&space;}\cdot&space;l_{r}}{\dot{x}}" title="https://latex.codecogs.com/png.image?\dpi{110}\theta _{V_{r}}=\frac{\dot{y}-\dot{\psi }\cdot l_{r}}{\dot{x}}" />




### 3. State-Space and Linear Time Invariant (LTI)



#### 3.1 Equations of motion to State-space

## Conclusion

## References
1. https://www.youtube.com/watch?v=W5Eo5EEbt0s&t=240s
2. https://www.youtube.com/watch?v=3xDBu6yXSn4
3. https://openscholarship.wustl.edu/cgi/viewcontent.cgi?article=1064&context=mems500#:~:text=Obstacle%20avoidance%2C%20especially%20lane%20change,a%20safe%20and%20stable%20driving.
4. https://www.youtube.com/watch?v=oYMMdjbmQXc
5. https://en.wikipedia.org/wiki/Ackermann_steering_geometry
6. https://www.britannica.com/science/centripetal-acceleration
7. https://www.khanacademy.org/science/physics/centripetal-force-and-gravitation/centripetal-acceleration-tutoria/a/what-is-centripetal-acceleration
8. https://simple.wikipedia.org/wiki/Centripetal_force
9. https://www.youtube.com/watch?v=9bs2cEyK7Uo
10. https://andresmendes.github.io/openvd/build/html/tire.html                                                                                                                   
                                                                                                                                            
                                                                                                                                            
