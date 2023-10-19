# Orbital
ESMD Capstone project

## Frame assignment + transforms:

## Kinematics:
robot.inverseK() returns:
  - motorTransAngles:   motor angles to achieve desired robot position (1x4 numpy array) [rad]
  - cableVectors:       unit vectors that describe each cable pointing from robot outlets to respective support fixtures (3x4 numpy array) [i, j, k]
  - cableLengths:       Length of each cable (1x4 numpy array) [m]
  - spoolRadii:         Radii at which each spool is currently at, this changes the torque needed for a desired output (1x4 numpy array) [m]
  
## Motion:

### Model:
  State Space model is as follows:
  $$\dot{x} = Ax + Bu$$
  $$Q = \begin{bmatrix} x\\ \dot{x}\\ y\\ \dot{y}\\ z\\ \dot{z}\\ \end{bmatrix}$$
    
This model is non-linear time-varying in the B matrix. To handle this, attempting to implement iLQR. To respect constraints attempting to wrap in augmented lagrangian.


