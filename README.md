# slam_evaluation


This package does slam/odometry evaluation according to metrics in https://ieeexplore.ieee.org/document/6385773

## Equations:

1. Relative pose error at time step i:
$E_i = (Q_iQ_{i+\triangle})^{-1}(P_iP_{i+\triangle})$

where:
- $Q_i$ - ground truth pose in SE(3) at time step $i$, a.k.a. homogeneous transformation matrix 
- $Q_{i+\triangle}$ - ground truth pose in SE(3) at time step $i+\triangle$ 
- $P_i$ - estimated pose in SE(3) at time step $i$, a.k.a. homogeneous transformation matrix
- $P_{i+\triangle}$ - estimated pose in SE(3) at time step $i+\triangle$ 

2. Relative pose error of the whole trajectory:
$RMSE(E_{1:n},\triangle) = (\frac{1}{m} \sum_{i=1}^m  ||trans(E_i)||^2)^\frac{1}{2}$

where:
- $trans(E_i)$ - translational part of matrix $E_i$, indexes $[0,3],[1,3],[2,3]$

3. Average relative pose error over possible time intervals $\triangle$
$RMSE(E_{1:n}) = \frac{1}{n} \sum_{\triangle=1}^n RMSE(E_{1:n},\triangle)$ 

where:
- $\triangle$ - step size, in this implementation the biggest step size is half of the trajectory size $n$

## Usage:

This is a ros package, so you should have ros installed, the only additional requirement is Eigen3 https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html

1. Build package `catkin build slam_evaluation`
2. Run executable `rosrun slam_evaluation slam_evaluation`

## Topics:
The package subscribes on 2 topics: both of them should be of type `nav_msgs::Odometry`. Their initial names are: `/unitree_odom` and `/loam_odom`

After all the calculations resulting rmse is published into topic `rmse`

