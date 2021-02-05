# ProbRobProject
This is the final project for the course of Probabilistic Robotics (2020/2021) - Sapienza University of Rome  

## Planar Monocular SLAM
A planar robot (z=0), on which was mounted a camera, has navigated through an environment populated with landmarks placed in 3D space.  

Input:  
* Integrated dead reckoning  
* Stream of point projections with "id"  
* Camera Parameters:  
    * Extrinsics (pose of camera on robot)  
    * Instrinsics (K)  
    
Expected output:  
* Trajectory  
* Map  

The proposed solution can be subdivided in 3 phases:  
* Initialization of the landmarks by using the odometry guess and triangulating all the available views for each landmark  
* Optimization of the landmarks by running the least squares solver for a few iterations only on the landmarks and blocking the poses of the robot.  
* Getting the final trajectory and map by running the least squares solver on both landmarks and poses of the robot.  

## Dataset:
The dataset was provided by the teacher  

## Author: 
[Ionut Marian Motoi](https://github.com/IonutMotoi)  
