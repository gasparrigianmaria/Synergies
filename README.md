# Synergies
Code to let the QbLegs robot to walk with synergies. It also adapts the stride as function of the speed.
--------

 Code provided by Gian Maria Gasparri Ph.D. Centro E. Piaggio 09/07/2017

 This code allows the robot to transit from one speed to another one. 

 In the specific, trajectory references are returned by a synergy mapping function
 which takes the speed and foot height desired values as inputs. 
 A service allows the user to modify the desired speed. 
 (IN FUTURE this service will be activated by a speed control.)
 
 Once the desired speed is modified the new trajectories are provided. 
 Hence previous signals and newest ones has to be coherently concatenated to provide
 a transient as smooth as possible. The transition occurs  when the norm position error is minimum. 
 (FUTURE WORK will consider also the speed, i.e. the robot state.)
 
 
 The code can be shared in several sections
 1 - INIT - Create publisher message and service in order to provide signlas and to modify them
     when necessary.
 2 - INIT - Create matrix and vector support.
 3 - TRAJ - Obtain interpolated trajectories exploiting a synergy mapping function. This function
     returns resampled data as function of the publishing rate.
 4 - TRANS - Find the best transition instant.
 5 - TRANS - Wait for it and then switch.
 6 - PUB - Publish new trajectories
 
