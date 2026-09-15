The TRAC-IK kinematics solver is built in trac\_ik\_lib as a .so library (this
has been tested using ROS Indigo using Catkin).  The headers and shared
objects in this package can be linked against by user programs.

###As of v1.6.0, this package is part of the ROS Noetic binaries: `sudo apt-get install ros-noetic-trac-ik`
###As of v1.4.3, this package is part of the ROS Indigo/Jade binaries: `sudo apt-get install ros-jade-trac-ik`

This requires the Ubuntu packages for NLOpt Libraries to be installed (the
ros-indigo-nlopt packages do not use proper headers).  This can be done by
running ```sudo apt-get install libnlopt-dev``` on the trusty (and later)
standard Ubuntu distros.  Alternatively, you can run ```rosdep update &&
rosdep install trac_ik_lib```.

KDL IK:

```c++
KDL::ChainFkSolverPos_recursive fk_solver(chain);
KDL::ChainIkSolverVel_pinv vik_solver(chain);
KDL::ChainJntToJacSolver jac_solver(chain);

KDL::ChainIkSolverPos_NR_JL ik_solver(KDL::Chain chain, KDL::JntArray lower_joint_limits, KDL::JntArray upper_joint_limits, fk_solver, vik_solver, int num_iterations, double error);

int rc = ik_solver.CartToJnt(KDL::JntArray joint_seed, KDL::Frame desired_end_effector_pose, KDL::JntArray& return_joints);

% NOTE: CartToJnt succeeded if rc >=0

% NOTE: to use a timeout in seconds (e.g., 0.005), the iterations can be set to 1, and this can be called in a loop with your own timer.

% NOTE: error == 1e-5 is acceptable for most purposes
```

TRAC-IK:

```c++
#include <trac_ik/trac_ik.hpp>

TRAC_IK::TRAC_IK ik_solver(KDL::Chain chain, KDL::JntArray lower_joint_limits, KDL::JntArray upper_joint_limits, rclcpp::Logger logger=rclcpp::get_logger("trac_ik.trac_ik_lib"));  

% NOTE: the constructor describes the mechanism and nothing else, so one
% solver can answer many queries. Build the chain yourself with kdl_parser;
% there is no longer a constructor that reads a URDF parameter.

TRAC_IK::Query query;                        % everything that may differ between calls
query.timeout = 0.005;                       % seconds
query.epsilon = 1e-5;
query.solve_type = TRAC_IK::Speed;
query.tolerance_bounds = KDL::Twist::Zero(); % per-axis tolerances in the goal frame
% query.q_min, query.q_max                   % joint bounds for this call; empty = the mechanism's own
% query.stall_window                         % how long the search may go without improving, as a fraction of the timeout

% The solve type can be one of the following: 
% Speed: returns very quickly the first solution found
% Distance: runs for the full timeout_in_secs, then returns the solution that minimizes SSE from the seed
% Manip1: runs for full timeout, returns solution that maximizes sqrt(det(J*J^T)) (the product of the singular values of the Jacobian)
% Manip2: runs for full timeout, returns solution that minimizes the ratio of min to max singular values of the Jacobian.
% Manip3: runs for full timeout, returns solution that maximizes the smallest singular value of the Jacobian.

int rc = ik_solver.CartToJnt(KDL::JntArray joint_seed, KDL::Frame desired_end_effector_pose, KDL::JntArray& return_joints, TRAC_IK::Query query);

% NOTE: CartToJnt succeeded if rc >=0	

% NOTE: the query is optional, and its tolerances default to 0. If not
% provided, then by default are 0.  If given, the ABS() of the
% values will be used to set tolerances at -tol..0..+tol for each of
% the 6 Cartesian dimensions of the end effector pose.
```


