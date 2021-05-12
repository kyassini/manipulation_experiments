# Misc files that may be needed for Gen3 configuration 

## gen3.launch:  
Launch file for the GPD_ROS package (`catkin_ws/src/gpd_ros/launch`). Replaces `urf.launch`

## ros_eigen_params.cfg
Gen3 params for GPD, should be palced in `gpd-2.0.0/cfg`

## gen3_robotiq_2f_85.srdf.xacro
srdf to be placed in `ros_kortex/kortex_move_it_config/gen3_robotiq_2f_85_move_it_config/config/7dof`  
Contains modified arm and end-effector group names that are neccesary for the code in this repo to function with the Gen3. Can also just change the planning groups in the code to match your srdf.