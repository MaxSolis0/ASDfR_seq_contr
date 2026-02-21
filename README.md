Package seq_contr
-----------------------------------------------
Description: This package aims to accomplish the tasks in Assignment 1.2 of basic image processing with ROS2: the sequence controller and the tracking of an object in the camera.

Building instructions: Place folder in src. Build normally via colcon build.

*****Node: setpoint_sequence_node*****
Input:
/output/robot_pose
        Type: geometry_msgs/msg/PoseStamped
        (the orientation is supposed to be quaternion format but only z is used)
/
	    Type: geometry_msgs/msg/Point
	

Output:
/input/left_motor/setpoint_vel
	    Type: example_interfaces/msg/Float64
	
/input/right_motor/setpoint_vel
	Type: example_interfaces/msg/Float64
	
/target
	Type: geometry_msgs/msg/Point
	The x,y,z coordinates of the desired point to travel.
	
Run:
	ros2 launch seq_contr relbot_test.launch.xml
	# To give a new setpoint during runtime:
	(first ensure setpoints is true):
	ros2 param set /setpoint_sequence_node setpoints true
	ros2 param set /setpoint_sequence_node setpoints_array "[10.0, 1.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0]"

Parameters:
	bool setpoints: Defines if the robot should respond to setpoints, Default true.
	double array setpoints_array: The setpoint coordinates in array form with format [duration, x, y, z ], Default empty.
	double kp_linear: Proportional gain for linear velocity based on error, Default: 10.0
	double kp_angular: Proportional gain for rotation based on error of heading angle, Default: 50.0
	double wheel_base: Distance between wheels, required for Differential Drive model, Default: 0.209
	
Core components: 
	timer_callback(): Runs the periodic programs and selects tracking mode (setpoint or object from camera)
	sequence_controller(): Differential Drive proportional controller from X,Y target to left and right wheel speeds
	object_controller(): 
	publish_cmd(): Sends the wheel speeds to the relbol_simulator node in proper structure
        parse_sequence(): Parses setpoint double array into Point array
	parameters_callback(): Manages changes to parameters during runtime
	pose_callback(): Parses robot_pose into doubles 
