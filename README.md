Package seq_contr
-----------------------------------------------
### Description
This package aims to accomplish the tasks in Assignment 1.2 of basic image processing with ROS2: the sequence controller and the tracking of an object in the camera.

### Inputs
/output/robot_pose
        Type: geometry_msgs/msg/PoseStamped
        (the orientation is supposed to be quaternion format but only z is used)
        
/tracker_CoM
        Type: geometry_msgs/msg/Point
	    
/output/camera_position
        Type: geometry_msgs/msg/PointStamped

/tracked_bbox
        Type: vision_msgs/msg/BoundingBox2D

Output:
/input/left_motor/setpoint_vel
        Type: example_interfaces/msg/Float64
	
/input/right_motor/setpoint_vel
        Type: example_interfaces/msg/Float64
	
/target
        Type: geometry_msgs/msg/Point
	
### Run
	ros2 launch seq_contr seq_contr.launch.py 
	# To give a new setpoint during runtime:
	# (first ensure setpoints is true):
	ros2 param set /setpoint_sequence_node setpoints true
	# input one or more new setpoints in format [duration, x, y, z]
	ros2 param set /setpoint_sequence_node setpoints_array "[10.0, 1.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0]"

### Parameters
	bool setpoints: Defines if the robot should respond to setpoints, Default true.
	double array setpoints_array: The setpoint coordinates in array form with format [duration, x, y, z ], Default empty.
	double kp_linear: Proportional gain for linear velocity based on error, Default: 10.0
	double kp_angular: Proportional gain for rotation based on error of heading angle, Default: 50.0
	double wheel_base: Distance between wheels, required for Differential Drive model, Default: 0.209
	bool mode: If the robot should assume input from camera (simulation open loop) or moving_camera (simualtion closed loop), Default: true
	int width: Width of camera window, Default: 360
	double desired_width: Desired width of tracked object in camera view, Default: 100.0
	
### Core components 
	timerCallback(): Runs the controller periodically for a sequence of setpoints or stops the robot if finished.
	sequenceController(): Differential Drive proportional controller from X,Y target to left and right wheel speeds.
	objectController(): Differential Drive proportional controller to track object in camera view.
	publishCmd(): Sends the wheel speeds to the relbol_simulator node in proper structure.
	parametersCallback(): Manages changes to parameters during runtime
