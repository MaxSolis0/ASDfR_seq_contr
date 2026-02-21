//==============================================================================
// Authors : Max Solis, Aleksei Obshatko
// Group : ASDfR 5
// License : LGPL open source license
//
// Brief : Node that controls the relbot based on, coordinate setpoints or 
// tracking an object.
//==============================================================================

#ifndef SETPOINT_SEQUENCE_NODE_HPP_
#define SETPOINT_SEQUENCE_NODE_HPP_

// ROS Client Library CPP
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

// RELbot simulator message types
#include "example_interfaces/msg/float64.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

// Object tracker message types
#include <geometry_msgs/msg/point.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>

// CPP library headers
#include <cmath>

/**
 * @brief Controller for RELbot.
 *
 * ROS2 parameters are described below:
 * @param setpoints [bool]: Which commands the robot should follow, true for coordinate setpoints, false for object tracking. Default: true.
 * @param setpoints_array [vector[double]]: The coordinates of the setpoints in format (duration, x, y, z). Default: {} (empty)
 * @param kp_linear [double]: Proportional controller gain for translation. Default: 10.0
 * @param wheel_base [double]: The distance between wheels. Default: 0.209 (from relbot_simulator.hpp)
 * @param mode [bool]: If the robot should assume input from camera (simulation open loop) or moving_camera (simualtion closed loop). Default: true
 * @param width [int]: The width of the window of camera view. Default: 360
 * @param desired_width [double]: The desired width in pixels of the object tracked to determine if the robot should move closer or farther from it. Default: 100.0
 */

class SetpointSequenceNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Setpoint Sequence Controller object
   *
   * @param time_step time-step defines how quickly the motors are simulated to catch up to commands.
   */
  SetpointSequenceNode();

private:
  //ROS interfaces
  //Topics
  rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr leftPub_;
  rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr rightPub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr targetPub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poseSub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr objectSub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr viewSub_;
  rclcpp::Subscription<vision_msgs::msg::BoundingBox2D>::SharedPtr bboxSub_;
  //Timer for setpoint sequence
  rclcpp::TimerBase::SharedPtr timer_;
  //Parameter callback for changes during runtime
  OnSetParametersCallbackHandle::SharedPtr paramCallbackHandle_;

  //Establish variables
  //Time step 
  double sampleTimeS_;
  //Sequence variables
  std::vector<double> setpointsArray_; //original array
  std::vector<geometry_msgs::msg::Point> sequence_; //points part of array
  std::vector<double> durations_; //durations part of array
  bool setpointsEnabled_;
  size_t seqIndex_ = 0;
  double elapsed_ = 0.0;
  //State (coordinate) variables
  double currentX_ = 0.0;
  double currentY_ = 0.0;
  double currentTheta_ = 0.0;
  
  //Parameters
  //Controller Parameters
  double kpLinear_;
  double kpAngular_;
  //Differential Drive Parameters
  double wheelBase_;
  //Object tracking Parameters
  bool mode_;
  int width_;
  double bboxWidth_;
  double desiredWidth_;
  double viewX_;

  /**
   * @brief Parse the sequence recieved into array of Points and array of durations 
   *
   */
  void parseSequence();

  /**
   * @brief Get parameters when changed
   *
   * @param params: vector of ROS parameters as previously described
   */
  rcl_interfaces::msg::SetParametersResult
    parametersCallback(const std::vector<rclcpp::Parameter> & params);

  /**
   * @brief Collect information from robot pose (x,y,thetaZ)
   *
   * @param msg: Robot pose, orientation is quaternion format but actually expresses only z as Euler angle while x,y,w are unchanged.
   */
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  /**
   * @brief Run the sequence controller periodically or stop robot if finished
   *
   */
  void timerCallback();

  /**
   * @brief Proportional Controller based on Differential Drive model.
   *
   */
  void sequenceController();

  /**
   * @brief Publish velocities to relbot_simulator
   *
   * @param left: Left wheel speed, opposite sign of traditional Differential Drive model
   * @param right: Right wheel speed
   */
  void publishCmd(double left, double right);

  /**
   * @brief Get x coordinate of simulated moving camera view for orientation control
   *
   * @param msg: Pixel Coordinates of center from simulated moving camera in reference to static camera.
   */
  void parseView(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  
  /**
   * @brief Get width of object bounding box for translation control
   *
   * @param msg: Coordinates and size of bounding box in reference to static camera.
   */
  void parseBbox(const vision_msgs::msg::BoundingBox2D msg);
  
  /**
   * @brief Controller to track object in camera view with open loop static camera mode and closed loop simulated moving camera mode
   *
   * @param msg: Coordinates of Center of Mass of tracked object. 
   */
  void objectController(const geometry_msgs::msg::Point::SharedPtr msg);
  
};
#endif /* SETPOINT_SEQUENCE_NODE_HPP_ */
