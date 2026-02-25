//==============================================================================
// Authors : Max Solis, Aleksei Obshatko
// Group : ASDfR 5
// License : LGPL open source license
//
// Brief : Node that controls the relbot based on, coordinate setpoints or
// tracking an object.
//==============================================================================

#include "setpoint_sequence_node.hpp"

SetpointSequenceNode::SetpointSequenceNode() : Node("setpoint_sequence_node") {
  // Declare setpoints as parameters
  this->declare_parameter("setpoints", true);  // use setpoints or track object
  this->declare_parameter("setpoints_array",
                          std::vector<double>{});  // setpoint coordinates
  // Controller parameters (Proportional)
  this->declare_parameter(
      "kp_linear", 10.0);  // proportional controller gain for translation
  this->declare_parameter("kp_angular",
                          50.0);  // proportional controller gain for rotation
  // Parameter needed for differential drive model
  this->declare_parameter("wheel_base", 0.209);  // from relbot_simulator.hpp
  // Object tracking parameters
  this->declare_parameter("mode",
                          true);  // true, for /moving_camera (1.2.3), false,
                                  // for /image (1.2.2) [must also change remap]
  this->declare_parameter(
      "width",
      360);  // when zooming in or out it will become unnaccurate but
             // relbot_simulator does not output dimension of moving_camera
  this->declare_parameter("desired_width",
                          100.0);  // 100.0 for 1.2.2 because of larger FOV.

  // Callback for parameter set during runtime
  paramCallbackHandle_ = this->add_on_set_parameters_callback(std::bind(
      &SetpointSequenceNode::parametersCallback, this, std::placeholders::_1));

  // Publishers of velocities for relbol_simulator
  leftPub_ = this->create_publisher<example_interfaces::msg::Float64>(
      "/input/left_motor/setpoint_vel", 10);
  rightPub_ = this->create_publisher<example_interfaces::msg::Float64>(
      "/input/right_motor/setpoint_vel", 10);
  // Publishers of coordinates for plotting
  targetPub_ = this->create_publisher<geometry_msgs::msg::Point>("/target", 10);

  // Subscriber to get current position
  poseSub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/output/robot_pose", 10,
      std::bind(&SetpointSequenceNode::poseCallback, this,
                std::placeholders::_1));
  // Subscribe to get Point for object tracking
  objectSub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/tracked_CoM", 10,
      std::bind(&SetpointSequenceNode::objectController, this,
                std::placeholders::_1));
  // Subscribe to get Point for object tracking
  viewSub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/output/camera_position", 10,
      std::bind(&SetpointSequenceNode::parseView, this, std::placeholders::_1));
  // Subscribe to get BoundingBox for object tracking
  bboxSub_ = this->create_subscription<vision_msgs::msg::BoundingBox2D>(
      "/tracked_bbox", 10,
      std::bind(&SetpointSequenceNode::parseBbox, this, std::placeholders::_1));

  // Timer for controller
  // Declare time step
  sampleTimeS_ = 0.03;  // run sequence controller every 30 miliseconds
  timer_ = this->create_wall_timer(
      std::chrono::duration<double>(sampleTimeS_),
      std::bind(&SetpointSequenceNode::timerCallback, this));

  // Get parameters initial values
  setpointsEnabled_ = this->get_parameter("setpoints").as_bool();
  setpointsArray_ = this->get_parameter("setpoints_array").as_double_array();
  kpLinear_ = this->get_parameter("kp_linear").as_double();
  kpAngular_ = this->get_parameter("kp_angular").as_double();
  wheelBase_ = this->get_parameter("wheel_base").as_double();
  mode_ = this->get_parameter("mode").as_bool();
  width_ = this->get_parameter("width").as_int();
  desiredWidth_ = this->get_parameter("desired_width").as_double();
  // Parse initial sequence (YAML)
  parseSequence();

  RCLCPP_INFO(this->get_logger(), "Setpoint sequence controller started");
}

// parse setpoint array
void SetpointSequenceNode::parseSequence() {
  // empty sequence (for param set during runtime)
  sequence_.clear();
  durations_.clear();

  // Check that array is 4xN [duration, x, y, z ]
  if (setpointsArray_.size() % 4 != 0) {
    RCLCPP_ERROR(
        this->get_logger(),
        "setpoints_array must contain groups of 4: [duration, x, y, z]");
    return;
  }

  // turn the squished 1x4N double array into 1xN Point array
  for (size_t i = 0; i < setpointsArray_.size(); i += 4) {
    durations_.push_back(setpointsArray_[i]);  // concatenate at end of array

    // Format as geometry_msgs/msg/Point
    geometry_msgs::msg::Point p;
    p.x = setpointsArray_[i + 1];
    p.y = setpointsArray_[i + 2];
    p.z = setpointsArray_[i + 3];

    sequence_.push_back(p);  // concatenate at end of array
  }

  // Reset execution state
  seqIndex_ = 0;
  elapsed_ = 0.0;
}

// Callback for changes in parameters
rcl_interfaces::msg::SetParametersResult
SetpointSequenceNode::parametersCallback(
    const std::vector<rclcpp::Parameter>& params) {
  for (const auto& param : params) {
    if (param.get_name() == "kp_linear") {
      kpLinear_ = param.as_double();
    } else if (param.get_name() == "kp_angular") {
      kpAngular_ = param.as_double();
    } else if (param.get_name() == "wheel_base") {
      wheelBase_ = param.as_double();
    } else if (param.get_name() == "setpoints") {
      setpointsEnabled_ = param.as_bool();
    } else if (param.get_name() == "mode") {
      setpointsEnabled_ = param.as_bool();
    } else if (param.get_name() == "width") {
      setpointsEnabled_ = param.as_int();
    } else if (param.get_name() == "desired_width") {
      setpointsEnabled_ = param.as_double();
    } else if (param.get_name() == "setpoints_array") {
      setpointsArray_ = param.as_double_array();
      parseSequence();

      RCLCPP_INFO(this->get_logger(), "New setpoint sequence loaded");
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

// parse pose
void SetpointSequenceNode::poseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  currentX_ = msg->pose.position.x;
  currentY_ = msg->pose.position.y;
  currentTheta_ =
      msg->pose.orientation.z;  // Pose should be quaternion, but only z is used
                                // for theta_z and the rest are static
}

// main periodic process for sequences
void SetpointSequenceNode::timerCallback() {
  // if not using setpoints or finished sequence, send 0,0
  if (!setpointsEnabled_) {
    return;
  } else {
    if (seqIndex_ < sequence_.size()) {
      sequenceController();
    } else {
      publishCmd(0.0, 0.0);
    }
  }
}

// Proportional controller for sequences (2D xy coordinate points)
void SetpointSequenceNode::sequenceController() {
  // get current setpoint
  auto target = sequence_[seqIndex_];
  double duration = durations_[seqIndex_];

  // calculate error
  double dx = target.x - currentX_;
  double dy = target.y - currentY_;
  double distanceError = std::sqrt(dx * dx + dy * dy);  // euclidean distance
  double angleToGoal = std::atan2(dy, dx);
  double headingError = angleToGoal - currentTheta_;

  // bound angle to (-pi,pi)
  while (headingError > M_PI) headingError -= 2.0 * M_PI;
  while (headingError < -M_PI) headingError += 2.0 * M_PI;

  // calculate velocities
  double v = kpLinear_ * distanceError;
  double omega = kpAngular_ * headingError;

  // convert to differential drive velocities
  double vLeft = v - (wheelBase_ / 2.0) * omega;
  double vRight = v + (wheelBase_ / 2.0) * omega;

  // publish velocities
  publishCmd(vLeft, vRight);

  // publish target (for plotting)
  targetPub_->publish(target);

  elapsed_ += sampleTimeS_;

  // check if time for next setpoint
  if (elapsed_ >= duration) {
    seqIndex_++;
    elapsed_ = 0.0;
    RCLCPP_INFO(this->get_logger(), "Switching to setpoint %ld", seqIndex_);
  }
}

// send to relbot_simulator
void SetpointSequenceNode::publishCmd(double left, double right) {
  example_interfaces::msg::Float64
      lmsg;  // convert to example_interfaces/msgs/Float64 which is format used
             // by relbot_simulator
  example_interfaces::msg::Float64 rmsg;

  // Some safety for the motor speeds
  if (abs(left) > 5.0 || (right) > 5.0) {
    // get biggest number
    double max_value =
        std::max(abs(left), abs(right));  // this needs absolute values in
                                          // relbot_adapter (really guys?)
    // find scaling value
    double scaling_factor = 5.0 / max_value;  // magic number

    // rescale BOTH components to keep same directionality
    left = left * scaling_factor;
    right = right * scaling_factor;
  }

  lmsg.data = -left;  // inverted in relbot_simulator
  rmsg.data = right;

  leftPub_->publish(lmsg);
  rightPub_->publish(rmsg);
}

void SetpointSequenceNode::parseView(
    const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  viewX_ = msg->point.x;
}

void SetpointSequenceNode::parseBbox(
    const vision_msgs::msg::BoundingBox2D msg) {
  bboxWidth_ = msg.size_x;
}

void SetpointSequenceNode::objectController(
    const geometry_msgs::msg::Point::SharedPtr msg) {
  // dont run if following setpoints
  if (setpointsEnabled_) {
    return;
  }

  double objectX = msg->x;
  if (objectX == -1.0) {   // check for no object detected
    publishCmd(0.0, 0.0);  // safety stop
    return;
  }

  double errorX;

  // Image errors
  if (mode_) {
    errorX =
        objectX - (width_ / 2.0);  // orientation error for closed loop 1.2.3
  } else {
    errorX = objectX - viewX_;  // orientation error for open loop 1.2.2
  }
  double errorSize = bboxWidth_ - desiredWidth_;  // size error
  double v = kpLinear_ / 200.0 *
             errorSize;  // smaller gain because of increased frequency
  double omega = kpAngular_ / 200.0 *
                 errorX;  // smaller gain because of increased frequency

  // Differential drive mapping
  double vLeft = v - (wheelBase_ / 2.0) * omega;
  double vRight = v + (wheelBase_ / 2.0) * omega;

  // publish velocities
  publishCmd(-vLeft, -vRight);  // negative because mirrored camera
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SetpointSequenceNode>());
  rclcpp::shutdown();
  return 0;
}
