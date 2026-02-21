#include <rclcpp/rclcpp.hpp>
#include "example_interfaces/msg/float64.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <cmath>

class SetpointSequenceNode : public rclcpp::Node
{
public:
    SetpointSequenceNode()
    : Node("setpoint_sequence_node")
    {
        // Declare setpoints as parameters, for future use
        this->declare_parameter("setpoints", true);
        this->declare_parameter("setpoints_array", std::vector<double>{});
        // Controller parameters (Proportional)
        this->declare_parameter("kp_linear", 10.0);
        this->declare_parameter("kp_angular", 50.0);
        // Parameter needed for differential drive model
        this->declare_parameter("wheel_base", 0.209); //from relbot_simulator.hpp
        // Object tracking parameters
        this->declare_parameter("mode", true); //true, for /moving_camera (1.2.3), false, for /image (1.2.2) [must also change remap]
        this->declare_parameter("width", 360); // when zooming in or out it will become unnaccurate but relbot_simulator does not output dimension of moving_camera
        this->declare_parameter("desired_width", 100.0); //100.0 for 1.2.2 because of larger FOV.

        //Callback for parameter set during runtime
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&SetpointSequenceNode::parameters_callback, this, std::placeholders::_1)
        );

        //Publishers of velocities for relbol_simulator
        left_pub_ = this->create_publisher<example_interfaces::msg::Float64>(
            "/input/left_motor/setpoint_vel", 10);
        right_pub_ = this->create_publisher<example_interfaces::msg::Float64>(
            "/input/right_motor/setpoint_vel", 10);
            
        //Publishers of coordinates for plotting
        target_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
            "/target", 10);

        //Subscriber to get current position
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/output/robot_pose",
            10,
            std::bind(&SetpointSequenceNode::pose_callback, this, std::placeholders::_1));
        
        //Subscribe to get Point for object tracking
        object_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/tracked_CoM", 
            10,
            std::bind(&SetpointSequenceNode::object_controller, this, std::placeholders::_1));
            
        //Subscribe to get Point for object tracking
        view_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/output/camera_position", 
            10,
            std::bind(&SetpointSequenceNode::parse_view, this, std::placeholders::_1));
            
        //Subscribe to get BoundingBox for object tracking
        bbox_sub_ = this->create_subscription<vision_msgs::msg::BoundingBox2D>(
            "/tracked_bbox", 
            10,
            std::bind(&SetpointSequenceNode::parse_bbox, this, std::placeholders::_1));

        //Timer for controller
        //Declare time step
        sample_time_s_ = 0.03;
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(sample_time_s_),
            std::bind(&SetpointSequenceNode::timer_callback, this));

        //Get parameters initial values
        setpoints_enabled_ = this->get_parameter("setpoints").as_bool();
        setpoints_array_ = this->get_parameter("setpoints_array").as_double_array();
        kp_linear_ = this->get_parameter("kp_linear").as_double();
        kp_angular_ = this->get_parameter("kp_angular").as_double();
        wheel_base_ = this->get_parameter("wheel_base").as_double();
        mode_ = this->get_parameter("mode").as_bool();
        width_ = this->get_parameter("width").as_int();
        desired_width_ = this->get_parameter("desired_width").as_double();
        // Parse initial sequence (YAML)
        parse_sequence();

        RCLCPP_INFO(this->get_logger(), "Setpoint sequence controller started");
    }

private:

    //parse setpoint array
    void parse_sequence()
    {
        //empty sequence (for param set during runtime)
        sequence_.clear();
        durations_.clear();

        //Check that array is 4xN [duration, x, y, z ]
        if (setpoints_array_.size() % 4 != 0) {
            RCLCPP_ERROR(this->get_logger(),
                         "setpoints_array must contain groups of 4: [duration, x, y, z]");
            return;
        }

        //turn the squished 1x4N double array into 1xN Point array
        for (size_t i = 0; i < setpoints_array_.size(); i += 4)
        {
            durations_.push_back(setpoints_array_[i]); //concatenate at end of array

            //Format as geometry_msgs/msg/Point
            geometry_msgs::msg::Point p;
            p.x = setpoints_array_[i + 1];
            p.y = setpoints_array_[i + 2];
            p.z = setpoints_array_[i + 3];

            sequence_.push_back(p); //concatenate at end of array
        }

        // Reset execution state
        seq_index_ = 0;
        elapsed_ = 0.0;
    }

    //Callback for changes in parameters
    rcl_interfaces::msg::SetParametersResult
    parameters_callback(const std::vector<rclcpp::Parameter> & params)
    {
        for (const auto & param : params)
        {
            if (param.get_name() == "kp_linear") {
                kp_linear_ = param.as_double();
            }
            else if (param.get_name() == "kp_angular") {
                kp_angular_ = param.as_double();
            }
            else if (param.get_name() == "wheel_base") {
                wheel_base_ = param.as_double();
            }
            else if (param.get_name() == "setpoints") {
                setpoints_enabled_ = param.as_bool();
            }
            else if (param.get_name() == "mode") {
                setpoints_enabled_ = param.as_bool();
            }
            else if (param.get_name() == "width") {
                setpoints_enabled_ = param.as_int();
            }
            else if (param.get_name() == "desired_width") {
                setpoints_enabled_ = param.as_double();
            }
            else if (param.get_name() == "setpoints_array") {
                setpoints_array_  = param.as_double_array();
                parse_sequence();

                RCLCPP_INFO(this->get_logger(), "New setpoint sequence loaded");
            }
        }

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    //parse pose
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_x_ = msg->pose.position.x;
        current_y_ = msg->pose.position.y;
        current_theta_ = msg->pose.orientation.z; //Pose should be quaternion, but only z is used for theta_z and the rest are static
    }

    //main periodic process
    void timer_callback()
    {
        //if not using setpoints or finished sequence, send 0,0
        if (!setpoints_enabled_) {
            return;
        } else {
          if (seq_index_ < sequence_.size()) {
            sequence_controller();
          } else {
            publish_cmd(0.0, 0.0);
          }
        }
    }

    //Proportional controller for sequences (2D xy coordinate points)
    void sequence_controller()
    {
        //get current setpoint
        auto target = sequence_[seq_index_];
        double duration = durations_[seq_index_];

        //calculate error
        double dx = target.x - current_x_;
        double dy = target.y - current_y_;
        double distance_error = std::sqrt(dx * dx + dy * dy); //euclidean distance
        double angle_to_goal  = std::atan2(dy, dx);
        double heading_error  = angle_to_goal - current_theta_;

        //bound angle to (-pi,pi)
        while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
        while (heading_error < -M_PI) heading_error += 2.0 * M_PI;

        //calculate velocities
        double v     = kp_linear_ * distance_error;
        double omega = kp_angular_ * heading_error;
        
        //convert to differential drive velocities
        double v_left  = v - (wheel_base_ / 2.0) * omega;
        double v_right = v + (wheel_base_ / 2.0) * omega;

        //publish velocities
        publish_cmd(v_left, v_right);
        
        //publish target (for plotting)
        target_pub_->publish(target);

        elapsed_ += 0.03;

        //check if time for next setpoint
        if (elapsed_ >= duration) {
            seq_index_++;
            elapsed_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "Switching to setpoint %ld", seq_index_);
        }
    }

    //send to relbot_simulator
    void publish_cmd(double left, double right)
    {
        example_interfaces::msg::Float64 lmsg; //convert to example_interfaces/msgs/Float64 which is format used by relbot_simulator
        example_interfaces::msg::Float64 rmsg;

        lmsg.data = -left; // inverted in relbot_simulator
        rmsg.data = right;

        left_pub_->publish(lmsg);
        right_pub_->publish(rmsg);
    }
    
    void parse_view(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        view_x_  = msg->point.x;
    }
    
    void parse_bbox(const vision_msgs::msg::BoundingBox2D msg)
    {
        bbox_width_  = msg.size_x;
    }
    
    void object_controller(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        //dont run if following setpoints
        if (setpoints_enabled_) {
          return;
        }
        
        double object_x = msg->x;
        if (object_x == -1.0){ //check for no object detected
          publish_cmd(0.0, 0.0); //safety stop
          return;
        }
        
        double error_x;

        // --- Image errors ---
        if (mode_){
          error_x = object_x - (width_/2.0); //orientation error for 1.2.3
        } else {
          error_x = object_x - view_x_; //orientation error for 1.2.2
        }
        
        double error_size = bbox_width_ - desired_width_; //size error
        
        double v = kp_linear_/200.0 * error_size;
        double omega = kp_angular_/200.0 * error_x;

        // --- Differential drive mapping ---
        double v_left  = v - (wheel_base_ / 2.0) * omega;
        double v_right = v + (wheel_base_ / 2.0) * omega;
        
        //publish velocities
        publish_cmd(-v_left, -v_right); //negative because mirrored camera
    }

    //Establish variables
    //ROS interfaces
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr left_pub_;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr right_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr object_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr view_sub_;
    rclcpp::Subscription<vision_msgs::msg::BoundingBox2D>::SharedPtr bbox_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    //Time step 
    double sample_time_s_;
    //Sequence variables
    std::vector<double> setpoints_array_;
    std::vector<geometry_msgs::msg::Point> sequence_;
    std::vector<double> durations_;
    bool setpoints_enabled_;
    size_t seq_index_ = 0;
    double elapsed_ = 0.0;
    //Controller Parameters
    double kp_linear_;
    double kp_angular_;
    //Differential Drive Parameters
    double wheel_base_;
    //Object tracking Parameters
    bool mode_;
    int width_;
    double bbox_width_;
    double desired_width_;
    double view_x_ = 0;
    //State (coordinate) variables
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_theta_ = 0.0;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SetpointSequenceNode>());
    rclcpp::shutdown();
    return 0;
}
