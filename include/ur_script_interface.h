
#ifndef UR_SCRIPT_INTERFACE_H
#define UR_SCRIPT_INTERFACE_H

#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <vector>

class ur_script_interface
{
  // Note: most ur_script arguments are floats or ints
  public:
    // Constructor
    // Specify the rostopic where UR commands get published.
    ur_script_interface (std::string ur_rostopic);

    // Destructor automatically ends force mode (if it was enabled)
    ~ur_script_interface(void);

    int enable_force_mode_( std::vector<float> task_frame, std::vector<int> selection_vector, std::vector<int> target_wrench, int type, std::vector<float> limits );

    int end_force_mode_();

    // Listen to ROS /joy commands and jog joints
    int jog_joints_(std::string incoming_cmd_topic);

    // Callback for incoming joystick commands
    void joystick_cb_(const sensor_msgs::Joy::ConstPtr& msg);

    // This speed is in the UR "Base" frame.
    // xyz vector component of 1 => 1 cm/s in that direction
    // Does not mix with force_mode()
    int linear_speed_(std::vector<float> speed);

    // Units in rad
    int move_to_joints_(std::vector<float> joints);

    // Kind of flaky. I haven't figured out what the units are
    // or even what frame it's in.
    int move_to_pose_(std::vector<float> pose);

    int publish_command_();

    int set_digital_output_( int output_num );  // Useful to test the connection with robot. See the pendant I/O
    
    constexpr static float control_period_ = 0.01;

  private:
    ros::NodeHandle node_;
    ros::Publisher ur_topic_pub_;
    char cmd_ [400];
    std_msgs::String ur_script_string_;
    geometry_msgs::Vector3 position_;
    geometry_msgs::Vector3 orientation_;
};

#endif
