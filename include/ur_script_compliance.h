#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>

class ur_script_compliance
{
  // Note: most ur_script arguments are floats or ints
  public:
    // Constructor for compliance of one arm.
    // Specify the UR rostopic
    ur_script_compliance (std::string ur_rostopic);

    // Destructor automatically ends force mode
    ~ur_script_compliance(void);

    int enable_force_mode_( std::vector<float> task_frame, std::vector<int> selection_vector, std::vector<int> target_wrench, int type, std::vector<float> limits );

    int end_force_mode_();

    int set_digital_output_( int output_num );  // Useful to test the connection with robot. See the pendant I/O

    int publish_command_();

    // Units in rad
    int move_to_joints_(std::vector<float> joints);

    // This speed is in the UR "Base" frame.
    // xyz vector component of 1 => 1 cm/s in that direction
    // Does not mix with force_mode()
    int linear_speed_(std::vector<float> speed);

    // Kind of flaky. I haven't figured out what the units are
    // or even what frame it's in.
    int move_to_pose_(std::vector<float> pose);
    
    constexpr static float control_period_ = 0.008;

  private:
    ros::NodeHandle node_;
    ros::Publisher ur_topic_pub_;
    char cmd_ [400];
    std_msgs::String ur_script_string_;
};
