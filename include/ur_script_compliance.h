#include <ros/ros.h>
#include <std_msgs/String.h>

class ur_script_compliance
{
  public:
    ur_script_compliance(std::string ur_rostopic);
    int enable_force_mode();

  private:
    ros::NodeHandle node_;
    ros::Publisher ur_topic_pub_;
    char cmd_ [400];
    std_msgs::String ur_script_string_;
};
