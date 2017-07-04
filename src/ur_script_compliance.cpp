
#include <ur_script_compliance.h>

// Push a lever up
int lever_up() {
  // Left arm compliance object
  ur_script_compliance left("/left_ur5_controller/left_ur5_URScript");

  // Put the robot in force mode
  left.enable_force_mode();

  // Begin motion

  return 0;
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "ur_script_compliance");
  ros::start();

  lever_up();

  ros::shutdown();
  return 0;
}

// Constructor for compliance of one arm
ur_script_compliance::ur_script_compliance(std::string ur_rostopic) :
  node_()
{
  ur_topic_pub_ = node_.advertise<std_msgs::String>(ur_rostopic, 1);
}

int ur_script_compliance::enable_force_mode()
{
  // URScript force_mode command:
  sprintf(cmd_, "force_mode([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f]," 
    "[%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f],"
    "[%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f],"
    "%1.5f,"
    "[%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f])\n", 
    0., 0., 0., 0., 0., 0.,  //task frame
    1., 1., 1., 1., 1., 1.,  // selection vector
    0., 0., 68., 0., 0., 0.,  // wrench
    2., // type
    0.1, 0.1, 0.1, 0.1, 0.1, 0.1  //limits 
  );

  ur_script_string_.data = cmd_;  // Convert from char array to ROS string
  ROS_INFO_STREAM( ur_script_string_.data );
  ur_topic_pub_.publish( ur_script_string_ );

  return 0;
}
