
#include <ur_script_compliance.h>


int main(int argc, char** argv) {
  ros::init(argc, argv, "ur_script_compliance");
  ros::start();

  both_arms_up_();

  ros::shutdown();
  return 0;
}


// Push up with both arms
int both_arms_up_() {
  // right arm compliance object
  ur_script_compliance right("/right_ur5_controller/right_ur5_URScript");

  // Put the robot in force mode
  right.enable_force_mode_();

  // Begin motion
  std::vector<float> joints { -1.017, -1.556, -1.142, -0.4117, 1.099, 0.7076};
  right.move_to_joints_( joints );

  // Back into normal mode
  //right.end_force_mode_();

  return 0;
}


// Constructor for compliance of one arm
ur_script_compliance::ur_script_compliance (std::string ur_rostopic) :
  node_()
{
  ur_topic_pub_ = node_.advertise<std_msgs::String>(ur_rostopic, 1);
  ros::Duration(0.5).sleep();
}


int ur_script_compliance::enable_force_mode_()
{

  // URScript force_mode command:
  sprintf(cmd_, "force_mode(p[%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f]," 
    "[%d, %d, %d, %d, %d, %d],"
    "[%d, %d, %d, %d, %d, %d],"
    "%d,"
    "[%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f])\n", 
    0., 0., 0., 0., 0., 0.,  //task frame. Note the leading 'p' for 'pose'
    1, 1, 1, 1, 1, 1,  // selection vector -- does each axis act like compliance or admittance?
    0, 0, 0, 0, 0, 0,  // target wrench
    1, // type
    0.8, 0.8, 0.8, 1.571, 1.571, 1.571  // adjustment limits
  ); 

  publish_command_();

  return 0;
}


int ur_script_compliance::end_force_mode_()
{
  sprintf(cmd_, "end_force_mode()\n");
  publish_command_();

  return 0;
}


// Angles in rad
int ur_script_compliance::move_to_joints_(std::vector<float> joints)
{
	sprintf(cmd_, "movej([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], a=0.1, v=0.1)\n", joints.at(0), joints.at(1), joints.at(2), joints.at(3), joints.at(4), joints.at(5) );
    publish_command_();

	return 0;
}


// This speed is in the UR "Base" frame.
// xyz vector component of 1 => 1 cm/s in that direction
int ur_script_compliance::linear_speed_(std::vector<float> speed)
{
  sprintf(cmd_, "speedl([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], a=0.1, v=0.01)\n", speed.at(0), speed.at(1), speed.at(2), speed.at(3), speed.at(4), speed.at(5) );

  while( ros::ok() )
  {
    publish_command_();
    ros::Duration( 10*control_period_ ).sleep();
  }

  return 0;
}


int ur_script_compliance::move_to_pose_(std::vector<float> pose)
{
  sprintf(cmd_, "movel(p[%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], a=0.1, v=0.01)\n", pose.at(0), pose.at(1), pose.at(2), pose.at(3), pose.at(4), pose.at(5) );
  publish_command_();

  return 0;
}


int ur_script_compliance::publish_command_()
{
  ur_script_string_.data = cmd_;  // Convert from char array to ROS string
  //ROS_INFO_STREAM( ur_script_string_.data );
  ur_topic_pub_.publish( ur_script_string_ );
  ros::Duration(0.1).sleep();

  return 0;
}


// Set a digital output. Useful for testing the connection.
// Check the pendant's IO monitor to see if it worked.
int ur_script_compliance::set_digital_output_()
{
    sprintf(cmd_, "set_digital_out(2,True)\n");
    publish_command_();

    return 0;
}