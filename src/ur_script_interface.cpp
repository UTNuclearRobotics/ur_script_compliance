
#include <ur_script_interface.h>

// Constructor
ur_script_interface::ur_script_interface (std::string ur_rostopic) :
  node_()
{
  ur_topic_pub_ = node_.advertise<std_msgs::String>(ur_rostopic, 1);
  ros::Duration(0.5).sleep();
}

// Destructor -- it ends force_mode
ur_script_interface::~ur_script_interface(void)
{
  ur_script_interface::end_force_mode_();
}

int ur_script_interface::enable_force_mode_( std::vector<float> task_frame, std::vector<int> selection_vector, std::vector<int> target_wrench, int type, std::vector<float> limits )
{

  // URScript force_mode command:
  sprintf(cmd_, "force_mode(p[%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f]," 
    "[%d, %d, %d, %d, %d, %d],"
    "[%d, %d, %d, %d, %d, %d],"
    "%d,"
    "[%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f])\n", 
    task_frame.at(0), task_frame.at(1), task_frame.at(2), task_frame.at(3), task_frame.at(4), task_frame.at(5),  //task frame. Note the leading 'p' for 'pose'
    selection_vector.at(0), selection_vector.at(1), selection_vector.at(2), selection_vector.at(3), selection_vector.at(4), selection_vector.at(5),  // selection vector -- does each axis act like compliance or admittance?
    target_wrench.at(0), target_wrench.at(1), target_wrench.at(2), target_wrench.at(3), target_wrench.at(4), target_wrench.at(5),  // target wrench
    type, // type of force_frame transform
    limits.at(0), limits.at(1), limits.at(2), limits.at(3), limits.at(4), limits.at(5)  // Displacement limits
  ); 

  publish_command_();

  return 0;
}


int ur_script_interface::end_force_mode_()
{
  sprintf(cmd_, "end_force_mode()\n");
  publish_command_();

  return 0;
}


// Listen to ROS /joy commands and jog joints
int ur_script_interface::jog_joints_(std::string incoming_cmd_topic)
{
  // Subscribe to /joy topic
  ros::Subscriber sub_pose_cmd = node_.subscribe(incoming_cmd_topic, 1,  &ur_script_interface::joystick_cb_, this);

  // LOOP:
  // Can be interrupted by the action server.
  while ( ros::ok() )
  {
    // Convert joy commands to Cartesian speeds

    // Publish the speed cmd to the robot

    ros::Duration(control_period_).sleep();
  }

  return 0;
}


// Process incoming joystick commands, e.g. from SpaceMouse
void ur_script_interface::joystick_cb_(const sensor_msgs::Joy::ConstPtr& msg)
{
  position_.x = msg->axes[0];
  position_.y = msg->axes[1];
  position_.z = msg->axes[2];

  orientation_.x = msg->axes[3];  // roll
  orientation_.y = msg->axes[4];  // pitch
  orientation_.z = msg->axes[5];  // yaw

  return;
}


// This speed is in the UR "Base" frame.
// xyz vector component of 1 => 1 cm/s in that direction
int ur_script_interface::linear_speed_(std::vector<float> speed)
{
  sprintf(cmd_, "speedl([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], a=0.1, v=0.01)\n", speed.at(0), speed.at(1), speed.at(2), speed.at(3), speed.at(4), speed.at(5) );

  while( ros::ok() )
  {
    publish_command_();
    ros::Duration( 10*control_period_ ).sleep();
  }

  return 0;
}


// Angles in rad
int ur_script_interface::move_to_joints_(std::vector<float> joints)
{
	sprintf(cmd_, "movej([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], a=0.1, v=0.1)\n", joints.at(0), joints.at(1), joints.at(2), joints.at(3), joints.at(4), joints.at(5) );
    publish_command_();

	return 0;
}


int ur_script_interface::move_to_pose_(std::vector<float> pose)
{
  sprintf(cmd_, "movel(p[%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], a=0.1, v=0.01)\n", pose.at(0), pose.at(1), pose.at(2), pose.at(3), pose.at(4), pose.at(5) );
  publish_command_();

  return 0;
}


int ur_script_interface::publish_command_()
{
  ur_script_string_.data = cmd_;  // Convert from char array to ROS string
  //ROS_INFO_STREAM( ur_script_string_.data );
  ur_topic_pub_.publish( ur_script_string_ );

  return 0;
}


// Set a digital output. Useful for testing the connection.
// Check the pendant's IO monitor to see if it worked.
int ur_script_interface::set_digital_output_( int output_num )
{
    sprintf(cmd_, "set_digital_out(%d,True)\n", output_num );
    publish_command_();

    return 0;
}
