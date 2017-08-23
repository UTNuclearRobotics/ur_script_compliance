
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <ur_script_interface.h>
#include <vector>

namespace set_digital_output
{
  int set_digital_output();
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "ur_script_interface");

  set_digital_output::set_digital_output();

  ros::shutdown();
  return 0;
}


// Set a digital output of the UR arm
int set_digital_output::set_digital_output() {

  // arm compliance objects
  ur_script_interface right("/right_ur5_controller/right_ur5_URScript");
  ur_script_interface left("/left_ur5_controller/left_ur5_URScript");

  right.set_digital_output_( 2 );
  left.set_digital_output_( 2 );

  return 0;
}
