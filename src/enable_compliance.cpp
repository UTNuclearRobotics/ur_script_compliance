
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <ur_script_interface.h>
#include <vector>

namespace enable_compliance
{
  int enable_force_mode();
}
using namespace enable_compliance;


int main(int argc, char** argv) {
  ros::init(argc, argv, "ur_script_interface");

  enable_force_mode();

  ros::shutdown();
  return 0;
}


// Enable force mode
int enable_compliance::enable_force_mode() {

  // arm compliance objects
  ur_script_interface right("/right_ur5_controller/right_ur5_URScript");
  ur_script_interface left("/left_ur5_controller/left_ur5_URScript");

  // Put the robot in force mode
  // This will be compliant in all directions with a target wrench of 0
  std::vector<float> force_frame {0., 0., 0., 0., 0., 0.};  // A pose
  std::vector<int> selection_vector {1, 1, 1, 1, 1, 1};  // Compliant in all directions
  std::vector<int> target_wrench {0, 0, 0, 0, 0, 0};
  int type = 1;  // Force frame transform
  std::vector<float> limits {0.8, 0.8, 0.8, 1.571, 1.571, 1.571};  // Displacement limits

  right.enable_force_mode_( force_frame, selection_vector, target_wrench, type, limits );
  left.enable_force_mode_( force_frame, selection_vector, target_wrench, type, limits );

  while ( ros::ok() )
  {
  	ros::spinOnce();
  	ros::Duration(ur_script_interface::control_period_).sleep();
  }

  return 0;
}
