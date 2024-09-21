#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <cmath>
#include <chrono>

class MyNode : public rclcpp::Node
{
public:
  
  // Constructor
  MyNode() : Node("my_node") {

    //retrieveParams();

    double target_pose[5] = {200, 0, 100, 0, 0};
    std::array<double, 5> IK_solution; 

    IK_solution = solve_IK(target_pose, link_params);

    
  }

private:

  std::array<double, 3> solve_for_J3_pose(double* target_pose, double* link_params) {

    // theta4 as IK input represents angle of the fourth link wrt the xy plane
    // therefore, we can calculate position of joint 3 given this angle and some other info

    double x = target_pose[0]; double y = target_pose[1]; double z = target_pose[2]; 
    double theta4 = target_pose[3];
    double ll4 = link_params[3]; double ll5 = link_params[4];

    double dz = (ll4 + ll5) * sin(theta4);

    double theta1 = atan2(x, y); 
    double r = abs((ll4 + ll5) * cos(theta4));
    double dx  = r * cos(theta1); 
    double dy = r * sin(theta1);

    std::array<double, 3> ret_angs = {
      x - dx, y - dy, z - dz
    }
    printf("Found J3 pose (%f, %f, %f) \n", ret_angs[0], ret_angs[1], ret_angs[2]);

    return ret_angs;
  } 

  //std::array<double, 3> first_3_joint_angles(double* J3_pose) {}


  

  //std::array<double, 5> final_IK_solution(double* first_3_joint_angles, double* target_pose) {}


  std::array<double, 5> solve_IK(double* target_pose, double* link_params) {

    std::array<double, 5> solution; 
    std::array<double, 3> J3_pose = solve_for_J3_pose(target_pose, link_params);
    //std::array<double, 3> first_3_joint_angles = first_3_joint_angles(J3_pose);
    //solution = final_IK_solution(first_3_joint_angles, target_pose);

    return solution;

  }

  // class variable declaration

  // link lengths in mm
  double link_params[5] = {100, 100, 100, 100, 100};
 
};

int main(int argc, char *argv[])
{
  // Initialize the ROS 2 system
  rclcpp::init(argc, argv);

  // Create the node and spin it
  rclcpp::spin(std::make_shared<MyNode>());

  // Shutdown the ROS 2 system
  rclcpp::shutdown();
  return 0;
}
