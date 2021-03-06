#ifndef __LIB_MOVEMENT_H__
#define __LIB_MOVEMENT_H__

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>
#include <Eigen/Core>
#include <boost/timer.hpp>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryActionFeedback.h>
#include <actionlib/client/simple_action_client.h>
#include <crustcrawler_core_msgs/EndpointState.h>
#include <crustcrawler_core_msgs/JointCommand.h>
#include <crustcrawler_core_msgs/EndEffectorCommand.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene/planning_scene.h>

#include "../../src/parameters.hpp"


/**
 * @brief Execute a trajectory given as joint trajectory using joint action server
 * @param action client to joint action server,
 * @param joint trajectory_msgs, and
 * @param Data_config class
 * @return true if the trajectory is successful, false otherwise
**/
bool execute_joint_trajectory(actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac,
                              trajectory_msgs::JointTrajectory& joint_trajectory,
                              Data_config& parameters,
                              ros::Publisher& gripper_pub);

/**
 * @brief Guide the arm to the initial position of the trajectory to be executed later, using joint action server
 * @param Data_config class, and
 * @param action client to joint action server
 * @return true if guiding of the arm is successful, false otherwise
**/
bool go_to_initial_position(Data_config& parameters,
                            actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> &ac,
                            ros::Publisher& gripper_pub);

/**
 * @brief Execute a trajectory given as joint trajectory using joint action server
 * @param action client to joint action server
 * @param joint trajectory_msgs
 * @return true if the guiding of the arm is successful, false otherwise
**/
void record_feedback(Data_config &parameters,
                     control_msgs::FollowJointTrajectoryActionFeedback& feedback,
                     std::ofstream& output_file);

/**
 * @brief Check if all joint trajectory points are valid (in terms of self collision)
 * @param Data_config class
 * @return true if trajectory is valid, false otherwise
**/
bool is_trajectory_valid(Data_config& parameters);

/**
 * @brief Extract joint values from the sensor_msgs::JointState using the correct arm choise set in parameters
 * @param Data_config class
 * @return vector of joint values and fill in the parameters the corresponding joint values
**/
std::vector<double>& extract_arm_joints_values(Data_config& parameters);

/**
 * @brief Same as extract_arm_joints_values() but user can specify the arm here
 * @param Data_config class, and
 * @param arm choice (left, or right)
 * @return vector of joint values and fill in the parameters the corresponding joint values
**/
std::vector<double>& extract_certain_arm_joints_values(Data_config& parameters);

/**
 * @brief Extract from joint command msgs the values arranged from joint_1 to joint_6
 * @param Data_config class, and
 * @return vector of arranged joint values and fill in the parameters the corresponding joint values
**/
std::vector<double>& get_arranged_joint_command(Data_config& parameters);

/**
 * @brief Extract from each joint command msgs the values arranged from joint_1 to joint_6 and return the vector
 * @param Data_config class, and
 * @return vector of arranged joint values and fill in the parameters the corresponding joint values
**/
std::vector<double>& get_arranged_individual_joint_command(Data_config& parameters);

/**
 * @brief Read a text file that include joint waypoints and construct the joint trajectory_msgs
 * @param text file that includes joints waypoints
 * @param Data_config class
 * @return Nothing but fill in the parameters the joint trajectory_msgs to be followed
**/
void record_arm_joint_trajectory(std::ofstream& output_file, Data_config& parameters);

/**
 * @brief largest_difference
 * @param first
 * @param second
 * @return double that represents the largest difference between the two vectors
 */
double largest_difference(std::vector<double> &first, std::vector<double> &second);

/**
 * @brief Read a vector that includes joint waypoints and construct the joint trajectory_msgs
 * @param the trajectory to be filled
 * @param the vector of waypoints
 * @param double that represents the delta time between points
 * @return Nothing but fill in the parameters the joint trajectory_msgs to be followed
**/
void construct_joint_trajectory_from_vector(trajectory_msgs::JointTrajectory& my_joint_trajectory,
                                            std::vector<std::vector<double> >& raw_joint_traj,
                                            double& dt,
                                            bool& velocity_option,
                                            bool& acceleration_option);

/**
 * @brief Read a text file that includes joint waypoints and construct the joint trajectory_msgs
 * @param text file that includes joints waypoints
 * @param Data_config class
 * @return Nothing but fill in the parameters the joint trajectory_msgs to be followed
**/
void construct_joint_trajectory_from_file(std::ifstream &text_file, Data_config& parameters);

/**
 * @brief Use the passed action server to execute the passed joint trajectory_msgs
 * @param Joint action server
 * @param Joint trajectory_msgs
 * @return Boolean that shows if the action server was able to execute the trajectory successfully or not
**/
bool move_with_action_server(actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac,
                             trajectory_msgs::JointTrajectory& my_joint_trajectory);

/**
 * @brief Return a predefined joint target that is sought to be safe enough for next motions
 * @param Nothing
 * @return Joint trajectory_msgs that contains one point
**/
trajectory_msgs::JointTrajectoryPoint get_neutral_point();

/**
 * @brief In simulation (i.e. Gazebo) it returns the position of the ball to be picked
 * @param Data_config class
 * @return geometry_msgs::Pose type that contains the ball position relative to the base (or world) frame
**/
geometry_msgs::Pose get_ball_pose(Data_config& parameters);

/**
 * @brief Get the pose of the link defined as the end effector of the robot_model
 * @param geometry_msgs::Pose that includes the end effector pose from a the topic /crustcrawler/endpoint_state
 * @param Data_config class
 * @return Nothing but fill in the parameters the end effector pose
**/
void locate_eef_pose(geometry_msgs::Pose eef_feedback, Data_config& parameters);

/**
 * @brief Along with get_ball_pose() function it returns the solution for the picking point as joint trajectory_msgs
 * @param Data_config class
 * @return Joint trajectory_msgs that contains one point (i.e. picking joint configurations
**/
trajectory_msgs::JointTrajectoryPoint get_grapping_point(Data_config& parameters);

/**
 * @brief Construction a joint trajectory_msgs that contains two points, starting point is
 *  the current joint states and the target is second_pt
 * @param Data_config class
 * @param Joint trajectory_msgs that will hold the resulting trajectory_msgs
 * @param trajectory_msgs::JointTrajectoryPoint which is the target second point to be reached
 * @return Nothing but fill in the trajectory_msgs::JointTrajectory correspondily
**/
void construct_two_points_trajectory(Data_config& parameters,
                                     trajectory_msgs::JointTrajectory& my_joint_trajectory,
                                     trajectory_msgs::JointTrajectoryPoint second_pt);

/**
 * @brief Construction a joint trajectory_msgs that contains multiple points, starting point is
 * the current joint states and the target is second_pt. It uses polynomial interpolation to
 * smooth the trajectory between the two points
 * @param Data_config class
 * @param Joint trajectory_msgs that will hold the resulting trajectory_msgs
 * @param trajectory_msgs::JointTrajectoryPoint which is the target second point to be reached
 * @return Nothing but fill in the trajectory_msgs::JointTrajectory correspondily
**/
void construct_safe_initial_point_trajectory(Data_config& parameters,
                                             trajectory_msgs::JointTrajectory& my_joint_trajectory,
                                             trajectory_msgs::JointTrajectoryPoint second_pt);

/**
 * @brief Open the gripper of the robot_model
 * @param Data_config class
 * @param A publisher that publish to the right topic (/crustcrawler/end_effector/gripper/command)
 * @return Nothing just open the gripper
**/
void open_gripper(Data_config& parameters, ros::Publisher& gripper_pub);

/**
 * @brief Close the gripper of the robot_model
 * @param Data_config class
 * @param A publisher that publish to the right topic (/crustcrawler/end_effector/gripper/command)
 * @return Nothing just close the gripper
**/
void close_gripper(Data_config& parameters, ros::Publisher& gripper_pub);

/**
 * @brief Delete the name model from the simulated environment
 * @param Model name
 * @param Data_config class
 * @return Boolean
**/
bool delete_model(std::string model_name,
                   Data_config& parameters);

#endif /* __LIB_MOVEMENT_H__ */

