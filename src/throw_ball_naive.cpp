#include "crustcrawler_throwing_ball_exp/tools.hpp"

// The parameters structure is used by all call backs, main and service
Data_config parameters;
std::ofstream output_file;

void joint_state_callback(sensor_msgs::JointState jo_state){
    parameters.set_joint_state(jo_state);
}

void joint_command_callback(const crustcrawler_core_msgs::JointCommand::ConstPtr& joint_command){
    parameters.set_joint_command(*joint_command);
}

void feedback_callback(control_msgs::FollowJointTrajectoryActionFeedback feedback){
    //parameters.set_joint_action_feedback(feedback);
    if(parameters.get_start_record_feedback()){
        parameters.set_action_server_feedback(feedback);
        record_feedback(parameters, feedback, output_file);
    }
}

void status_callback(const actionlib_msgs::GoalStatusArray::ConstPtr& status){
    parameters.set_action_server_status(*status);
}

void result_callback(control_msgs::FollowJointTrajectoryActionResult result){
    parameters.set_action_server_result(result);
}

//call back that register end effector pose and rearrange the orientation in RPY
void eef_callback(const crustcrawler_core_msgs::EndpointState::ConstPtr& eef_feedback){
    locate_eef_pose(eef_feedback->pose, parameters);
}

//joints commands callbacks
void joint_1_cammand_callback(const std_msgs::Float64::ConstPtr& joint_command){
    parameters.set_joint_1_command(joint_command->data);
}
void joint_2_cammand_callback(const std_msgs::Float64::ConstPtr& joint_command){
    parameters.set_joint_2_command(joint_command->data);
}
void joint_3_cammand_callback(const std_msgs::Float64::ConstPtr& joint_command){
    parameters.set_joint_3_command(joint_command->data);
}
void joint_4_cammand_callback(const std_msgs::Float64::ConstPtr& joint_command){
    parameters.set_joint_4_command(joint_command->data);
}
void joint_5_cammand_callback(const std_msgs::Float64::ConstPtr& joint_command){
    parameters.set_joint_5_command(joint_command->data);
}
void joint_6_cammand_callback(const std_msgs::Float64::ConstPtr& joint_command){
    parameters.set_joint_6_command(joint_command->data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "throw_ball_naive");
    ros::NodeHandle n;

    //subscribers
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/crustcrawler/follow_joint_trajectory", true);
    ros::Subscriber sub_r_eef_msg = n.subscribe<crustcrawler_core_msgs::EndpointState>("/crustcrawler/endpoint_state", 10, eef_callback);
    ros::Subscriber joint_state_sub = n.subscribe<sensor_msgs::JointState>("/crustcrawler/joint_states", 1, joint_state_callback);
    ros::Subscriber joint_command_sub = n.subscribe<crustcrawler_core_msgs::JointCommand>("/crustcrawler/joint_command", 1, joint_command_callback);
    ros::Subscriber feedback_sub = n.subscribe<control_msgs::FollowJointTrajectoryActionFeedback>("/crustcrawler/follow_joint_trajectory/feedback", 1, feedback_callback);
    ros::Subscriber result_sub = n.subscribe<control_msgs::FollowJointTrajectoryActionResult>("/crustcrawler/follow_joint_trajectory/result", 1, result_callback);
    ros::Subscriber status_sub = n.subscribe<actionlib_msgs::GoalStatusArray>("/crustcrawler/follow_joint_trajectory/status", 1, status_callback);
    ros::Publisher gripper_pub = n.advertise<crustcrawler_core_msgs::EndEffectorCommand>("/crustcrawler/end_effector/gripper/command", true);
    //subscribe to the actual joints commands
    ros::Subscriber joint_1_command = n.subscribe<std_msgs::Float64>("/crustcrawler/joint_1_position_controller/command", 1, joint_1_cammand_callback);
    ros::Subscriber joint_2_command = n.subscribe<std_msgs::Float64>("/crustcrawler/joint_2_position_controller/command", 1, joint_2_cammand_callback);
    ros::Subscriber joint_3_command = n.subscribe<std_msgs::Float64>("/crustcrawler/joint_3_position_controller/command", 1, joint_3_cammand_callback);
    ros::Subscriber joint_4_command = n.subscribe<std_msgs::Float64>("/crustcrawler/joint_4_position_controller/command", 1, joint_4_cammand_callback);
    ros::Subscriber joint_5_command = n.subscribe<std_msgs::Float64>("/crustcrawler/joint_5_position_controller/command", 1, joint_5_cammand_callback);
    ros::Subscriber joint_6_command = n.subscribe<std_msgs::Float64>("/crustcrawler/joint_6_position_controller/command", 1, joint_6_cammand_callback);

    ros::AsyncSpinner my_spinner(1);
    my_spinner.start();

    std::string input_file_path, feedback_file_path;
    double dt;
    bool record = false, execute = false;
    n.getParam("input_file_path", input_file_path);
    n.getParam("feedback_file_path", feedback_file_path);
    n.getParam("dt", dt);
    n.getParam("rate", parameters.get_rate());
    n.getParam("release_ball_dt", parameters.get_release_ball_dt());
    n.getParam("record", record);
    n.getParam("grap_simulation", parameters.get_grap_ball_simulation());
    n.getParam("epsilon", parameters.get_epsilon());
    n.getParam("execute", execute);
    n.getParam("velocity_option", parameters.get_velocity_option());
    n.getParam("acceleration_option", parameters.get_acceleration_option());
    n.getParam("simulation", parameters.get_simulation());
    n.getParam("check_collision", parameters.get_check_collision());
    n.getParam("start_trajectory_number", parameters.get_start_trajectory_number());
    n.getParam("last_trajectory_number", parameters.get_last_trajectory_number());
    n.getParam("gripper_id", parameters.get_gripper_id());

    if(parameters.get_grap_ball_simulation()){
        ros::ServiceClient spawner = n.serviceClient<gazebo_msgs::SpawnModel> ("/gazebo/spawn_sdf_model");
        ros::ServiceClient state = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
        ros::ServiceClient deleter = n.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
        parameters.set_gazebo_model_spawner(spawner);
        parameters.set_gazebo_model_state_clt(state);
        parameters.set_gazebo_model_delete_clt(deleter);
    }

    parameters.set_dt(dt);
    parameters.create_model();

    //test writing
    if(record){
        std::ofstream output_file;
        output_file.open("1.txt", std::ofstream::out);
        std::cin.ignore();
        //parameters.set_last_time(ros::Time::now().toSec());
        parameters.set_last_time(0.0);
        ros::Rate rate(parameters.get_rate());
        while(ros::ok()){
            extract_arm_joints_values(parameters);
            record_arm_joint_trajectory(output_file, parameters);
            rate.sleep();
        }
        output_file.close();
    }
    else{
        for(int i = parameters.get_start_trajectory_number(); i <= parameters.get_last_trajectory_number(); i++){

            //test reading and executing a trajectory
            std::ifstream input_file(input_file_path + std::to_string(i) + ".txt", std::ios_base::in);

            output_file.open(feedback_file_path, std::ofstream::out);

            construct_joint_trajectory_from_file(input_file, parameters);


            if(parameters.get_check_collision()){
                if(is_trajectory_valid(parameters)){
                    if(execute){
                        while(!go_to_initial_position(parameters, ac, gripper_pub))
                            ROS_WARN_STREAM("trying to move to initial position, the action server gave: "
                                     << parameters.get_joint_action_result().result.error_code);
                        execute_joint_trajectory(ac, parameters.get_joint_trajectory(), parameters, gripper_pub);
                    }
                }
                else
                    ROS_ERROR("trajectory not valid !!!!!!!!!");
            }
            else if(execute){
                while(!go_to_initial_position(parameters, ac, gripper_pub))
                    ROS_WARN_STREAM("trying to move to initial position, the action server gave: "
                             << parameters.get_joint_action_result().result.error_code);
                execute_joint_trajectory(ac, parameters.get_joint_trajectory(), parameters, gripper_pub);
            }

            input_file.close();
            output_file.close();


            parameters.set_start_record_feedback(false);
            parameters.set_record(false);
            ROS_INFO_STREAM("trajectory size is: " << parameters.get_joint_trajectory().points.size());
            ROS_WARN_STREAM("finished trajectory: " << i << " press enter for next trajectory");
            ros::Duration my_duration(0);
            parameters.get_action_server_feedback().feedback.actual.time_from_start = my_duration;
            std::cin.ignore();
            if(parameters.get_grap_ball_simulation())
                delete_model("ball", parameters);
        }
    }
    return 0;
}
