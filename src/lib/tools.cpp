#include "../../include/crustcrawler_throwing_ball_exp/tools.hpp"

using namespace std;

typedef vector <double> record_t;
typedef vector <record_t> data_t;


//The function that will execute the trajectory
bool execute_joint_trajectory(actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac,
                              trajectory_msgs::JointTrajectory& joint_trajectory,
                              Data_config &parameters, ros::Publisher &gripper_pub){
    crustcrawler_core_msgs::EndEffectorCommand gripper_command;
    gripper_command.id = 1;
    //if on the real robot calibrate the gripper and grasp the object
    if(!parameters.get_simulation()){
        //open gripper command "release"
        gripper_command.args = "{position: 100.0}";
        gripper_command.command = "go";
        gripper_pub.publish(gripper_command);

        std::cin.ignore();

        //close gripper command "grip"
        gripper_command.args = "{position: 0.0}";
        gripper_command.command = "go";
        gripper_pub.publish(gripper_command);

        std::cin.ignore();
    }
    //
    parameters.set_point_count(0.0);
    if (!ac.waitForServer(ros::Duration(2.0)))
    {
        ROS_ERROR("Could not connect to action server");
        return false;
    }

    //usleep(2e6);
    control_msgs::FollowJointTrajectoryGoal goal;

    goal.trajectory = joint_trajectory;
    goal.goal_time_tolerance = ros::Duration(1.0);
    ac.sendGoal(goal);
    if(!parameters.get_start_record_feedback())
        parameters.set_start_record_feedback(true);
    if(!parameters.get_record())
        parameters.set_record(true);

    ROS_WARN_STREAM("trajectory last point time is: " <<
                    joint_trajectory.points[(int)joint_trajectory.points.size() - 1].time_from_start.toSec());
    for(size_t i = 0; i < joint_trajectory.points[joint_trajectory.points.size() - 100].positions.size(); i++)
        ROS_ERROR_STREAM("joint: " << i << " has angle: " << joint_trajectory.points[joint_trajectory.points.size() - 100].positions[i]);
    ROS_INFO("*****************************************************************************************");
    if(!parameters.get_simulation()){
        while(!parameters.get_action_server_status().status_list.empty()){
            usleep(1e3);
            if(largest_difference(get_arranged_individual_joint_command(parameters),
                                  joint_trajectory.points[joint_trajectory.points.size() - 100].positions) < 0.1){

                //if(joint_trajectory.points[(int)joint_trajectory.points.size() - 1].time_from_start.toSec()
                //      - parameters.get_action_server_feedback().feedback.actual.time_from_start.toSec() < parameters.get_release_ball_dt()){
                gripper_command.args = "{position: 100.0}";
                gripper_command.command = "go";
                gripper_pub.publish(gripper_command);
            }
            //ROS_ERROR_STREAM("the difference is: " << largest_difference(extract_certain_arm_joints_values(parameters),
              //                                                           joint_trajectory.points[joint_trajectory.points.size() - 100].positions));
            //for(size_t i = 0; i < extract_certain_arm_joints_values(parameters).size(); i++)
              //  ROS_ERROR_STREAM("joint state for joint: " << i << " is: " << extract_certain_arm_joints_values(parameters)[i]);
            //ROS_INFO("*****************************************************************************************");
        }

    }
    else if(parameters.get_grap_ball_simulation())
    {

        while(!ac.getState().isDone()){
            double time_diff = joint_trajectory.points[(int)joint_trajectory.points.size() - 1].time_from_start.toSec()
                    - parameters.get_action_server_feedback().feedback.actual.time_from_start.toSec();
            if(time_diff < parameters.get_release_ball_dt() ){
                //ROS_ERROR_STREAM("the condition seems true cause the difference is: "
                //    << time_diff);
                open_gripper(parameters, gripper_pub);
            }
        }
    }
    if (ac.waitForResult(goal.trajectory.points[goal.trajectory.points.size()-1].time_from_start /*+ ros::Duration(10)*/))
    {
        ROS_INFO_STREAM("Action server reported successful execution: " << parameters.get_joint_action_result().result.error_code);
        return true;
    } else {
        ROS_WARN_STREAM("Action server could not execute trajectory: " << parameters.get_joint_action_result().result.error_code);
        return false;
    }
}

std::vector<double>& get_arranged_joint_command(Data_config& parameters){
    std::vector<double> joint_values;
    for(size_t i = 0; i < parameters.get_crustcrawler_arm_joints_names().size(); i++){
        joint_values.push_back(parameters.get_joint_command().command[distance(parameters.get_joint_command().names.begin(),
                                                                              find(parameters.get_joint_command().names.begin(),
                                                                                   parameters.get_joint_command().names.end(),
                                                                                   parameters.get_crustcrawler_arm_joints_names()[i]))]);
    }
     parameters.set_crustcrawler_arm_joint_command(joint_values);
    return parameters.get_crustcrawler_arm_joint_command();
}

std::vector<double>& get_arranged_individual_joint_command(Data_config& parameters){
    std::vector<double> joint_values;
    joint_values.push_back(parameters.get_joint_1_command());
    joint_values.push_back(parameters.get_joint_2_command());
    joint_values.push_back(parameters.get_joint_3_command());
    joint_values.push_back(parameters.get_joint_4_command());
    joint_values.push_back(parameters.get_joint_5_command());
    joint_values.push_back(parameters.get_joint_6_command());

    parameters.set_crustcrawler_arm_individual_joint_command(joint_values);
    return parameters.get_crustcrawler_arm_individual_joint_command();
}

//this function record the feedback of the joint server when executing a trajectory
void record_feedback(Data_config& parameters,
                     control_msgs::FollowJointTrajectoryActionFeedback& feedback,
                     ofstream& output_file){
    if(parameters.get_record() && parameters.get_joint_trajectory().points.size() > 3){
        if(!feedback.feedback.desired.positions.empty()){
            /*if(parameters.get_first()){
                parameters.set_start_time(feedback.feedback.error.time_from_start);
                parameters.set_first(false);
            }*/
            parameters.set_point_count(parameters.get_point_count() + 1);
            //output_file << feedback.feedback.header.stamp.toSec() - parameters.get_start_time()
            //record time of recording
            output_file << feedback.feedback.error.time_from_start
                        << ",";
            //record positions: desired and actual
            for(size_t i = 0; i < feedback.feedback.desired.positions.size(); ++i)
                output_file << feedback.feedback.desired.positions[i] << ",";
            for(size_t i = 0; i < feedback.feedback.actual.positions.size(); ++i)
                output_file << feedback.feedback.actual.positions[i] << ",";

            //record velocities: desired and actual
            if(parameters.get_velocity_option()){
                /*ROS_WARN_STREAM("recording velocity feedbacks, size of actual is: " <<
                                feedback.feedback.actual.velocities.size() <<
                                " and size of desired is: " <<
                                feedback.feedback.desired.velocities.size());*/
                for(size_t i = 0; i < feedback.feedback.desired.velocities.size(); ++i)
                    output_file << feedback.feedback.desired.velocities[i] << ",";
                for(size_t i = 0; i < feedback.feedback.actual.velocities.size(); ++i)
                    output_file << feedback.feedback.actual.velocities[i] << ",";
            }

            //record acceleration: desired and actual
            if(parameters.get_acceleration_option()){
                /*ROS_WARN_STREAM("recording acceleration feedbacks, size of actual is: " <<
                                feedback.feedback.actual.accelerations.size() <<
                                " and size of desired is: " <<
                                feedback.feedback.desired.accelerations.size());*/
                for(size_t i = 0; i < feedback.feedback.desired.accelerations.size(); ++i)
                    output_file << feedback.feedback.desired.accelerations[i] << ",";
                for(size_t i = 0; i < feedback.feedback.actual.accelerations.size(); ++i)
                    output_file << feedback.feedback.actual.accelerations[i] << ",";
            }
            output_file << "\n";
        }

    }
}


/**
 * @brief the function to spawn objects
 * @param name of the object to spawn, and its pose and the service client
 * @param argv
 * @return bool that sees if it succeed in spawning the model in gazebo or not
 */
bool spawn_model(const std::string model_to_spawn,
                 Data_config& parameters){
    parameters.configure_object_pose();
    gazebo_msgs::SpawnModel my_model;
    //char buffer[MAXPATHLEN];
    char* pPath;
    pPath = getenv ("PWD");
    //pPath = "/home/mukhtar/git/catkin_ws";
    std::string model_file_location;
    model_file_location.append(pPath);
    model_file_location.append("/src/" + parameters.get_package_name() + "/world/");
    model_file_location.append(model_to_spawn);
    model_file_location.append("/model.sdf");
    //ROS_INFO_STREAM("file location is: " << model_file_location.c_str());
    std::ifstream model_file(model_file_location.c_str());
    if (model_file){
        std::string line;
        while (!model_file.eof()) {
            std::getline(model_file,line);
            my_model.request.model_xml+=line;
        }
        model_file.close();
        my_model.request.model_name = model_to_spawn;
        my_model.request.reference_frame = "base";
        my_model.request.initial_pose = parameters.get_object_pose(model_to_spawn);
        parameters.get_gazebo_model_spawner().call(my_model);
        return my_model.response.success;
    } else {
        ROS_ERROR_STREAM("File " << model_file_location << " not found while spawning");
        exit(-1);
    }
}

/**
 * @brief remove one model
 * @param a
 * @param a
 * @return bool that
 */
bool delete_model(std::string model_name,
                  Data_config& parameters){
    //std::string current_model = model_name;
    gazebo_msgs::DeleteModel delete_model;
    delete_model.request.model_name = model_name;
    /*if (std::strcmp(current_model.c_str(), "table") == 0){
        delete_model.request.model_name = "table";
    } else if (std::strcmp(current_model.c_str(), "cube") == 0){
        delete_model.request.model_name = "cube";
    } else if (std::strcmp(current_model.c_str(), "cylinder") == 0){
        delete_model.request.model_name = "cylinder";
    } else {
        ROS_ERROR_STREAM("remove_object method : unknown model " << current_model);
        return false;
    }*/
    parameters.get_gazebo_model_delete_client().call(delete_model);

    return true;
}

bool move_with_action_server(actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac,
                             trajectory_msgs::JointTrajectory& my_joint_trajectory){
    if (!ac.waitForServer(ros::Duration(2.0)))
    {
        ROS_ERROR("Could not connect to action server");
        return false;
    }

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = my_joint_trajectory;
    goal.goal_time_tolerance = ros::Duration(1.0);
    ac.sendGoal(goal);

    while(!ac.getState().isDone());
    if (ac.waitForResult(goal.trajectory.points[goal.trajectory.points.size()-1].time_from_start + ros::Duration(1)))
    {
        ROS_INFO("Action server reported successful execution");
        return true;
    } else {
        ROS_WARN("Action server could not execute trajectory");
        return false;
    }
}

trajectory_msgs::JointTrajectoryPoint get_neutral_point(){
    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions = {-1.3, 0.3, -1.1, 0.0, -0.5, 0.0, 0.0, 0.0};
    pt.velocities.resize(7, 0.0);
    pt.accelerations.resize(7, 0.0);
    pt.effort.resize(7, 0.0);
    return pt;
}

geometry_msgs::Pose get_ball_pose(Data_config& parameters){
    gazebo_msgs::GetModelState model_state;
    model_state.request.relative_entity_name = "base";
    model_state.request.model_name = "ball";
    parameters.get_gazebo_model_state_client().call(model_state);
    return model_state.response.pose;
}

void locate_eef_pose(geometry_msgs::Pose eef_feedback, Data_config& parameters){

    Eigen::VectorXd end_effector_pose(6);
    geometry_msgs::Pose eef_pose_quat = eef_feedback;
    tf::Quaternion eef_rpy_orientation;

    tf::quaternionMsgToTF(eef_pose_quat.orientation, eef_rpy_orientation);

    double roll, yaw, pitch;
    tf::Matrix3x3 m(eef_rpy_orientation);
    m.getRPY(roll, pitch, yaw);
    Eigen::Vector3d eef_current_position;
    Eigen::Vector3d eef_current_orientation;
    eef_current_position << eef_pose_quat.position.x,
            eef_pose_quat.position.y,
            eef_pose_quat.position.z;

    eef_current_orientation <<    roll,
            pitch,
            yaw;
    end_effector_pose << eef_pose_quat.position.x,
            eef_pose_quat.position.y,
            eef_pose_quat.position.z,
            roll,
            pitch,
            yaw;
    parameters.set_eef_position(eef_current_position);
    parameters.set_eef_rpy_orientation(eef_current_orientation);
    parameters.set_eef_pose(eef_pose_quat);
    parameters.set_eef_rpy_pose(end_effector_pose);
    //ROS_WARN_STREAM("locating eef stuff gave for position: " << eef_current_position << "\n and for orientation: " << eef_current_orientation);
}

trajectory_msgs::JointTrajectoryPoint get_grapping_point(Data_config& parameters){
    geometry_msgs::PoseStamped my_desired_pose;
    my_desired_pose.header.frame_id = "/base";
    my_desired_pose.pose = get_ball_pose(parameters);
    /*ROS_ERROR_STREAM("ball position is: x = " << my_desired_pose.pose.position.x <<
                         ", y = " << my_desired_pose.pose.position.y <<
                         ", and z = " << my_desired_pose.pose.position.z);*/
    my_desired_pose.pose.orientation = parameters.get_eef_pose().orientation;
    /*ROS_ERROR_STREAM("right eef orientation is: x = " << my_desired_pose.pose.orientation.x <<
                         ", y = " << my_desired_pose.pose.orientation.y <<
                         ", z = " << my_desired_pose.pose.orientation.z <<
                         ", and w = " << my_desired_pose.pose.orientation.w);*/

    /////////// need to be solved
    /// ...............................................................................................
    /// .................................................................................................
    /// --------------------------------------------------------------------------------------------------
    trajectory_msgs::JointTrajectoryPoint pt;
    /*ROS_ERROR("trying to print solution to grap the ball");
    for(size_t i = 0; i < res.joints[0].position.size(); i++)
        ROS_ERROR_STREAM("solution to grap the ball give for joint: " << i << " angle: " << res.joints[0].position[i]);*/
    pt.velocities.resize(7, 0.0);
    pt.accelerations.resize(7, 0.0);
    pt.effort.resize(7, 0.0);
    return pt;
}

void construct_two_points_trajectory(Data_config& parameters,
                                     trajectory_msgs::JointTrajectory& my_joint_trajectory,
                                     trajectory_msgs::JointTrajectoryPoint second_pt){
    for(size_t i = 0; i < 2; i++){
        trajectory_msgs::JointTrajectoryPoint pt;
        //first point is current point
        if(i == 0)
            pt.positions = extract_arm_joints_values(parameters);
        //second point is given
        else
            pt.positions = second_pt.positions;
        pt.velocities.resize(second_pt.positions.size(), 0.0);
        pt.accelerations.resize(second_pt.positions.size(), 0.0);
        pt.effort.resize(second_pt.positions.size(), 0.0);
        pt.time_from_start = ros::Duration(3*i);
        my_joint_trajectory.points.push_back(pt);
    }
}

void open_gripper(Data_config& parameters, ros::Publisher& gripper_pub){
    crustcrawler_core_msgs::EndEffectorCommand gripper_command;
    gripper_command.id = parameters.get_gripper_id();
    gripper_command.args = "{position: 100.0}";
    gripper_command.command = "go";
    gripper_pub.publish(gripper_command);
}

void close_gripper(Data_config& parameters, ros::Publisher& gripper_pub){
    crustcrawler_core_msgs::EndEffectorCommand gripper_command;
    gripper_command.id = parameters.get_gripper_id();
    gripper_command.args = "{position: 0.0}";
    gripper_command.command = "go";
    gripper_pub.publish(gripper_command);
}

//go to the first position in the trajectory to be executed
bool go_to_initial_position(Data_config& parameters,
                            actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac,
                            ros::Publisher &gripper_pub){
    //move to neutral position so we can spawn objects
    trajectory_msgs::JointTrajectory my_joint_trajectory;
    my_joint_trajectory.joint_names = parameters.get_crustcrawler_arm_joints_names();
    if(parameters.get_grap_ball_simulation()){
        //first spawn the table and the ball in a known position then grap the ball and go to the first point
        construct_two_points_trajectory(parameters, my_joint_trajectory, get_neutral_point());
        if(!move_with_action_server(ac, my_joint_trajectory))
            return false;

        if(parameters.get_grap_ball_simulation()){
            spawn_model("table", parameters);
            spawn_model("ball", parameters);

            close_gripper(parameters, gripper_pub);
            usleep(1e6);
            open_gripper(parameters, gripper_pub);
            //std::cin.ignore();
            //grap the ball

            my_joint_trajectory.points.clear();
            construct_two_points_trajectory(parameters, my_joint_trajectory, get_grapping_point(parameters));


            ROS_WARN_STREAM("joints trajectory size to grap the ball is: " << my_joint_trajectory.points.size());
            if(!move_with_action_server(ac, my_joint_trajectory))
                return false;

            close_gripper(parameters, gripper_pub);
            usleep(1e5);
        }
    }


    //construct a trajectory with two point (current point and desired point, first point in the trajectory)

    if(!parameters.get_joint_trajectory().points.empty()){
        my_joint_trajectory.points.clear();
        construct_two_points_trajectory(parameters, my_joint_trajectory, parameters.get_joint_trajectory().points[0]);
    }
    move_with_action_server(ac, my_joint_trajectory);
    //ROS_WARN_STREAM("trying to move to initial position, the action server gave: "
    //  << parameters.get_joint_action_result().result.error_code);
    if(parameters.get_joint_action_result().result.error_code != 0)
        return false;

    if(parameters.get_grap_ball_simulation())
        delete_model("table", parameters);
    return true;
}

//check all trajectory points for selfcollision
bool is_trajectory_valid(Data_config& parameters){
    planning_scene::PlanningScene planning_scene(parameters.get_crustcrawler_robot_model());
    //planning_scene.
    collision_detection::CollisionRequest req;
    req.contacts = true;
    req.max_contacts = 100;
    collision_detection::CollisionResult collision_result;
    collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
    std::filebuf fb;
    fb.open ("test.txt", std::ios::out);
    std::ostream acm_matrix(&fb);

    acm.print(acm_matrix);
    std::vector<std::string> joint_names = parameters.get_crustcrawler_arm_joints_names();
    robot_state::RobotState robot_state_holder(parameters.get_crustcrawler_robot_model());
    //this is for simulation
    //robot_state_holder.setVariableValues(parameters.get_joint_state());

    //this is real robot
    robot_state_holder.setVariablePositions(joint_names, extract_certain_arm_joints_values(parameters));

    for(unsigned i = 0; i < parameters.get_joint_trajectory().points.size(); i++){
        collision_result.clear();
        robot_state_holder.setVariablePositions(parameters.get_crustcrawler_arm_joints_names(),
                                                parameters.get_joint_trajectory().points[i].positions);
        planning_scene.setCurrentState(robot_state_holder);

        planning_scene.checkSelfCollision(req, collision_result, planning_scene.getCurrentState(), planning_scene.getAllowedCollisionMatrix());
        ROS_INFO_STREAM("Test 6: Current state is "
                        << (collision_result.collision ? "in" : "not in")
                        << " self collision, for iteration: " << i);
        collision_detection::CollisionResult::ContactMap::const_iterator it;
        for(it = collision_result.contacts.begin();
            it != collision_result.contacts.end();
            ++it)
        {
            ROS_INFO("Contact between: %s and %s",
                     it->first.first.c_str(),
                     it->first.second.c_str());
        }

        /*robot_state::RobotState check_state = planning_scene.getCurrentState();
        const double* my_variables = check_state.getVariablePositions();
        for(size_t j = 0; j < check_state.getVariableCount(); j++){
            ROS_WARN_STREAM("joint number: " << j << "with the name: " << check_state.getVariableNames()[j] << " position is: " << my_variables[j]);
        }*/
        if(collision_result.collision)
            return false;
    }
    return true;
}

std::vector<double>& extract_arm_joints_values(Data_config& parameters){
    std::vector<std::string> joint_names;
    std::vector<double> joint_values;
    //std::vector<int> joint_index;
    joint_names = parameters.get_crustcrawler_arm_joints_names();

    for(unsigned i = 0; i < joint_names.size(); ++i){
        joint_values.push_back(parameters.get_joint_state().position[distance(parameters.get_joint_state().name.begin(),
                                                                              find(parameters.get_joint_state().name.begin(),
                                                                                   parameters.get_joint_state().name.end(),
                                                                                   joint_names[i]))]);
    }
    parameters.set_crustcrawler_arm_joint_values(joint_values);
    return parameters.get_crustcrawler_arm_joint_values();
}

std::vector<double>& extract_certain_arm_joints_values(Data_config& parameters){
    std::vector<std::string> joint_names;
    std::vector<double> joint_values;
    //std::vector<int> joint_index;
    joint_names = parameters.get_crustcrawler_arm_joints_names();

    for(unsigned i = 0; i < joint_names.size(); i++){
        joint_values.push_back(parameters.get_joint_state().position[distance(parameters.get_joint_state().name.begin(),
                                                                              find(parameters.get_joint_state().name.begin(),
                                                                                   parameters.get_joint_state().name.end(),
                                                                                   joint_names[i]))]);
    }
    parameters.set_crustcrawler_arm_joint_values(joint_values);
    return parameters.get_crustcrawler_arm_joint_values();
}

//record arm joint positions into a file
void record_arm_joint_trajectory(ofstream& output_file, Data_config& parameters){
    //first record the time stamp for this waypoint
    //output_file << parameters.get_joint_state().header.stamp.now().toSec() - parameters.get_last_time() << ",";
    output_file << parameters.get_last_time() + 0.002 << ",";
    parameters.set_last_time(parameters.get_last_time() + 0.002);
    for(unsigned i = 0; i < parameters.get_crustcrawler_arm_joint_values().size(); ++i)
        output_file << parameters.get_crustcrawler_arm_joint_values()[i] << ",";
    output_file << "\n";
}

//-----------------------------------------------------------------------------
// Let's overload the stream input operator to read a list of CSV fields (which a CSV record).
// Remember, a record is a list of doubles separated by commas ','.
istream& operator >> ( istream& ins, record_t& record )
{
    // make sure that the returned record contains only the stuff we read now
    record.clear();

    // read the entire line into a string (a CSV record is terminated by a newline)
    string line;
    getline( ins, line );

    // now we'll use a stringstream to separate the fields out of the line
    stringstream ss( line );
    string field;
    while (getline( ss, field, ',' ))
    {
        // for each field we wish to convert it to a double
        // (since we require that the CSV contains nothing but floating-point values)
        stringstream fs( field );
        double f = 0.0;  // (default value is 0.0)
        fs >> f;

        // add the newly-converted field to the end of the record
        record.push_back( f );
    }

    // Now we have read a single line, converted into a list of fields, converted the fields
    // from strings to doubles, and stored the results in the argument record, so
    // we just return the argument stream as required for this kind of input overload function.
    return ins;
}

//-----------------------------------------------------------------------------
// Let's likewise overload the stream input operator to read a list of CSV records.
// This time it is a little easier, just because we only need to worry about reading
// records, and not fields.
istream& operator >> ( istream& ins, data_t& data )
{
    // make sure that the returned data only contains the CSV data we read here
    data.clear();

    // For every record we can read from the file, append it to our resulting data
    record_t record;
    while (ins >> record)
    {
        data.push_back( record );
    }

    // Again, return the argument stream as required for this kind of input stream overload.
    return ins;
}

//get largest difference between elements of two vectors
double largest_difference(std::vector<double> &first, std::vector<double> &second){
    Eigen::VectorXd difference(first.size());
    double my_max = 0;
    for(size_t j = 0; j < first.size(); ++j)
        difference(j) = fabs(first[j] - second[j]);
    for(size_t j = 0; j < first.size(); ++j){
        if(difference(j) > my_max)
            my_max = difference(j);
    }
    return my_max;
}

//git rid of doubled vectors in a a vector of vectors
void optimize_vector_of_vectors(std::vector<std::vector<double>>& vector_to_optimize, Data_config& parameters){
    //make a copy to work with and another to be the output
    std::vector<std::vector<double>> working_copy = vector_to_optimize, output_copy;
    for(unsigned i = 0; i < working_copy.size(); i++)
        for(unsigned j = 1; j < working_copy.size(); j++)
            if(largest_difference(working_copy[i], working_copy[j]) < parameters.get_epsilon())
                working_copy.erase(working_copy.begin() + j);
    vector_to_optimize = working_copy;
}

void construct_joint_trajectory_from_vector(trajectory_msgs::JointTrajectory& my_joint_trajectory,
                                            data_t& raw_joint_traj,
                                            double& dt,
                                            bool& velocity_option,
                                            bool& acceleration_option){
    double t_ = 0.0;
    if(!raw_joint_traj.empty()){
        //if it is sangsu kind of files (15 variables per line) do the following loop
        if(raw_joint_traj[0].size() > 6){
            for(size_t i = 0; i < raw_joint_traj.size(); i++){
                trajectory_msgs::JointTrajectoryPoint pt;
                pt.positions = {raw_joint_traj[i][1], raw_joint_traj[i][2], raw_joint_traj[i][3],
                                raw_joint_traj[i][4], raw_joint_traj[i][5], raw_joint_traj[i][6]};
                if(velocity_option && acceleration_option){
                    pt.velocities = {raw_joint_traj[i][8], raw_joint_traj[i][9], raw_joint_traj[i][10],
                                     raw_joint_traj[i][11], raw_joint_traj[i][12], raw_joint_traj[i][13]};
                    pt.accelerations = {raw_joint_traj[i][15], raw_joint_traj[i][16], raw_joint_traj[i][17],
                                        raw_joint_traj[i][18], raw_joint_traj[i][19], raw_joint_traj[i][20]};
                }
                else{
                    pt.velocities.resize(6, 0.0);
                    pt.accelerations.resize(6, 0.0);
                }
                pt.effort.resize(6, 0.0);
                //pt.time_from_start = ros::Duration(raw_joint_traj[i][0]);

                pt.time_from_start = ros::Duration(i*0.01);
                my_joint_trajectory.points.push_back(pt);
            }
        }
        //if it is a file with only joints waypoints for one arm (7 variables per line) do the following loop
        else {
            for(unsigned i = 0; i < raw_joint_traj.size(); i++){
                trajectory_msgs::JointTrajectoryPoint pt;
                pt.positions = raw_joint_traj[i];
                pt.velocities.resize(raw_joint_traj[i].size(), 0.0);
                pt.accelerations.resize(raw_joint_traj[i].size(), 0.0);
                pt.effort.resize(raw_joint_traj[i].size(), 0.0);
                pt.time_from_start = ros::Duration(i*dt);
                t_+=i*dt;
                my_joint_trajectory.points.push_back(pt);
            }
        }
    }
}

//The function that will construct the joint trajectory
void construct_joint_trajectory_from_file(std::ifstream& text_file, Data_config& parameters){
    //Extract all joints waypoints and delta_t from textfile
    // Here is the data we want.
    data_t data;
    text_file >> data;
    // Complain if something went wrong.
    if (!text_file.eof())
    {
        ROS_ERROR("Fooey!");
        return ;
    }
    text_file.close();
    //delete doupled values
    optimize_vector_of_vectors(data, parameters);

    //for printing all joints values
    /*for(unsigned i = 0; i < data.size(); i++){
        for(unsigned j = 0; j < data[i].size(); j++)
            ROS_INFO_STREAM("this element is: " << data[i][j]);
        ROS_INFO("*****************************************");
    }*/

    //check that all waypoints

    trajectory_msgs::JointTrajectory my_joint_trajectory;
    my_joint_trajectory.joint_names = parameters.get_crustcrawler_arm_joints_names();
    construct_joint_trajectory_from_vector(my_joint_trajectory, data, parameters.get_dt(),
                                           parameters.get_velocity_option(),
                                           parameters.get_acceleration_option());
    my_joint_trajectory.header.frame_id = "real_trajectory";

    //set the trajectory in the parameters
    parameters.set_joint_trajectory(my_joint_trajectory);
}
