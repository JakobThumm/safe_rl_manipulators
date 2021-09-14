#include "failsafe_consistent_planner/nodelet.h"

namespace online_verification
{
  void FailsafeConsistentNodelet::onInit(){
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    /// get the parameters
    double sample_time = 0.004, t_buff = 0, max_s_stop = 0;
    std::vector<double> v_max_allowed, a_max_allowed, j_max_allowed, a_max_traj, j_max_traj;
    std::string robot_name = "modrob0";
    if (ros::param::has("/robot_name")) {
        ros::param::get("/robot_name", robot_name);
    }
    if (ros::param::has("/sample_time")) {
        ros::param::get("/sample_time", sample_time);
        ROS_ASSERT(sample_time > 0);
    }
    if (ros::param::has("/buffer_time")) {
        ros::param::get("/buffer_time", t_buff);
    }
    if (ros::param::has("/max_s_stop")) {
        ros::param::get("/max_s_stop", max_s_stop);
    }
    if (ros::param::has("/v_max_allowed")) {
        ros::param::get("/v_max_allowed", v_max_allowed);
    }
    if (ros::param::has("/a_max_allowed")) {
        ros::param::get("/a_max_allowed", a_max_allowed);
    }
    if (ros::param::has("/j_max_allowed")) {
        ros::param::get("/j_max_allowed", j_max_allowed);
    }
    if (ros::param::has("/a_max_traj")) {
        ros::param::get("/a_max_traj", a_max_traj);
    }
    if (ros::param::has("/j_max_traj")) {
        ros::param::get("/j_max_traj", j_max_traj);
    }
    double nb_joints = 1;
    if (ros::param::has("/nb_joints")) {
        ros::param::get("/nb_joints", nb_joints);
    }
    std::vector<std::vector<double>> q_vals(nb_joints);
    for (int i=1; i <= nb_joints; i++){
        std::string qi = "/q" + std::to_string(i);
        ros::param::get(qi, q_vals[i-1]);
    }

    ros::Publisher pub_braking_time = private_nh.advertise<custom_robot_msgs::DoubleHeadered>("/tbrake", 1000);
    ros::Publisher start_goal_pub = private_nh.advertise<custom_robot_msgs::StartGoalMotion>(
        "/start_goal_motion", 1000);
    ros::Publisher motion_pub = private_nh.advertise<modrob_workstation::RobotConfigCommanded>("/ns/robot_config_commanded", 1000);
    ros::Publisher cycle_finished_pub = private_nh.advertise<std_msgs::Empty>("/cycle_finished", 1000);

    // Get the max step size (dt sim time per update cycle) from gazbeo
    ros::ServiceClient physics_properties_client = private_nh.serviceClient<gazebo_msgs::GetPhysicsProperties>("/gazebo/get_physics_properties");
    double max_time_step = 0;
    while (max_time_step <= 0) {
        gazebo_msgs::GetPhysicsProperties srv;
        physics_properties_client.call(srv);
        max_time_step = srv.response.time_step;
        ros::Duration(0.1).sleep();
        ROS_WARN_STREAM("Gazebo physics parameter time step = " << srv.response.time_step << " not available yet. Waiting for GetPhysicsProperties service...");
    }
    
    ROS_INFO_STREAM("Max time step size = " << max_time_step);
    // Set the sample time to be a multiplicative of the max_time_step 
    int n_sim_steps_per_sample_time = 1;
    if (sample_time >= max_time_step){
        double d = sample_time / max_time_step;
        n_sim_steps_per_sample_time = floor(d);
        sample_time = max_time_step * n_sim_steps_per_sample_time;
    } else {
        sample_time = max_time_step;
    }
    ROS_INFO_STREAM("n_sim_steps_per_sample_time = " << n_sim_steps_per_sample_time);

    // store the long term trajectory
    std::vector<Motion> long_term_traj;
    for(int i = 0; i < q_vals[0].size(); i++){
        std::vector<double> angles(nb_joints);
        for(int j=0; j < nb_joints; j++){
            angles[j] = q_vals[j][i];
        }
        long_term_traj.push_back(Motion(i*sample_time, angles));
    }
    ROS_DEBUG("Creating LTT...");
    LongTermTraj long_term_trajectory = LongTermTraj(long_term_traj);
    ROS_DEBUG("Initializing FailSafePathConsistent object...");
    // initialize the failsafe planner
    fspc = new FailsafePathConsistent(nb_joints, sample_time, t_buff, 
                                      max_s_stop, v_max_allowed, a_max_allowed, 
                                      j_max_allowed, a_max_traj, j_max_traj, 
                                      long_term_trajectory, pub_braking_time, start_goal_pub, 
                                      motion_pub, cycle_finished_pub);
    ROS_DEBUG("FailSafePathConsistent object initialized");
    /// subscribe to the path_edges topic
    sub_init = private_nh.subscribe("/initialisation", 1000, &FailsafePathConsistent::initPotentialTrajectory, fspc);
    sub_path_safety = private_nh.subscribe("/path_safety", 1000, &FailsafePathConsistent::getPathSafety, fspc);
    sub_new_goal = private_nh.subscribe("/" + robot_name + "/new_goal_motion", 1000, &FailsafePathConsistent::newLongTermTrajectory, fspc);
    sub_next_cycle = private_nh.subscribe("/next_sim_cycle", 1000, &FailsafePathConsistent::getNextCycle, fspc);
  }
} // namespace online_verification

PLUGINLIB_EXPORT_CLASS(online_verification::FailsafeConsistentNodelet, nodelet::Nodelet);
    
