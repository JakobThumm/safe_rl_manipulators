#include <simulation_time_control/nodelet.h>

namespace simulation_time_control {

void SimulationTimeControlNodelet::onInit() {
  ros::NodeHandle& private_nh = getPrivateNodeHandle();

  /// get the parameters
  double sample_time = 0.004;

  if (ros::param::has("/sample_time")) {
    ros::param::get("/sample_time", sample_time);
    ROS_ASSERT(sample_time > 0);
  }

  bool step_control = true;
  if (ros::param::has("/step_control")) {
    ros::param::get("/step_control", step_control);
  }
  double initial_max_update_rate=10000;
  int n_delays_queue=5;
  double speed_factor=0.1;
  double max_max_update_rate=15000;
  double max_delay_time=0.5;
  double first_reduction_after=10;
  double time_between_reductions=1;
  double time_between_speedups=10;
  if (!step_control) {
    if (ros::param::has("/initial_max_update_rate")) {
      ros::param::get("/initial_max_update_rate", initial_max_update_rate);
    }
    if (ros::param::has("/n_delays_queue")) {
      ros::param::get("/n_delays_queue", n_delays_queue);
    }
    if (ros::param::has("/speed_factor")) {
      ros::param::get("/speed_factor", speed_factor);
    }
    if (ros::param::has("/max_max_update_rate")) {
      ros::param::get("/max_max_update_rate", max_max_update_rate);
    }
    if (ros::param::has("/max_delay_time")) {
      ros::param::get("/max_delay_time", max_delay_time);
    }
    if (ros::param::has("/first_reduction_after")) {
      ros::param::get("/first_reduction_after", first_reduction_after);
    }
    if (ros::param::has("/time_between_reductions")) {
      ros::param::get("/time_between_reductions", time_between_reductions);
    }
    if (ros::param::has("/time_between_speedups")) {
      ros::param::get("/time_between_speedups", time_between_speedups);
    }
  }
  // Get the max step size (dt sim time per update cycle) from gazbeo
  ros::ServiceClient get_physics_properties_client = private_nh.serviceClient<gazebo_msgs::GetPhysicsProperties>("/gazebo/get_physics_properties");
  gazebo_msgs::GetPhysicsProperties srv;
  double max_time_step = 0;
  double max_update_rate = 0;
  while (max_time_step <= 0) {
    get_physics_properties_client.call(srv);
    max_time_step = srv.response.time_step;
    max_update_rate = srv.response.max_update_rate;
    ros::Duration(0.1).sleep();
    ROS_WARN_STREAM("Gazebo physics parameter time step = " << srv.response.time_step << " not available yet. Waiting for GetPhysicsProperties service...");
  }
  // Set initial max update rate
  ros::ServiceClient set_physics_properties_client = private_nh.serviceClient<gazebo_msgs::SetPhysicsProperties>("/gazebo/set_physics_properties");
  gazebo_msgs::SetPhysicsProperties set_physics_srv;
  if (!step_control) {
    set_physics_srv.request.gravity = srv.response.gravity;
    set_physics_srv.request.max_update_rate = initial_max_update_rate;
    set_physics_srv.request.ode_config = srv.response.ode_config;
    set_physics_srv.request.time_step = srv.response.time_step;
    // Set the initial update rate
    set_physics_properties_client.call(set_physics_srv);

    ROS_INFO_STREAM("Max time step size = " << max_time_step);
  }
  
  // Set the sample time to be a multiplicative of the max_time_step 
  int n_sim_steps_per_sample_time = 1;
  if (sample_time >= max_time_step) {
    double d = sample_time / max_time_step;
    n_sim_steps_per_sample_time = floor(d);
    sample_time = max_time_step * n_sim_steps_per_sample_time;
  } else {
    sample_time = max_time_step;
  }
  ROS_INFO_STREAM("n_sim_steps_per_sample_time = " << n_sim_steps_per_sample_time << " sample time = " << sample_time);


  // Publishes the signal to start the next online verification calculation cycle
  ros::Publisher pub_start_next_cycle = private_nh.advertise<custom_robot_msgs::BoolHeadered>("/next_sim_cycle", 1000);

  // Simulation step control
  if (step_control) {
    sim_time_control_ = new SimulationTimeControl(pub_start_next_cycle, sample_time, n_sim_steps_per_sample_time);
    sub_cycle_finished_ = private_nh.subscribe("/cycle_finished", 1000, &SimulationTimeControl::stepSimControl, sim_time_control_);
  } else { 
    // Simulation speed control
    sim_time_control_ = new SimulationTimeControl(pub_start_next_cycle, 
        set_physics_properties_client,
        set_physics_srv,
        sample_time, 
        initial_max_update_rate,
        n_delays_queue,
        speed_factor,
        max_max_update_rate,
        max_delay_time,
        first_reduction_after,
        time_between_reductions,
        time_between_speedups);
    /// subscribe to the path_edges topic
    sub_cycle_finished_ = private_nh.subscribe("/cycle_finished", 1000, &SimulationTimeControl::simControl, sim_time_control_);
  }
}
} // namespace simulation_time_control

PLUGINLIB_EXPORT_CLASS(simulation_time_control::SimulationTimeControlNodelet, nodelet::Nodelet);
    
