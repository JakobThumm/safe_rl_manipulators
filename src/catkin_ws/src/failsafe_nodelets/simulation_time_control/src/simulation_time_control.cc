#include <simulation_time_control/simulation_time_control.h>

namespace simulation_time_control {

SimulationTimeControl::SimulationTimeControl(const ros::Publisher &pub_start_next_cycle, 
    const ros::ServiceClient& set_physics_properties_client,
    gazebo_msgs::SetPhysicsProperties& set_physics_srv,
    double sample_time, 
    double initial_max_update_rate,
    int n_delays_queue,
    double speed_factor,
    double max_max_update_rate,
    double max_delay_time,
    double first_reduction_after,
    double time_between_reductions,
    double time_between_speedups):
  pub_start_next_cycle_(pub_start_next_cycle),
  set_physics_properties_client_(set_physics_properties_client),
  set_physics_srv_(set_physics_srv),
  sample_time_(sample_time),
  max_update_rate_(initial_max_update_rate),
  initial_max_update_rate_(initial_max_update_rate),
  n_delays_queue_(n_delays_queue),
  speed_factor_(speed_factor),
  max_max_update_rate_(max_max_update_rate),
  n_sim_steps_per_sample_time_(0)
{
  // Fill the queue
  cycle_begin_time_ = ros::Time::now();
  for (int i = 0; i < n_delays_queue; i++) {
      delay_queue_.push(cycle_begin_time_);
  }
  this->max_delay_time_ = ros::Duration(max_delay_time);
  this->first_reduction_after_ = ros::Time(first_reduction_after);
  this->time_between_reductions_ = ros::Duration(time_between_reductions);
  this->time_between_speedups_ = ros::Duration(time_between_speedups);
}

SimulationTimeControl::SimulationTimeControl(const ros::Publisher &pub_start_next_cycle, 
    double sample_time,
    int n_sim_steps_per_sample_time):
  pub_start_next_cycle_(pub_start_next_cycle),
  sample_time_(sample_time),
  n_sim_steps_per_sample_time_(n_sim_steps_per_sample_time) 
{
  gazebo::client::setup();
  gazebo::transport::NodePtr gazebonode_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  gazebonode_->Init();
  world_control_pub_ = gazebonode_->Advertise<gazebo::msgs::WorldControl>("~/world_control");
  ROS_WARN_STREAM("Wait for gazebo connection for world control");
  world_control_pub_->WaitForConnection();
  ROS_WARN_STREAM("Gazebo connected for world control");
}

void SimulationTimeControl::simControl(const std_msgs::EmptyConstPtr& calculation_finished) {
  ROS_DEBUG_STREAM("SimulationTimeControl::simControl");
  ros::Time now = ros::Time::now();
  ros::Duration sleep = ros::Duration(sample_time_) - (now - cycle_begin_time_); 
  if (sleep.toSec() >= 0) {
    ros::Time().sleepUntil(cycle_begin_time_ + ros::Duration(sample_time_));
  } else {
    if ((now - delay_queue_.front()) < max_delay_time_ && now > first_reduction_after_ && (now-last_reduction_) > time_between_reductions_) {
      max_update_rate_ *= (1-speed_factor_);
      ROS_INFO_STREAM("Online verification is slower than simulation (now = " << now.toSec() << ", delay_queue.front() = " << delay_queue_.front().toSec() << "). Slowing down simulation by " << (speed_factor_*100) << "%. New max update rate = " << max_update_rate_);
      set_physics_srv_.request.max_update_rate = max_update_rate_;
      set_physics_properties_client_.call(set_physics_srv_);
      last_reduction_ = now;
    }
    //ROS_INFO_STREAM("Online verification is slower than simulation by " << -1*sleep.toSec() << " (now = " << now.toSec() << ", begin = " << begin.toSec() << ").");
    delay_queue_.push(now);
    delay_queue_.pop();
  }
  if ((now-last_reduction_) > time_between_speedups_ && now > first_reduction_after_ && (1+speed_factor_) * max_update_rate_ <= max_max_update_rate_) {
    max_update_rate_ *= (1+speed_factor_);
    ROS_INFO_STREAM("Attempting to speed up simulation by " << (speed_factor_*100) << "%. New max update rate = " << max_update_rate_);
    set_physics_srv_.request.max_update_rate = max_update_rate_;
    set_physics_properties_client_.call(set_physics_srv_);
    last_reduction_ = now;
  }

  cycle_begin_time_ = ros::Time::now();
  // Publish done signal
  custom_robot_msgs::BoolHeadered done_msg;
  done_msg.header.stamp = cycle_begin_time_;
  done_msg.data = true;
  pub_start_next_cycle_.publish(done_msg);
}

void SimulationTimeControl::stepSimControl(const std_msgs::EmptyConstPtr& calculation_finished) {
  ROS_DEBUG_STREAM("SimulationTimeControl::stepSimControl");
  try {
    // wait until the end of the loop.
    ros::Duration sleep = ros::Duration(sample_time_) - (ros::Time::now() - cycle_begin_time_); 
    ROS_DEBUG("Sample time = %f, Sleep = %f, now = %f, cycle_begin_time = %f", sample_time_, sleep.toSec(), ros::Time::now().toSec(), cycle_begin_time_.toSec());
    if (sleep.toSec() >= 0) {
      ros::Time().sleepUntil(cycle_begin_time_ + ros::Duration(sample_time_));
    }
    
    // Forward simulation to end of sample time
    gazebo::msgs::WorldControl step_msg;
    // Unpause for 0 steps / Pause for >=1 steps
    bool to_pause = (n_sim_steps_per_sample_time_ >= 1);
    step_msg.set_pause(to_pause);
    if (n_sim_steps_per_sample_time_ > 1)
    { // Multi-step:
      step_msg.set_multi_step(n_sim_steps_per_sample_time_);
    }
    else
    { // One-step:
      step_msg.set_step(n_sim_steps_per_sample_time_ == 1);
    }
    world_control_pub_->Publish(step_msg, true);

    // begin the new loop
    cycle_begin_time_ = ros::Time::now();

    // Publish done signal
    custom_robot_msgs::BoolHeadered done_msg;
    done_msg.header.stamp = cycle_begin_time_;
    done_msg.data = true;
    pub_start_next_cycle_.publish(done_msg);
  } catch (const std::exception &exc) {
    ROS_ERROR_STREAM("Exception in SimulationTimeControl::stepSimControl: " << exc.what());
  }
}
} // namespace simulation_time_control
