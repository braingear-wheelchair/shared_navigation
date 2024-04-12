#ifndef SHARED_NAVIGATION_SHAREDDYNAMICS_CPP
#define SHARED_NAVIGATION_SHAREDDYNAMICS_CPP

#include "shared_navigation/SharedNavigation.hpp"

namespace shared_navigation {

SharedNavigation::SharedNavigation(void) : p_nh_("~") {

  this->n_rate_ = nullptr;

  // Configure node
  this->configure();

  // Dynamic reconfiguration
  this->n_dynreconfig_function_ = boost::bind(&SharedNavigation::reconfigure_callback, this, _1, _2);
  this->n_dynreconfig_server_.setCallback(this->n_dynreconfig_function_);

  // Initialize subscribers
  this->s_repellors_  = this->p_nh_.subscribe(this->t_repellors_, 1, &SharedNavigation::on_received_repellors, this);
  this->s_attractors_ = this->p_nh_.subscribe(this->t_attractors_, 1, &SharedNavigation::on_received_attractors, this);
  // Initialiaze publishers
  this->p_velocity_   = this->p_nh_.advertise<geometry_msgs::Twist>(this->t_velocity_, 1);
  this->p_partial_velocity_ = this->p_nh_.advertise<geometry_msgs::PoseArray>("pt_velocity", 1);
  this->p_angular_directions_ = this->p_nh_.advertise<geometry_msgs::PoseArray>("pt_angular_directions", 1);

  // Initialize services
  this->srv_enable_ = this->p_nh_.advertiseService("navigation_enable", &SharedNavigation::on_requested_enable, this);
  this->srv_disable_ = this->p_nh_.advertiseService("navigation_disable", &SharedNavigation::on_requested_disable, this);
  this->srv_start_ = this->p_nh_.advertiseService("navigation_start", &SharedNavigation::on_requested_start, this);
  this->srv_stop_  = this->p_nh_.advertiseService("navigation_stop", &SharedNavigation::on_requested_stop, this);
}

SharedNavigation::~SharedNavigation(void) {
  if(this->n_rate_ != nullptr)
    delete this->n_rate_;
}

bool SharedNavigation::configure(void) {

  // Default topics
  this->t_repellors_  = "/proximity_grid/repellors";
  this->t_attractors_ = "/proximity_grid/attractors"; 
  this->t_velocity_   = "/cmd_vel";

  // Getting parameters
  this->p_nh_.param<bool>("enable_repellors",	 this->n_enable_repellors_, true);
  this->p_nh_.param<bool>("enable_attractors", this->n_enable_attractors_, true);
  this->p_nh_.param<float>("update_rate",	this->n_update_rate_, 10.0f);
  this->p_nh_.param<float>("publish_frequency",	this->publish_frequency_, 10.0f);

  // Robot parameters
  this->p_nh_.param<std::string>("base_frame", this->base_frame_, "base_link");
  this->p_nh_.param<float>("size", this->size_, 0.75f);

  // Velocity Dynamics parameters 
  this->p_nh_.param<float>("safe_distance_front", this->safe_distance_front_, 0.1f);
  this->p_nh_.param<float>("safe_distance_lateral", this->safe_distance_lateral_, 0.0f);

  this->p_nh_.param<float>("angular_velocity_min", this->dyn_angular_velocity_min_, 0.0f);
  this->p_nh_.param<float>("angular_velocity_max", this->dyn_angular_velocity_max_, 0.3f);

  this->p_nh_.param<float>("linear_velocity_min", this->dyn_linear_velocity_min_, 0.10f);
  this->p_nh_.param<float>("linear_velocity_max", this->dyn_linear_velocity_max_, 0.15f);

  this->p_nh_.param<float>("angular_repellors_strength", this->dyn_angular_repellors_strength_, 0.5f);
  this->p_nh_.param<float>("angular_repellors_decay", this->dyn_angular_repellors_decay_, 0.8f);
  this->p_nh_.param<float>("angular_attractors_strength", this->dyn_angular_attractors_strength_, 1.0f); // TODO: remove this, no more used
  this->p_nh_.param<float>("linear_velocity_decay", this->dyn_linear_velocity_decay_, 1.0f); // TODO: remove this, no more used 
  this->p_nh_.param<float>("target_duration", this->target_duration_, 5.0f); // TODO: remove this, no more used

  // Initialize the angles of the robot
  // TODO: This should be a parameter
  this->vertices_x_ = {0.65,-0.50,-0.50, 0.65}; // [m]
  this->vertices_y_ = {0.35, 0.35,-0.35,-0.35}; // [m]

  // Initialize the radius of the robot
  this->vertices_r_.resize(this->n_vertices_);
  this->vertices_t_.resize(this->n_vertices_);

  // Compute the radius and the angle of each vertex
  for (int index_ver = 0; index_ver < this->n_vertices_; index_ver++) {
    vertices_r_[index_ver] = std::sqrt(std::pow(vertices_x_[index_ver], 2) + std::pow(vertices_y_[index_ver], 2));
    vertices_t_[index_ver] = std::atan2(vertices_y_[index_ver], vertices_x_[index_ver]);
  }

  this->target_ = 0.0f;

  // Initialize boolean states
  this->is_data_available_ = false;

  // Initialize boolean
  this->is_running_ = false;
  this->Enable();

  // Initialize update rate
  this->init_update_rate(this->n_update_rate_);

  // Create publish timer
  this->publish_timer_ = this->p_nh_.createTimer(ros::Duration(1.0f/this->publish_frequency_), &SharedNavigation::on_publish_velocity, this);

  // Create target timer
  this->target_timer_ = this->nh_.createTimer(ros::Duration(this->target_duration_), &SharedNavigation::on_target_elapsed, this);
  this->target_timer_.stop();

  return true;
}

void SharedNavigation::Run(void) {

  while(this->nh_.ok()) {

    if(this->IsEnabled() == true) {
      // Compute velocity
      this->MakeVelocity();
    }

    ros::spinOnce();
    this->n_rate_->sleep();
  }
}

void SharedNavigation::MakeVelocity(void) {

  float vangular;
  float c_nearest_obstacle = std::numeric_limits<float>::max();
  float vangular_r  = 0.0f;
  float vangular_a  = 0.0f;
  float vlinear     = 0.0f;
  float vlinear_r   = 0.0f;
  float vlinear_a   = 0.0f;
  float vangular_limited, vangular_scaled;

  // Set the nearest obstacle to infinity
  this->nearest_obstacle_ = std::numeric_limits<float>::max();

  std::vector<float> force_attractors = {0.0f, 0.0f};
  std::vector<float> force_repellors  = {0.0f, 0.0f};

  if(this->n_enable_attractors_ == true) {
    // Here there should be a choice, get an attractor directly or
    // use the "smart" selector, as now it will only use the second option
    force_attractors = this->get_force_attractors();
    // Now is needed to compensate that attractor is one, but the force is four
    //force_attractors[0] *= 4.0f; // TODO: update this to a better implementation such as a parameter
  }

  if(this->n_enable_repellors_ == true) {
    // This should be always true n.d.r.
    force_repellors = this->get_force_repellors();
  }

  ROS_DEBUG("Force repellors: [%f, %f]", force_repellors[0], force_repellors[1]);
  ROS_DEBUG("Force attractors: [%f, %f]", force_attractors[0], force_attractors[1]);

  // The minus in the force is due to the orientation of the robot (reversed)
  std::vector<float> final_force_theta = {-force_repellors[1], -force_attractors[1]};
  std::vector<float> final_force_d     = {force_repellors[0], force_attractors[0]};

  std::vector<float> final_force = this->sum_forces(final_force_theta, final_force_d);

  // Publish partial value of the sum force
  this->publish_partial_velocity({-final_force[1], -final_force_theta[0], -final_force_theta[1]});

  // Compute the final cmd
  std::vector<float> final_cmd = compute_local_potential(final_force[0], final_force[1]);

  if (this->n_enable_attractors_) {
    vlinear = final_cmd[0];

    c_nearest_obstacle = this->get_nearest_obstacle_depending_on_direction(final_cmd[1]);
    vlinear = get_linear_depending_on_distance(c_nearest_obstacle);

    //vlinear = get_linear_depending_on_distance(this->nearest_obstacle_);
    // vlinear = std::min(vlinear, final_cmd[0]);
  }else {
    c_nearest_obstacle = this->get_nearest_obstacle_depending_on_direction(final_cmd[1]);
    vlinear = get_linear_depending_on_distance(c_nearest_obstacle);
  }
  vangular = final_cmd[1];

  // Just in case set velocity to 0.0f if they are setted to nan or inf
  vlinear  = this->regulate_velocity(vlinear);
  vangular = this->regulate_velocity(vangular);

  // Limit the output velocity between -max and max
  vangular_limited = this->limit_range_value(vangular, 
                                            -this->dyn_angular_velocity_max_,
                                            this->dyn_angular_velocity_max_);

  // Scale the output velocity assuming that the absolute value of the input
  // is between [0 max] and the scaled range must be between [velocity_min
  // velocity_max]
  vangular_scaled  = this->scale_range_value(vangular_limited, 0.0f, 
                                            this->dyn_angular_velocity_max_,
                                            this->dyn_angular_velocity_min_, 
                                            this->dyn_angular_velocity_max_);

  // Now do the same with the linear velocity
  float vlinear_limited = this->limit_range_value(vlinear,
                                                  -this->dyn_linear_velocity_max_,
                                                  this->dyn_linear_velocity_max_);

  // Set to 0 the velocity if the input data is not available
  if(!this->is_data_available_){
    vlinear_limited = 0.0f;
    vangular_scaled = 0.0f;
  }

  // TODO: check this, due to the reversed orientation of the robot
  // --------------------------------------------------------------------------
  if (vlinear_limited < 0.0f) {
    vangular_scaled = -vangular_scaled;
  }
  // --------------------------------------------------------------------------

  // Fill the Twist message
  //this->velocity_.linear.x  = 0.0;
  this->velocity_.linear.x  = vlinear_limited;
  this->velocity_.linear.y  = 0.0;
  this->velocity_.linear.z  = 0.0;
  this->velocity_.angular.x = 0.0;
  this->velocity_.angular.y = 0.0;
  this->velocity_.angular.z = -vangular_scaled;
}

float SharedNavigation::get_linear_depending_on_distance(float distance) {
  // I do not care for now to which is the correct distance
  float safe_distance = std::min(this->safe_distance_front_, this->safe_distance_lateral_);
  float vlinear = 0.0f;

  // Be sure that the distance is in the correct format
  distance = std::fabs(distance);

  if (distance > safe_distance) {
    // IF the distance greater put the velocity similar to the distance
    // - Fast if the obstacle are far away
    // - Slow if the obstacle are close

    vlinear = this->dyn_linear_velocity_decay_ * std::pow(distance - safe_distance, 2.0f);
  } 

  // Check infinity, if that put to max velocity
  if (std::isinf(vlinear)) {
    vlinear = this->dyn_linear_velocity_max_;
  }

  if (this->target_ > M_PI * (2.0f/3.0f) || this->target_ < -M_PI * (2.0f/3.0f)) {
    // Invert the linear velocity
    vlinear = -vlinear;
  }

  return vlinear;
}

float SharedNavigation::get_nearest_obstacle_depending_on_direction(float direction) {

  float min_distance = std::numeric_limits<float>::infinity();

  proximitygrid::ProximityGridConstIt it;

  float c_distance, c_theta;
  float r_distance, r_theta;

  // Now for each vertex of the robot check the nearest obstacle in that direction
  for (int index_ver = 0; index_ver < this->n_vertices_; index_ver++) {

    for (it = this->pr_repellors_.Begin(); it != this->pr_repellors_.End(); it++) {
      c_distance = this->pr_repellors_.GetSectorValue(it);

      if(std::isnan(c_distance) || std::isinf(c_distance))
        continue;

      c_theta    = this->pr_repellors_.GetSectorAngle(it);

      r_distance = SharedNavigation::compute_distance(
          this->vertices_r_[index_ver],
          c_distance,
          this->vertices_t_[index_ver],
          c_theta + direction);
      r_theta    = SharedNavigation::compute_theta_first(
          c_distance,
          c_theta,
          this->vertices_r_[index_ver],
          this->vertices_t_[index_ver]);

      if ( is_angle_in_range(r_theta, direction, this->pr_repellors_.GetAngleIncrement()) ) {
        if (r_distance < min_distance) {
          min_distance = r_distance;
        }
      }

    }

  }

  return min_distance;
}

float SharedNavigation::is_angle_in_range(float angle, float target, float range) {
  return (angle > target - range && angle < target + range);
}

float SharedNavigation::regulate_velocity(float v) {
  if ( std::isnan(v) || std::isinf(v) )
    return 0.0f;
  return v;
}

bool SharedNavigation::IsEnabled(void) {
  return this->is_enabled_;
}

void SharedNavigation::Disable(void) {
  this->is_enabled_ = false;
  this->Stop();
  ROS_WARN("[SharedNavigation] Node has been disabled");
}

void SharedNavigation::Enable(void) {
  this->is_enabled_ = true;
  ROS_WARN("[SharedNavigation] Node has been enabled");
}

bool SharedNavigation::IsRunning(void) {
  return this->is_running_;
}

void SharedNavigation::Start(void) {
  this->is_running_ = true;
  ROS_WARN("[SharedNavigation] Node has been started");
}

void SharedNavigation::Stop(void) {
  this->is_running_ = false;
  this->velocity_.linear.x  = 0.0f;
  this->velocity_.linear.y  = 0.0f;
  this->velocity_.linear.z  = 0.0f;
  this->velocity_.angular.x = 0.0f;
  this->velocity_.angular.y = 0.0f;
  this->velocity_.angular.z = 0.0f;
  this->p_velocity_.publish(this->velocity_);
  ROS_WARN("[SharedNavigation] Node has been stopped");
}

std::vector<float> SharedNavigation::get_smart_attractor_sector(float attractor_distance, float attractor_angle) {
  // The required direction (in this setup the one provided by the proximitygrid)
  // is confronted with the set of sector in the repellors
  // Then a smart rule is applied. The rule is defined as follow:
  // - If the target sector is empty put the attractor in it at d distance
  // - If the sector is not empty, but the distance is such that you can be there put
  //   the attractor in it with distance according to it (dobstacle-dsafty)
  // - If the sector is not empty and the distance do not allow to put a attractor in
  //   a safe distance put the attractor in the closest sector that allows it
  //
  // The output will be a vector in form of { distance, angle }

  std::vector<float> attractor_sector = {0.0f, 0.0f};
  
  proximitygrid::ProximityGridConstIt repellor_grid_it; 
  float distance = 0.0f;
  float angle    = 0.0f;

  bool found = false;

  repellor_grid_it = this->pr_repellors_.Begin();

  while (!found && repellor_grid_it != this->pr_repellors_.End()) {
    distance = this->pr_repellors_.GetSectorValue(repellor_grid_it);
    angle    = this->pr_repellors_.GetSectorAngle(repellor_grid_it);

    // Arrive to the target decision 
    if (angle < attractor_angle) {
      repellor_grid_it++;
    } else {
      // Now we are in the correct sector
      found = true;

      // Now check if we are in the case 1 or
      if (distance > attractor_distance) {
        // The sector is ok return it with the correct distance
        attractor_sector[0] = attractor_distance;
        attractor_sector[1] = angle;
      } else if (distance > this->size_/2.0f * this->safe_distance_lateral_) {
        // Then the sector is not full ok, but is easy fixable
        attractor_sector[0] = this->size_/2.0f + this->safe_distance_lateral_;
        attractor_sector[1] = angle;
      } else {
        // Here there is the real tread
        // Now is need to check the sector near the target
        // for now try with recursion
        if (attractor_angle > 0.0f) {
          attractor_sector = get_smart_attractor_sector(attractor_distance, attractor_angle - this->pr_repellors_.GetAngleIncrement());
        } else if (attractor_angle < 0.0f) {
          attractor_sector = get_smart_attractor_sector(attractor_distance, attractor_angle + this->pr_repellors_.GetAngleIncrement());
        } else {
          // Here there is the problem, what is the base case? For now set the sector to 0.0 and hope this situation will be solved
          // By the user or by the environment
          // TODO: check what to do in this case
          attractor_sector[0] = 1.0f;
          attractor_sector[1] = 0.0f;
        }
      }
    }
  }

  // TODO: check the case if there the distance will set 1/d. Need to check
    
  return attractor_sector;
  
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

float median(std::vector<float> medi) 
{
  if (medi.size() == 0) 
    return 0;

    sort(medi.begin(), medi.end());     // sort temperatures
            
    float tmedian;
    if (medi.size() % 2 == 0)           // even
        tmedian = (medi[medi.size() / 2 - 1] + medi[medi.size() / 2]) / 2;
    else                                // odd
        tmedian = medi[medi.size() / 2];
    
    return tmedian;
}

std::vector<float> SharedNavigation::get_force_attractors() {
  proximitygrid::ProximityGrid data = this->pr_attractors_;
  proximitygrid::ProximityGridConstIt it;

  // Sum the force for each attractor
  std::vector<float> final_force = {0.0f, 0.0f};
  std::vector<float> partial_force = {0.0f, 0.0f};
  float distance;
  float angle;

  std::vector<float> distances;
  std::vector<float> angles;

  // First convert all the attractor to forces
  for (it = data.Begin(); it != data.End(); ++it) {
    distance = data.GetSectorValue(it);
    angle    = data.GetSectorAngle(it);

    if (std::isinf(distance) == true || std::isnan(distance) == true) {
      continue;
    }

    // Use the "smart" attractor system to put the attractor in the right sector
    partial_force = SharedNavigation::get_smart_attractor_sector(distance, angle);

    if (partial_force[0] == 0.0f) {
      continue;
    }

    // Put the distance as (1/d) in order to be compatible with the attractors
    distances.push_back(this->convert_to_pf(partial_force[0]));
    angles.push_back(partial_force[1]);

  }
  // Second to sum the forces into a final one and return it
  final_force = SharedNavigation::sum_forces(angles, distances);

  return final_force;
}

float SharedNavigation::convert_to_pf(float distance) {
  float cdtheta = this->pr_repellors_.GetAngleIncrement();
  float csigma = 0.0f;

  float base_size = this->size_/2.0f + this->safe_distance_lateral_;

  csigma = std::atan( std::tan(cdtheta/2.0f) + (base_size) / (base_size + distance) );

  return csigma;
}

std::vector<float> SharedNavigation::get_force_repellors() {


  /*
   * The setup is the following:
   *  - The x axis is aligned with the front of the robot
   *
   *          y
   *   _______|____
   *  |       |    |
   *  |       |    |
   *  --------|------> x
   *  |       |    |
   *  |_______|____|
   *          |
   */

  try {

  proximitygrid::ProximityGrid data = this->pr_repellors_;

  proximitygrid::ProximityGridConstIt	it;
  float distance, angle;
  float clambda, csigma, cdtheta;
  float cw;
  float w = 0.0f;

  float robot_width           = this->size_;
  float safe_distance_front   = this->safe_distance_front_;
  float safe_distance_lateral = this->safe_distance_lateral_;

  const int n_vertices = 4; // TODO: find a way to surpass this limitation

  std::vector<float> vertices_r = this->vertices_r_;
  std::vector<float> vertices_t = this->vertices_t_;

  float hangle = 0.0;

  float distance_obj_vert = 0.0;
  float angle_obj_vert = 0.0;

  // Save the current state of the vertices
  std::array<std::list<float>, n_vertices> vertices;

  // For each vertex compute the potential field
  std::array<float, (n_vertices + 1)> w_verts;
  for (int index_ver = 0; index_ver <= n_vertices; index_ver++) {
    w_verts[index_ver] = 0.0f;
  }

  for (int index_ver = 0; index_ver < n_vertices; index_ver++) {
    vertices[index_ver].clear();
  }

  // Initialize the void partial velocities
  this->clear_partial_velocity();

  std::vector<std::vector<float>> objs_theta;
  std::vector<std::vector<float>> objs_weights;

  std::vector<float> partial_weights;
  std::vector<float> partial_angles;

  for (int index_ver = 0; index_ver < n_vertices; index_ver++) {
    objs_theta.push_back(std::vector<float>());
    objs_weights.push_back(std::vector<float>());
  }

  // Iterate over the sectors
  for(it=data.Begin(); it!=data.End(); ++it) {

    // Get current distance and angle
    distance  = data.GetSectorValue(it);
    angle     = data.GetSectorAngle(it);

    // First determine if is a valid sector
    if(std::isinf(distance) == true || std::isnan(distance) == true )
      continue;

    // Now shift the sector to the corner of the robot
    for (int index_ver = 0; index_ver < n_vertices; index_ver++) {

      // Compute the new distance
      distance_obj_vert = SharedNavigation::compute_distance(vertices_r[index_ver], distance, vertices_t[index_ver], angle);
      // Compute the new angle
      angle_obj_vert = SharedNavigation::compute_theta_first(distance, angle, vertices_r[index_ver], vertices_t[index_ver]);
      // TODO: check the sign of the angle
      hangle = SharedNavigation::compute_hangle(angle, angle_obj_vert);

      // If this is the shortest distance
      if (distance_obj_vert < this->nearest_obstacle_) {
        this->nearest_obstacle_ = distance_obj_vert;
      }

      // Fixed sector angle
      //cdtheta = data.GetAngleIncrement();

      // Compute current sigma
      csigma  = this->convert_to_pf(distance_obj_vert);
      //  - std::atan( std::tan(cdtheta/2.0f) +
      //        + (robot_width + safe_distance_lateral)/(robot_width + safe_distance_lateral + distance_obj_vert ) );

      objs_theta[index_ver].push_back( hangle );
      objs_weights[index_ver].push_back( csigma );

      // Set the partial velocities
      this->add_partial_velocity(vertices_x_[index_ver], vertices_y_[index_ver], hangle);

    }
  }

  for (int index_ver = 0; index_ver < n_vertices; index_ver++) {
    std::vector<float> tmp = this->sum_forces(objs_theta[index_ver], objs_weights[index_ver]);
    partial_angles.push_back( tmp[1] );
    partial_weights.push_back( tmp[0] );
  }
 
  // Add the compute w to the visual output (rviz)
  for (int index_ver = 0; index_ver < n_vertices; index_ver++) {
    this->add_angular_directions(vertices_x_[index_ver], vertices_y_[index_ver], partial_angles[index_ver]);
  }

  std::vector<float> final_force = this->sum_forces(partial_angles, partial_weights);
  this->add_angular_directions(0.0f, 0.0f, final_force[1]);

  return final_force;

  } catch (std::exception& e) {
    ROS_ERROR("SharedNavigation: %s", e.what());
    return {0.0f, 0.0f};
  }
  
}

std::vector<float> SharedNavigation::compute_local_potential(float d, float theta) {

  float clambda = this->dyn_angular_repellors_strength_ *
    exp(-( ((1.0f/d) - safe_distance_front_)/this->dyn_angular_repellors_decay_));

  if( (1.0f/d) < (safe_distance_front_) )
    clambda = this->dyn_angular_repellors_strength_;

  // invert the cmd according to the sign of the merged singal
  if (theta > M_PI/2.0f || theta < -M_PI/2.0f) {
    theta =  sgn(theta) * (M_PI - std::abs(theta));
    d = d * -1.0f;
  }

  float potential = clambda * theta * std::exp(-(std::pow(theta, 2))/(2.0f*pow( d , 2)));

  if (std::isnan(potential) == true)
    potential = 0.0f;

  if (std::isnan(d) == true)
    d = 0.0f;

  return {d, potential};

}

std::vector<float> SharedNavigation::sum_forces(std::vector<float> forces_angle, std::vector<float> forces_distance) {

  std::vector<float> forces_x;
  std::vector<float> forces_y;

  for (int index_forces = 0; index_forces < forces_angle.size(); index_forces++) {
    forces_x.push_back(std::cos(forces_angle[index_forces]) * forces_distance[index_forces]);
    forces_y.push_back(std::sin(forces_angle[index_forces]) * forces_distance[index_forces]);
  }

  float sum_x = std::accumulate(forces_x.begin(), forces_x.end(), 0.0f);
  float sum_y = std::accumulate(forces_y.begin(), forces_y.end(), 0.0f);

  float final_theta = 0.0f;
  float final_distance = 0.0f;

  final_distance = std::sqrt(std::pow(sum_x, 2) + std::pow(sum_y, 2));

  final_theta = std::atan2(sum_y, sum_x);

  if (std::isnan(final_theta) == true)
    final_theta = 0.0f;

  return std::vector<float>{final_distance, final_theta};
}

void SharedNavigation::clear_partial_velocity(void) {
  this->partial_velocity_ = geometry_msgs::PoseArray();
  this->partial_velocity_.header.stamp = ros::Time::now();
  this->partial_velocity_.header.frame_id = this->base_frame_;

  this->angular_directions_ = geometry_msgs::PoseArray();
  this->angular_directions_.header.stamp = ros::Time::now();
  this->angular_directions_.header.frame_id = this->base_frame_;
}

void SharedNavigation::publish_partial_velocity(std::vector<float> partial_velocities) {

  for (int index_ver = 0; index_ver < partial_velocities.size(); index_ver++) {
    this->add_angular_directions(0.0f, 0.0f, partial_velocities[index_ver]);
  }

  this->p_partial_velocity_.publish(this->partial_velocity_);
  this->p_angular_directions_.publish(this->angular_directions_);
}

void SharedNavigation::add_partial_velocity(float x, float y, float w) {
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation = tf::createQuaternionMsgFromYaw(w);
  this->partial_velocity_.poses.push_back(pose);
}

void SharedNavigation::add_angular_directions(float x, float y, float w) {
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation = tf::createQuaternionMsgFromYaw(w);
  this->angular_directions_.poses.push_back(pose);
}

float SharedNavigation::compute_hangle(float theta, float theta_first) {
  float hangle = (theta) - theta_first;

  if (hangle > M_PI)
    hangle = hangle - 2*M_PI;
  else if (hangle < -M_PI)
    hangle = hangle + 2*M_PI;

  return hangle;
}

float SharedNavigation::compute_distance(float r_1, float r_2, float theta_1, float theta_2) {
  return std::sqrt( (r_1*r_1) + (r_2*r_2) - 2*r_1*r_2*std::cos(theta_1 - theta_2) );
}

float SharedNavigation::compute_theta_first(float d, float theta, float r, float phi) {

  float new_x = d * std::cos(theta) - r * std::cos(phi);
  float new_y = d * std::sin(theta) - r * std::sin(phi);

  return std::atan2(new_y, new_x);

}

float SharedNavigation::get_angular_velocity_attractors(float angle) {

  //float forcelet;

  //forcelet = -tau*std::sin(phi - psi);
  float w;
  float hangle = 0.0f; // direzione della sedia rispetto ai dati

  w =  this->dyn_angular_attractors_strength_*std::sin(hangle - angle);

  //printf("angle attractor: %3.2f | angular velocity attractor: %3.2f\n", angle, w );
  return w;
}

float SharedNavigation::get_linear_velocity_repellors(proximitygrid::ProximityGrid& data) {

  proximitygrid::ProximityGridConstIt  it;
  float distance, angle;
  float min_angle, min_distance;
  float safe_distance_front;
  float robot_width;
  float sangle;
  float x;
  float y;

  x = std::numeric_limits<float>::infinity();

  robot_width         = this->size_ + 2.0f*this->safe_distance_lateral_;
  safe_distance_front = this->safe_distance_front_;

  // Iterate over the sectors
  for(it=data.Begin(); it!=data.End(); ++it) {

    distance = data.GetSectorValue(it);
    angle    = data.GetSectorAngle(it);

    // Discard empty sectors
    if(std::isinf(distance) == true || std::isnan(distance) == true )
      continue;

    // Check if the repellor is inside the projection with respect to the
    // current wheelchair heading direction (angle=0.0). If it is outside,
    // then discard the repellor. Correct the current angle for standard
    // coordinates.
    if(this->is_projection_inside(distance, angle+M_PI/2.0f, robot_width) == false)
      continue;

    // If distance is smaller then the previous one, select this distance
    if(distance < x) {
      x = distance;
      min_angle = angle;
    }
  }

  // If distance smaller the safe distance, then fixed negative velocity.
  // Otherwise compute the velocity according to exponential function
  y = -0.02f; // TODO instead to set as fixed negative put into a 'safe' direction
  if (x > safe_distance_front) {
    y = this->dyn_linear_velocity_max_*
        (1.0f - std::exp(- (x - safe_distance_front)/this->dyn_linear_velocity_decay_ ));
  }

  //if (min_angle < -M_PI/2.0f || min_angle > M_PI/2.0f) {
  //  y = -y;
  //}

  return y;
}

float SharedNavigation::get_linear_velocity_attractors(proximitygrid::ProximityGrid& data) {
  return 0.0f;
}

bool SharedNavigation::is_projection_inside(float distance, float angle, float width) {

  float projection;

  projection = std::fabs(distance*cos(angle));

  if(projection > width/2.0f)
    return false;

  return true;
}

void SharedNavigation::on_publish_velocity(const ros::TimerEvent& event) {

  if (this->IsEnabled() == true && this->IsRunning() == true) {
    this->p_velocity_.publish(this->velocity_);

    ROS_DEBUG_NAMED("velocity", "Published velocity: v=%3.2f [m/s], o=%3.2f [deg/s]",
                    this->velocity_.linear.x, this->rad2deg(this->velocity_.angular.z));
  }
}

void SharedNavigation::on_target_elapsed(const ros::TimerEvent& event) {

  this->target_ = 0.0f;
  this->target_timer_.stop();
  ROS_INFO("Target duration elapsed (%3.2f s)", this->target_duration_);
}

float SharedNavigation::scale_range_value(float x, float minx, float maxx, float miny, float maxy) {

  float absx;
  float signx = 1.0f;
  float y;

  if(x < 0)
    signx = -signx;

  absx = std::fabs(x);

  y = signx*((maxy - miny)*( (absx - minx) / (maxx - minx) ) + miny);

  return y;
}

float SharedNavigation::limit_range_value(float x, float minx, float maxx) {

  float lx;

  lx = x;
  if(lx > maxx) 
    lx = maxx;
  else if (lx < minx)
    lx = minx;

  return lx;
}

void SharedNavigation::on_received_repellors(const proximity_grid::ProximityGridMsg& data) {

  // Set true the availability of repellors data (used for first iteration)
  ROS_WARN_ONCE("First proximity grid messaged received! Repellors data available.");
  this->is_data_available_ = true;

  // Convert and store repellor data
  if(proximitygrid::ProximityGridConverter::FromMessage(data, this->pr_repellors_) == false) {
    ROS_ERROR("Cannot convert repellor proximity grid message");
  }
}

void SharedNavigation::on_received_attractors(const proximity_grid::ProximityGridMsg& data) {

  proximitygrid::ProximityGridConstIt  it;
  proximitygrid::ProximityGrid         grid;

  // Convert and store attractor data
  if(proximitygrid::ProximityGridConverter::FromMessage(data, grid) == false) {
    ROS_ERROR("Cannot convert attractors proximity grid message");
  }

  // Iterate over the sectors of the grid
  for(it = grid.Begin(); it != grid.End(); ++it) {

    // Discard empty sectors
    if(std::isinf(grid.GetSectorValue(it)) == true || std::isnan(grid.GetSectorValue(it)) == true )
      continue;

    this->target_ = grid.GetSectorAngle(it) + grid.GetAngleIncrement()/2.0f;
    //this->target_timer_.stop();
    //this->target_timer_.start();
  } 

  // ROS_INFO("Target received: %1.2f", this->target_);

  // My code
  if(proximitygrid::ProximityGridConverter::FromMessage(data, this->pr_attractors_) == false) {
    ROS_ERROR("Cannot convert attractors proximity grid message");
  }

}

void SharedNavigation::reconfigure_callback(shared_navigation::SharedNavigationConfig &config, 
                                            uint32_t level) {

  // Angular minimum velocity
  if(this->update_if_different(config.robot_size, this->size_))
    ROS_WARN("Updated robot size to %f [m]", this->size_);

  // Angular minimum velocity
  if(this->update_if_different(config.angular_velocity_min, this->dyn_angular_velocity_min_))
    ROS_WARN("Updated angular velocity minimum to %f", this->dyn_angular_velocity_min_);

  // Angular maximum velocity
  if(this->update_if_different(config.angular_velocity_max, this->dyn_angular_velocity_max_))
    ROS_WARN("Updated angular velocity maximum to %f", this->dyn_angular_velocity_max_);

  // Angular repellors strength
  if(this->update_if_different(config.angular_repellors_strength, this->dyn_angular_repellors_strength_))
    ROS_WARN("Updated angular repellors strength to %f", this->dyn_angular_repellors_strength_);

  // Angular repellors decay
  if(this->update_if_different(config.angular_repellors_decay, this->dyn_angular_repellors_decay_))
    ROS_WARN("Updated angular repellors decay to %f", this->dyn_angular_repellors_decay_);

  // Angular attractors strength
  if(this->update_if_different(config.angular_attractors_strength, this->dyn_angular_attractors_strength_))
    ROS_WARN("Updated angular attractors strength to %f", this->dyn_angular_attractors_strength_);

  // Linear minimum velocity
  if(this->update_if_different(config.linear_velocity_min, this->dyn_linear_velocity_min_))
    ROS_WARN("Updated linear velocity minimum to %f", this->dyn_linear_velocity_min_);

  // Linear maximum velocity
  if(this->update_if_different(config.linear_velocity_max, this->dyn_linear_velocity_max_))
    ROS_WARN("Updated linear velocity maximum to %f", this->dyn_linear_velocity_max_);

  // Linear velocity decay
  if(this->update_if_different(config.linear_velocity_decay, this->dyn_linear_velocity_decay_))
    ROS_WARN("Updated linear velocity decay to %f", this->dyn_linear_velocity_decay_);

  // Linear safe distance
  if(this->update_if_different(config.safe_distance_front, this->safe_distance_front_))
    ROS_WARN("Updated robot safe front distance to %f", this->safe_distance_front_);

  // Linear safe distance
  if(this->update_if_different(config.safe_distance_lateral, this->safe_distance_lateral_))
    ROS_WARN("Updated robot safe lateral distance to %f", this->safe_distance_lateral_);

  // Update rate
  if(this->update_if_different(config.update_rate, this->n_update_rate_)) {
    ROS_WARN("Updated rate to %f", this->n_update_rate_);
    this->init_update_rate(this->n_update_rate_);
  }

  // Target duration
  if(this->update_if_different(config.target_duration, this->target_duration_)) {
    ROS_WARN("Updated target duration to %f", this->target_duration_);
    this->target_timer_.setPeriod(ros::Duration(this->target_duration_), true);
  }

  // Publish frequency
  if(this->update_if_different(config.publish_frequency, this->publish_frequency_)) {
    ROS_WARN("Updated publish frequency to %f", this->publish_frequency_);
    this->publish_timer_.setPeriod(ros::Duration(1.0f/this->publish_frequency_), true);
  }
}

float SharedNavigation::rad2deg(float radians) {
  return radians*180.0f/M_PI;
}

void SharedNavigation::init_update_rate(float rate) {
  if(this->n_rate_ != nullptr)
    delete this->n_rate_;

  this->n_rate_ = new ros::Rate(rate);
}

bool SharedNavigation::update_if_different(const float& first, float& second, float epsilon) {

  bool is_different = false;
  if(std::fabs(first - second) >= epsilon) {
    second = first;
    is_different = true;
  }

  return is_different;
}

bool SharedNavigation::on_requested_enable(std_srvs::Empty::Request& req,
                                           std_srvs::Empty::Response& res) {

  if(this->IsEnabled() == false) {
    this->Enable();
  }
  return true;
}

bool SharedNavigation::on_requested_disable(std_srvs::Empty::Request& req,
                                            std_srvs::Empty::Response& res) {
  if(this->IsEnabled() == true) {
    this->Disable();
  }
  return true;
}

bool SharedNavigation::on_requested_start(std_srvs::Empty::Request& req,
                                          std_srvs::Empty::Response& res) {
  if(this->IsRunning() == false) {
    this->Start();
  }
  return true;
}

bool SharedNavigation::on_requested_stop(std_srvs::Empty::Request& req,
                                         std_srvs::Empty::Response& res) {
  if(this->IsRunning() == true) {
    this->Stop();
  }
  return true;
}

} // namespace shared_navigation

#endif
