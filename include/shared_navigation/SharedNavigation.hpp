#ifndef SHARED_NAVIGATION_HPP
#define SHARED_NAVIGATION_HPP

// System includes
#include <exception>
#include <cmath>
#include <list>
#include <numeric>

// ROS includes
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/server.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

// Package includes
#include "proximity_grid/ProximityGrid.hpp"
#include "proximity_grid/ProximityGridMsg.h"
#include "proximity_grid/ProximityGridConverter.hpp"
#include "shared_navigation/SharedNavigationConfig.h"

namespace shared_navigation {

typedef dynamic_reconfigure::Server<shared_navigation::SharedNavigationConfig>	DynamicReconfig;

class SharedNavigation {

  public:
    SharedNavigation(void);
    virtual ~SharedNavigation(void);

    bool configure(void);
    void Run(void);

    void MakeVelocity(void);
    void SendVelocity(void);
    void SendVelocity(float x, float w);
    void ResetVelocity(void);

    bool IsEnabled(void);
    void Disable(void);
    void Enable(void);

    bool IsRunning(void);
    void Start(void);
    void Stop(void);

  private:

    // float get_angular_velocity_repellors(proximitygrid::ProximityGrid& data);
    float get_angular_velocity_attractors(float angle);
    float get_linear_velocity_repellors(proximitygrid::ProximityGrid& data);
    float get_linear_velocity_attractors(proximitygrid::ProximityGrid& data);
    bool  is_projection_inside(float distance, float angle, float width);

    // Callbacks
    void on_received_repellors(const proximity_grid::ProximityGridMsg& data);
    void on_received_attractors(const proximity_grid::ProximityGridMsg& data);
    void on_publish_velocity(const ros::TimerEvent& event);
    void on_target_elapsed(const ros::TimerEvent& event);
    void reconfigure_callback(shared_navigation::SharedNavigationConfig &config, uint32_t level);

    bool on_requested_enable(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool on_requested_disable(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool on_requested_start(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool on_requested_stop(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    // Utilities
    float rad2deg(float radians);
    bool  update_if_different(const float& first, float& second, float epsilon = 0.00001f);
    float limit_range_value(float x, float minx, float maxx);
    float scale_range_value(float x, float min_abs_x, float max_abs_x, float min_abs_y, float max_abs_y);
    float compute_distance(float r_1, float r_2, float theta_1, float theta_2);
    float compute_theta_first(float d, float theta, float r, float phi);
    float compute_hangle(float theta, float theta_first);
    std::vector<float> sum_forces(std::vector<float> forces_angle, std::vector<float> forces_distance);
    float get_linear_depending_on_distance(float distance);
    float get_nearest_obstacle_depending_on_direction(float direction);
    std::vector<float> get_smart_attractor_sector(float attractor_distance, float attractor_angle);
    float convert_to_pf(float distance);
    float is_angle_in_range(float angle, float target, float range);

    // Compute parts of the potential field
    std::vector<float> get_force_attractors();
    std::vector<float> get_force_repellors();

    // Utilities for partial velocity publications
    void clear_partial_velocity();
    void publish_partial_velocity(std::vector<float> partial_velocities);
    void add_partial_velocity(float x, float y, float w);
    void add_angular_directions(float x, float y, float w);
    std::vector<float> compute_local_potential(float d, float theta);
    float regulate_velocity(float v);

    void  init_update_rate(float rate);

  private:
    // Node handles
    ros::NodeHandle nh_;
    ros::NodeHandle p_nh_;

    // Topics
    std::string t_repellors_;
    std::string t_attractors_;
    std::string t_velocity_;

    // Subscribers
    ros::Subscriber s_repellors_;
    ros::Subscriber s_attractors_;

    // Publishers
    ros::Publisher  p_velocity_;
    ros::Publisher  p_angular_directions_;
    ros::Publisher  p_partial_velocity_;

    // Services
    ros::ServiceServer  srv_enable_;
    ros::ServiceServer  srv_disable_;
    ros::ServiceServer  srv_start_;
    ros::ServiceServer  srv_stop_;

    // Proximity grid
    proximitygrid::ProximityGrid   pr_repellors_;
    proximitygrid::ProximityGrid   pr_attractors_;

    // Dynamic reconfigure
    DynamicReconfig               n_dynreconfig_server_;
    DynamicReconfig::CallbackType n_dynreconfig_function_;

    // Robot
    std::string   base_frame_;
    float         size_;
    float         safe_distance_front_;
    float         safe_distance_lateral_;

    // General boolean states
    bool  is_data_available_;
    bool  is_enabled_;
    bool  is_running_;

    // General node variables
    bool        n_enable_repellors_;
    bool        n_enable_attractors_;
    float       n_update_rate_;
    ros::Rate*  n_rate_;
    ros::Timer  publish_timer_;
    ros::Timer  target_timer_;
    float       publish_frequency_;

    // Shape of the robot
    // TODO: Better implementation of n_vertices
    const int n_vertices_ = 4;
    std::vector<float> vertices_x_;
    std::vector<float> vertices_y_;
    std::vector<float> vertices_r_;
    std::vector<float> vertices_t_;

    // Partial Velocity
    geometry_msgs::PoseArray partial_velocity_;
    geometry_msgs::PoseArray angular_directions_;

    // Velocity Dynamics
    float dyn_angular_velocity_min_;
    float dyn_angular_velocity_max_;
    float dyn_linear_velocity_min_;
    float dyn_linear_velocity_max_;

    float dyn_angular_repellors_strength_;  // TODO: remove this, no more used
    float dyn_angular_attractors_strength_; // TODO: fuse this in a single variable
    float dyn_angular_repellors_decay_;
    float dyn_linear_velocity_decay_;       // TODO: remove this, no more used

    float target_;                          // TODO: remove this, no more used
    float target_duration_;                 // TODO: remove this, no more used

    float nearest_obstacle_;

    geometry_msgs::Twist  velocity_;
};
		
}

#endif
