#ifndef SHARED_NAVIGATION_HPP
#define SHARED_NAVIGATION_HPP

// System includes
#include <exception>
#include <cmath>
#include <list>

// ROS includes
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/server.h>

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

    float get_angular_velocity_repellors(proximitygrid::ProximityGrid& data);
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

    // Services
    ros::ServiceServer  srv_enable_;
    ros::ServiceServer  srv_disable_;
    ros::ServiceServer  srv_start_;
    ros::ServiceServer  srv_stop_;

    // Proximity grid
    proximitygrid::ProximityGrid   pr_repellors_;
    proximitygrid::ProximityGrid   pr_attractors_;
    // Not present because the attractors callback start the target timer

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

    // Velocity Dynamics
    float dyn_angular_velocity_min_;
    float dyn_angular_velocity_max_;
    float dyn_linear_velocity_min_;
    float dyn_linear_velocity_max_;

    float dyn_angular_repellors_strength_;
    float dyn_angular_attractors_strength_;
    float dyn_angular_repellors_decay_;
    float dyn_linear_velocity_decay_;

    float target_;
    float target_duration_;

    geometry_msgs::Twist  velocity_;
};
		
}

#endif
