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
	this->p_velocity_	   = this->p_nh_.advertise<geometry_msgs::Twist>(this->t_velocity_, 1);

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
	this->t_repellors_	= "/proximity_grid/repellors";
	this->t_attractors_	= "/proximity_grid/attractors"; 
	this->t_velocity_	= "/cmd_vel";

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
	this->p_nh_.param<float>("angular_attractors_strength", this->dyn_angular_attractors_strength_, 1.0f);
	this->p_nh_.param<float>("linear_velocity_decay", this->dyn_linear_velocity_decay_, 1.0f);
	this->p_nh_.param<float>("target_duration", this->target_duration_, 5.0f);

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
	float vangular_r	= 0.0f;
	float vangular_a	= 0.0f;
	float vlinear		= 0.0f;
	float vangular_limited, vangular_scaled;

	// Compute orientation for repellors
	if(this->n_enable_repellors_ == true) {
		vangular_r  = this->get_angular_velocity_repellors(this->pr_repellors_);
		ROS_DEBUG_NAMED("velocity_repellors", "Repellors angular velocity: %f [deg/s]", rad2deg(vangular_r));
	}
	
	// Compute orientation for attractors
	if(this->n_enable_attractors_ == true) {
		vangular_a = this->get_angular_velocity_attractors(this->target_);
		ROS_DEBUG_NAMED("velocity_attractors", "Attractors angular velocity: %f [deg/s]", rad2deg(vangular_a));
	}

	vangular = vangular_a - vangular_r;

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
	
	// Compute linear velocity (repellors based)
	if(this->is_data_available_ == true)
		vlinear = this->get_linear_velocity_repellors(this->pr_repellors_);


	// Fill the Twist message
	//this->velocity_.linear.x	= 0.0;
	this->velocity_.linear.x	= vlinear;
	this->velocity_.linear.y	= 0.0;
	this->velocity_.linear.z	= 0.0;
	this->velocity_.angular.x	= 0.0;
	this->velocity_.angular.y	= 0.0;
	this->velocity_.angular.z	= -vangular_scaled;
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
	this->velocity_.linear.x	= 0.0f;
	this->velocity_.linear.y	= 0.0f;
	this->velocity_.linear.z	= 0.0f;
	this->velocity_.angular.x	= 0.0f;
	this->velocity_.angular.y	= 0.0f;
	this->velocity_.angular.z	= 0.0f;
	this->p_velocity_.publish(this->velocity_);
	ROS_WARN("[SharedNavigation] Node has been stopped");
}

float SharedNavigation::get_angular_velocity_repellors(proximitygrid::ProximityGrid& data) {

	proximitygrid::ProximityGridConstIt	it;
	float distance, angle;
	float clambda, csigma, cdtheta;
	float cw;
	float w = 0.0f;

	float robot_width			= this->size_;
	float safe_distance_front	= this->safe_distance_front_;
	float safe_distance_lateral	= this->safe_distance_lateral_;

	float hangle = 0.0;

	
	// Iterate over the sectors
	for(it=data.Begin(); it!=data.End(); ++it) {
	
		// Get current distance and angle
		distance	= data.GetSectorValue(it);
		angle		= data.GetSectorAngle(it);
		
		// Discard empty sectors
		if(std::isinf(distance) == true || std::isnan(distance) == true )
			continue;

		
		// Compute the contribution to the velocity
		clambda = this->dyn_angular_repellors_strength_*
				  exp(-( (distance - safe_distance_front)/this->dyn_angular_repellors_decay_));

		// If distance is smaller than safe distance, maximum value of lambda
		if( distance < (safe_distance_front) )
			clambda = this->dyn_angular_repellors_strength_;

		// Fixed sector angle
		cdtheta = data.GetAngleIncrement();

		// Compute current sigma
		csigma  = std::atan( std::tan(cdtheta/2.0f) 
				  + (robot_width + safe_distance_lateral)/(robot_width + safe_distance_lateral + distance ) );

		// Compute current angular velocity for this sector
		cw = clambda*(hangle-angle)*std::exp(-(std::pow(hangle-angle, 2))/(2.0f*pow(csigma, 2)));

		// Update the total angular velocity
		w += cw;
	}

											
	return w;
}

float SharedNavigation::get_angular_velocity_attractors(float angle) {
	
	//float forcelet;

	//forcelet = -tau*std::sin(phi - psi);
	float w;
	float hangle = 0.0f;

	w =  this->dyn_angular_attractors_strength_*std::sin(hangle - angle);

	//printf("angle attractor: %3.2f | angular velocity attractor: %3.2f\n", angle, w );
	return w;
}

float SharedNavigation::get_linear_velocity_repellors(proximitygrid::ProximityGrid& data) {

	proximitygrid::ProximityGridConstIt	it;
	float distance, angle;
	float min_angle, min_distance;
	float safe_distance_front;
	float robot_width;
	float sangle;
	float x;
	float y;

	x = 6.0f;
	robot_width		    = this->size_ + 2.0f*this->safe_distance_lateral_;
	safe_distance_front = this->safe_distance_front_;

	// Iterate over the sectors
	for(it=data.Begin(); it!=data.End(); ++it) {
	
		distance = data.GetSectorValue(it);
		angle	 = data.GetSectorAngle(it);
		
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
	y = -0.2f;
	if (x > safe_distance_front) {
		y = this->dyn_linear_velocity_max_*
		    (1.0f - std::exp(- (x - safe_distance_front)/this->dyn_linear_velocity_decay_ ));
	}

	return y;
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

	float	absx;
	float	signx = 1.0f;
	float	y;

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

	proximitygrid::ProximityGridConstIt it;
	proximitygrid::ProximityGrid		 grid;
	float				 angle;

	angle = 0.0f;
	// Convert and store attractor data
	if(proximitygrid::ProximityGridConverter::FromMessage(data, grid) == false) {
		ROS_ERROR("Cannot convert attractors proximity grid message");
	}

	for(it = grid.Begin(); it != grid.End(); ++it) {

		// Discard empty sectors
		if(std::isinf(grid.GetSectorValue(it)) == true || std::isnan(grid.GetSectorValue(it)) == true )
			continue;

		this->target_ = grid.GetSectorAngle(it) + grid.GetAngleIncrement()/2.0f;
		this->target_timer_.stop();
		this->target_timer_.start();
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


}



#endif
