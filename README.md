# shared_navigation
The package provides a shared control system for wheelchair navigation. The navigation system is based on potential field algorithms. It controls the linear and rotation velocity of the device.

#### Dependencies:
The package depends on:
- proximity_grid 

#### Usage:
``
rosrun shared_navigation navigation
``

#### Subscribed topics:
- ~/repellors [proximity_grid/ProximityGridMsg]
- ~/attractors [proximity_grid/ProximityGridMsg]

#### Published topic:
- /cmd_vel [geometry_msgs/Twist]

#### Parameters:
- enable_repellors [bool] (default: True)
- enable_attractors [bool] (default: True)
- update_rate [float] (default: 10.0 [Hz])
- publish_frequency [float] (default: 10.0 [Hz])
- base_frame [string] (default: base_link)
- size [float] (default: 0.75 [m])
- safe_distance_front [float] (default: 0.1 [m])
- safe_distance_lateral [float] (default: 0.0 [m])
- angular_velocity_min [float] (default: 0.0 [m/s])
- angular_velocity_max [float] (default: 0.3 [m/s])
- linear_velocity_min [float] (default: 0.1 [m/s])
- linear_velocity_max [float] (default: 0.15 [m/s])
- angular_repellors_stength [float] (default: 0.5)
- angular_repellors_decay [float] (default: 0.8)
- angular_attractors_strength [float] (default: 1.0)
- linear_velocity_decay [float] (default: 1.0)
- target_duration [float] (default: 5.0 [s])

#### Services:
- ~/navigation_enable (it enables the shared control navigation)
- ~/navigation_disable (it disables the shared control navigation)
- ~/navigation_start (it starts the navigation)
- ~/navigation_stop (it stops the navigation)
