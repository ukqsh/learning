#
# Message from CarMaker ROS Node to external ROS2 Node
#

# General
std_msgs/Header  header                                 # General ROS Header (optional)
builtin_interfaces/Time    time                                   # ROS time when message was sent (optional)
int64   cycleno                                # Cycle number since simulation start (optional)
float64 synthdelay                             # Synthetic delay to check synchronization (optional)
float64 velocity
float64 steeringang
float64 gas
float64 brake
float64 steeringangspeed
float64 wheelbase

