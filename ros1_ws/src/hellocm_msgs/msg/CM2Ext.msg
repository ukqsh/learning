#
# Message from CarMaker ROS Node to external ROS Node
#

# General
Header  header                                 # General ROS Header (optional)
time    time                                   # ROS time when message was sent (optional)
int64   cycleno                                # Cycle number since simulation start (optional)
float64 synthdelay                             # Synthetic delay to check synchronization (optional)
float64 velocity
float64 sRoad
float64 tRoad
float64 SteeringAng
float64 Gas
float64 Brake
float64 SteeringAngSpeed
float64 WheelBase
float64 DevAngToRoute
float64 DevDistToRoute
float64 PrevViewPointX
float64 PrevViewPointY
float64 PrevViewPointZ
float64 DeltaAng
int64 vz
float64 VehiclePosX
float64 VehiclePosY
float64 VehiclePosZ
float64 PrevViewPointLastX
float64 PrevViewPointLastY
float64 PrevViewPointLastZ
float64 yaw
float64 roll
float64 pitch
