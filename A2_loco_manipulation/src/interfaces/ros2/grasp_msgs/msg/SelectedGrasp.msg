# Acquisition header in planning base frame; never inference completion time.
std_msgs/Header header
string session_id
int32 candidate_index
grasp_msgs/GraspCandidate candidate
geometry_msgs/Pose pregrasp_pose
geometry_msgs/Pose lift_pose
trajectory_msgs/JointTrajectory pregrasp_trajectory
trajectory_msgs/JointTrajectory approach_trajectory
trajectory_msgs/JointTrajectory lift_trajectory
# Object geometry used for planning and fresh-observation revalidation.
geometry_msgs/Point observed_object_center
float32 observed_object_radius_m
float32 observed_object_height_m
