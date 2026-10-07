# Every contact point reported for one simulation step.
#
# A whole-step batch rather than one message per point: the engine produces the
# full manifold at once, the count varies step to step, and a grasp can produce
# hundreds. One message per step keeps them atomically consistent -- a consumer
# never sees half of one step's contacts and half of the next's.

# stamp is simulated time, so use_sim_time:=true is required downstream, as it
# is for every topic this interface publishes. frame_id is the frame the
# positions, normals and forces are expressed in: the world frame.
std_msgs/Header header

ContactPoint[] points
