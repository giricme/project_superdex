# One contact point between two actors, as the SuperDex engine reports it.
#
# This is the one piece of SuperDex's state with no standard ROS equivalent.
# sensor_msgs has no dense-contact or tactile message, and the nearest
# candidates all lose something: WrenchStamped collapses the manifold to a
# single resultant, PointCloud2 keeps the geometry but drops the force, and
# visualization_msgs/Marker is a drawing, not a measurement. So we define it.
#
# Every quantity is in the world frame. SuperDex declares x-forward, y-left,
# z-up, which is REP-103, so nothing on this path converts anything.
#
# Fields map one-to-one onto superdex.physics.ContactPoint, renamed to the ROS
# spelling (pos_a -> position_a). The engine's actor handles are resolved to
# link names here, because a handle is meaningless outside the process that
# issued it and a frame name is what a ROS consumer can actually use.

# Bare link name, matching the URDF and the /tf frame ids (without tf_prefix).
# Non-bot actors keep their own name -- "ground" for the floor plane.
#
# The pair is symmetric: the engine does not designate one body as canonical,
# and which one lands in link_a depends on the direction of the contact test.
# A contact between two bodies the publisher queries separately -- two fingers
# touching each other, say -- therefore appears twice in one message, once from
# each side, with a and b swapped. If you want each pair once, filter on
# link_a < link_b.
string link_a
string link_b

# Contact position on link_a, and the closest point on link_b's surface.
# position_b is position_a - normal * distance.
geometry_msgs/Point position_a
geometry_msgs/Point position_b

# Unit contact normal, pointing away from link_b.
geometry_msgs/Vector3 normal

# Contact force [N] applied to link_a by link_b. Already weighted by
# patch_area, so summing force over a link gives that link's total contact
# force -- it is not a pressure.
geometry_msgs/Vector3 force

# Signed separation [m] between position_a and link_b's surface. Negative means
# the two surfaces overlap. The engine reports near-contacts too, so a point
# with distance > 0 and zero force is normal and expected.
float64 distance

# Quadrature weight of the sample point [m^2 for surface contact]: the surface
# area this point stands for. This is the field that makes the message worth
# defining -- it is what distinguishes a contact-first engine's output from a
# point-contact engine's, and it is what you integrate over to get pressure or
# contact area.
float64 patch_area
