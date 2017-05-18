# an occupancy grid which captures perspective

Header header

# The map resolution [m/cell]
float64 resolution
# Map width [cells]
uint32 width
# Map height [cells]
uint32 height
# Map layers [# of perspective bins]
uint32 layers
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
geometry_msgs/Pose2D origin

# The boundaries of each perspective. An angle falls into bin i if it satisfies:
# angle > bins[i] && angle < bins[i+1]. Bin boundaries begin at -pi and span
# [-pi, pi). If a grid has n bins, an angle that satisfies: angle > bins[n-1]
# belongs to the last bin
float64[] bins

# The map data, in row-major order, starting with (0,0).  Occupancy
# probabilities are stored in log odds format: [-inf, +inf]. Unknown = 0.0
float64[] data
