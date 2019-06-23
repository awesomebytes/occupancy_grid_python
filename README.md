# occupancy_grid_python

**occupancy_grid_python** offers a Python interface to manage OccupancyGrid messages. It supports topics representing a map or a costmap as usually seen in the navigation stack. Including costmaps with the costmap_updates subtopic. It allows to transform from world coordinates to map coordinates and to retrieve the costs from the costmap in any of these coordinates.

Note that to keep the class lightweight no transformations are offered in between frames.

Usage:

```python

from occupancy_grid_python import OccupancyGridManager

# Subscribe to the nav_msgs/OccupancyGrid topic
ogm = OccupancyGridManager('/move_base/global_costmap/costmap',
                            subscribe_to_updates=True)  # default False

# Now you can do basic operations
print(ogm.resolution)
# Note that OccupancyGrid data starts on lower left corner (if seen as an image)
# width / X is from bottom to top
# height / Y is from left to right
print(ogm.width)
print(ogm.height)
print(ogm.origin)  # geometry_msgs/Pose
print(ogm.reference_frame)  # frame_id of this OccupancyGrid

# You can check the costmap coordinates of world coordinates (in the frame of the OccupancyGrid)
print(ogm.get_costmap_x_y(0.0, 0.0))
# You can check the world coordinates of costmap coordinates (in the frame of the OccupancyGrid)
print(ogm.get_world_x_y(0, 0))

# You can get the cost of a costmap cell
print(ogm.get_cost_from_costmap_x_y(0, 0))
# You can get the cost from world coordinates (in the frame of the OccupancyGrid)
print(ogm.get_cost_from_world_x_y(0.0, 0.0))

# You can check if some coordinates are inside of the grid map
print(ogm.is_in_gridmap(9999, 9999))

# You can find the closest cell with a cost under a value (to find a free cell for example)
# radius is in costmap pixels
print(ogm.get_closest_cell_under_cost(x=100, y=100, cost_threshold=100, max_radius=4))

# You can find the closest cell with a cost over a value (to find an occupied cell for example)
print(ogm.get_closest_cell_over_cost(x=10, y=200, cost_threshold=254, max_radius=8))

```

## Example to transform a coordinate into another frame

If for example we have a `geometry_msgs/Point` in the frame `base_footprint` and we want to check if there is something considered as an obstacle in our `global_costmap`, as our `global_costmap` operates in `map` frame we will need to transform it.
This can be done in Python like:

```python
import rospy
from tf import TransformListener

# Initialize the listener (needs some time to subscribe internally to TF and fill its buffer)
tl = TransformListener()

# Our point would look like this
from geometry_msgs.msg import PointStamped
p = PointStamped()
p.header.stamp = rospy.Time.now()
p.header.frame_id = 'base_footprint'
p.point.x = 1.0
p.point.y = 0.5
p.point.z = 0.0

# Transform the point from base_footprint to map
map_p = tl.transformPoint('map', p)

# Note that transformXXXX can raise exceptions of types:
# ConnectivityException (Raised when the TF tree is not connected between the frames requested.)
# ExtrapolationException (Raised when a tf method has attempted to access a frame, but the frame is not in the graph. The most common reason for this is that the frame is not being published, or a parent frame was not set correctly causing the tree to be broken.)
# LookupException (couldn't find the frame in the buffer at all)
# So you should surround it with a try/except block

```

Once you have the correct coordinates, you could check the costmap value with:

```python

from occupancy_grid_python import OccupancyGridManager

# Subscribe to the nav_msgs/OccupancyGrid topic
ogm = OccupancyGridManager('/move_base/global_costmap/costmap',
                            subscribe_to_updates=True)  # default False

# from the previous piece of code we have map_p
cost = ogm.get_cost_from_world_x_y(map_p.point.x, map_p.point.y)

# Oh, the cost implies there is an obstacle... we can search for the closest point that's free
costmap_x, costmap_y = ogm.get_costmap_x_y(map_p.point.x, map_p.point.y)

safe_x, safe_y, cost_safe = ogm.get_closest_cell_under_cost(costmap_x, costmap_y,
    cost_threshold=50, max_radius=5)

# Maybe we should send a goal to (safe_x, safe_y instead)

```