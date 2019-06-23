# occupancy_grid_python

**occupancy_grid_python** offers a Python interface to manage OccupancyGrid messages. It supports topics of type map or type costmap as usually seen in the navigation stack. Including costmaps with the costmap_updates subtopic.

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
print(ogm.get_closest_cell_under_cost(x=100, y=100, cost_threshold=100, max_radius=4))

# You can find the closest cell with a cost over a value (to find an occupied cell for example)
print(ogm.get_closest_cell_over_cost(x=10, y=200, cost_threshold=254, max_radius=8))

```