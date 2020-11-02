#!/usr/bin/env python

from occupancy_grid_python import OccupancyGridManager
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
import rospy
from rospkg import RosPack

import cPickle

if __name__ == '__main__':
    rospy.init_node('test_occ_grid')
    rospy.loginfo("Initializing")

    global_costmap_pub = rospy.Publisher('/move_base/global_costmap/costmap',
                                         OccupancyGrid, queue_size=1,
                                         latch=True)
    local_costmap_pub = rospy.Publisher('/move_base/local_costmap/costmap',
                                        OccupancyGrid, queue_size=1,
                                        latch=True)
    map_pub = rospy.Publisher('/map',
                              OccupancyGrid, queue_size=1,
                              latch=True)
    update_pub = rospy.Publisher('/map_updates', OccupancyGridUpdate,
                                 queue_size=1)

    occgrid = OccupancyGrid()
    occgrid.header.frame_id = 'map'
    occgrid.info.width = 10  # x
    occgrid.info.height = 2  # y
    occgrid.info.origin.position.x = 0.0
    occgrid.info.origin.position.y = 0.0
    occgrid.info.origin.position.z = 0.0
    occgrid.info.origin.orientation.w = 1.0
    occgrid.info.resolution = 0.0500000007451

    occgrid.data = [i for i in range(0, 20)]
    # [[ 0  1  2  3  4  5  6  7  8  9]
    #  [10 11 12 13 14 15 16 17 18 19]]

    # but interpreted in Rviz costmap as
    # [10 11 12 13 14 15 16 17 18 19]
    # [ 0  1  2  3  4  5  6  7  8  9]

    # Initialize publishers...
    rospy.sleep(3.0)

    map_pub.publish(occgrid)
    rospy.sleep(1.0)

    ogm = OccupancyGridManager('/map',
                               subscribe_to_updates=False)

    for y in range(2):  # row
        for x in range(10):  # column
            if ogm.is_in_gridmap(x, y):
                cost = ogm.get_cost_from_costmap_x_y(x, y)
                print("x: {}, y: {} is in gridmap (cost: {})".format(x, y, cost))
            else:
                print("x: {}, y: {} is NOT in gridmap".format(x, y))

    if ogm.is_in_gridmap(ogm.width - 1, ogm.height - 1):
        print("ogm.width - 1: {}, ogm.height - 1: {} is in gridmap.".format(ogm.width - 1, ogm.height - 1))
    else:
        print("ogm.width - 1: {}, ogm.height - 1: {} is NOT in gridmap.".format(ogm.width - 1, ogm.height - 1))

    # simulate an update with adding all of it again
    ogu = OccupancyGridUpdate()
    ogu.header = occgrid.header
    ogu.header.stamp = rospy.Time.now()
    ogu.x = ogu.y = 0
    ogu.width = occgrid.info.width
    ogu.height = occgrid.info.height
    ogu.data = occgrid.data

    ogm = OccupancyGridManager('/map',
                               subscribe_to_updates=True)
    # Give a moment to the update subscriber to initialize
    rospy.sleep(1.0)

    update_pub.publish(ogu)
    # Wait a moment for the update to arrive
    rospy.sleep(1.0)

    for y in range(2):
        for x in range(10):
            if ogm.is_in_gridmap(x, y):
                cost = ogm.get_cost_from_costmap_x_y(x, y)
                print("x: {}, y: {} is in gridmap (cost: {})".format(x, y, cost))
            else:
                print("x: {}, y: {} is NOT in gridmap".format(x, y))

    if ogm.is_in_gridmap(ogm.width - 1, ogm.height - 1):
        print("ogm.width - 1: {}, ogm.height - 1: {} is in gridmap.".format(ogm.width - 1, ogm.height - 1))
    else:
        print("ogm.width - 1: {}, ogm.height - 1: {} is NOT in gridmap.".format(ogm.width - 1, ogm.height - 1))