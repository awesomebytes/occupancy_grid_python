#!/usr/bin/env python

import sys
import cPickle
import rospy
from occupancy_grid_python import OccupancyGridManager
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from rospkg import RosPack

# Thanks to @MoscowskyAnton at Github for the inspiration


class OGMTesterCellCost(object):
    def __init__(self):
        self.ogm = OccupancyGridManager('/map', subscribe_to_updates=True)
        self.pub = rospy.Publisher('/closest_cell_cost', PointStamped, queue_size=1)
        # Can use 'Publish Point' in Rviz
        rospy.Subscriber('/clicked_point', PointStamped, self.point_cb)

    def point_cb(self, msg):
        cost = self.ogm.get_cost_from_world_x_y(msg.point.x, msg.point.y)
        rospy.loginfo("Cost at (x, y): {}, {} is {}".format(msg.point.x, msg.point.y, cost))
        costmap_x, costmap_y = self.ogm.get_costmap_x_y(msg.point.x, msg.point.y)
        rospy.loginfo("Which is in costmap coords (x, y): {}, {}".format(costmap_x, costmap_y))

        # Publish a PointStamped of the closest cell over 99 cost (black in map)
        closest_cell_x, closest_cell_y, cost = self.ogm.get_closest_cell_over_cost(costmap_x, costmap_y, 99, 9)
        if closest_cell_x != -1:
            p = PointStamped()
            p.header.frame_id = self.ogm.reference_frame
            p.point.x, p.point.y = self.ogm.get_world_x_y(closest_cell_x, closest_cell_y)
            self.pub.publish(p)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    argv = rospy.myargv(sys.argv)
    use_test_data = True
    # Allow to not use test data
    if len(argv) > 1:
        if argv[1] == '--no-test-data':
            use_test_data = False
    rospy.init_node('test_clicking')

    if use_test_data:
        map_pub = rospy.Publisher('/map',
                                  OccupancyGrid, queue_size=1,
                                  latch=True)
        rp = RosPack()
        pkg_path = rp.get_path('occupancy_grid_python')
        map_msg = cPickle.load(
            open(pkg_path + '/data/map.pickle'))

    # Let publisher initialize
    rospy.sleep(1.0)
    map_pub.publish(map_msg)

    ogmt = OGMTesterCellCost()
    ogmt.run()
