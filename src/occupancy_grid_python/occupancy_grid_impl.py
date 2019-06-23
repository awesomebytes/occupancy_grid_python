#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
import numpy as np

"""
Class to deal with OccupancyGrid in Python
as in local / global costmaps.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""


class OccupancyGridManager(object):
    def __init__(self, topic, subscribe_to_updates=False):
        # OccupancyGrid starts on lower left corner
        # and width / X is from bottom to top
        # and height / Y is from left to right
        # This makes no sense to me, but it is what it is
        self._grid_data = None
        self._occ_grid_metadata = None
        self._reference_frame = None
        self._sub = rospy.Subscriber(topic, OccupancyGrid,
                                     self._occ_grid_cb,
                                     queue_size=1)
        if subscribe_to_updates:
            self._updates_sub = rospy.Subscriber(topic + '_updates',
                                                 OccupancyGridUpdate,
                                                 self._occ_grid_update_cb,
                                                 queue_size=1)
        rospy.loginfo("Waiting for '" +
                      str(self._sub.resolved_name) + "'...")
        while self._occ_grid_metadata is None and \
                self._grid_data is None:
            rospy.sleep(0.1)
        rospy.loginfo("OccupancyGridManager for '" +
                      str(self._sub.resolved_name) +
                      "'initialized!")
        rospy.loginfo("Height (x): " + str(self.height) +
                      " Width (y): " + str(self.width) +
                      " reference_frame: " + str(self.reference_frame) +
                      " origin: " + str(self.origin))

    @property
    def resolution(self):
        return self._occ_grid_metadata.resolution

    @property
    def width(self):
        return self._occ_grid_metadata.width

    @property
    def height(self):
        return self._occ_grid_metadata.height

    @property
    def origin(self):
        return self._occ_grid_metadata.origin

    @property
    def reference_frame(self):
        return self._reference_frame

    def _occ_grid_cb(self, data):
        self._occ_grid_metadata = data.info
        # Contains resolution, width & height
        # np.set_printoptions(threshold=99999999999, linewidth=200)
        self._grid_data = np.array(data.data,
                                   dtype=np.int8).reshape(data.info.height,
                                                          data.info.width)
        self._reference_frame = data.header.frame_id
        # self._grid_data = np.zeros((data.info.height,
        #                             data.info.width),
        #                            dtype=np.int8)

    def _occ_grid_update_cb(self, data):
        # x, y origin point of the update
        # width and height of the update
        # data, the udpdate
        data_np = np.array(data.data,
                           dtype=np.int8).reshape(data.height, data.width)
        self._grid_data[data.y:data.y +
                        data.height, data.x:data.x + data.width] = data_np
        # print("grid update:")
        # print(self._grid_data)

    def get_world_x_y(self, costmap_x, costmap_y):
        world_x = costmap_x * self.resolution + self.origin.position.x
        world_y = costmap_y * self.resolution + self.origin.position.y
        return world_x, world_y

    def get_costmap_x_y(self, world_x, world_y):
        costmap_x = int((world_x - self.origin.position.x) / self.resolution)
        costmap_y = int((world_y - self.origin.position.y) / self.resolution)
        return costmap_x, costmap_y

    def get_cost_from_world_x_y(self, x, y):
        cx, cy = self.get_costmap_x_y(x, y)
        try:
            return self.get_cost_from_costmap_x_y(cx, cy)
        except IndexError as e:
            raise IndexError("Coordinates out of grid (in frame: {}) x: {}, y: {} must be in between: [{}, {}], [{}, {}]. Internal error: {}".format(
                self.reference_frame, x, y,
                self.origin.position.x,
                self.origin.position.x + self.height * self.resolution,
                self.origin.position.y,
                self.origin.position.y + self.width * self.resolution,
                e))

    def get_cost_from_costmap_x_y(self, x, y):
        if self.is_in_gridmap(x, y):
            return self._grid_data[x][y]
        else:
            raise IndexError(
                "Coordinates out of gridmap, x: {}, y: {} must be in between: [0, {}], [0, {}]".format(
                    x, y, self.height, self.width))

    def is_in_gridmap(self, x, y):
        if -1 < x < self.height and -1 < y < self.width:
            return True
        else:
            return False


if __name__ == '__main__':
    rospy.init_node('test_occ_grid')
    ogm = OccupancyGridManager('/move_base_flex/global_costmap/costmap',
                               subscribe_to_updates=True)
    wx1, wy1 = ogm.get_world_x_y(0, 0)
    print("world from costmap coords  0 0: ")
    print((wx1, wy1))
    cx1, cy1 = ogm.get_costmap_x_y(wx1, wy1)
    print("back to costmap: ")
    print((cx1, cy1))

    cx2, cy2 = ogm.get_costmap_x_y(0.0, 0.0)
    print("costmap from world coords  0 0: ")
    print((cx2, cy2))
    wx2, wy2 = ogm.get_world_x_y(cx2, cy2)
    print("back to world: ")
    print((wx2, wy2))

    cost = ogm.get_cost_from_costmap_x_y(cx1, cy1)
    print("cost cx1, cy1: " + str(cost))
    cost = ogm.get_cost_from_world_x_y(wx1, wy1)
    print("cost wx1, wy1: " + str(cost))
    cost = ogm.get_cost_from_costmap_x_y(cx2, cy2)
    print("cost cx2, cy2: " + str(cost))
    cost = ogm.get_cost_from_world_x_y(wx2, wy2)
    print("cost wx2, wy2: " + str(cost))

    # known place right now
    cost = ogm.get_cost_from_world_x_y(0.307, -0.283)
    print("cost of know nplace is: " + str(cost))

    # cost = ogm.get_cost_from_world_x_y(6.485, -1.462)
    # print("cost of known place is: " + str(cost))
    # cx, cy = ogm.get_costmap_x_y(6.485, -1.462)
    # print("from costmap coords: " + str((cx, cy)))

    print("trying to access out of bounds")
    try:
        cost = ogm.get_cost_from_costmap_x_y(9999, 0)
        print(cost)
    except IndexError as e:
        print("We got, correctly, indexerror: " + str(e))
    try:
        cost = ogm.get_cost_from_costmap_x_y(0, 9999)
        print(cost)
    except IndexError as e:
        print("We got, correctly, indexerror: " + str(e))

    il = range(0, ogm.height)
    # reverse the list as the origin coordinate is bottom left
    il.reverse()
    for i in il:
        accum = ''
        l = range(0, ogm.width)
        # l.reverse()
        for j in l:
            accum += str(ogm.get_cost_from_costmap_x_y(i, j)) + ' '
            # print(ogm.get_cost_from_costmap_x_y(i, 270))
        print accum
    rospy.spin()
