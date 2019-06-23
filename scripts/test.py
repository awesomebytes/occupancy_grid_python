#!/usr/bin/env python

from occupancy_grid_python import OccupancyGridManager
from nav_msgs.msg import OccupancyGrid
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
    # Give some time to initialize
    rospy.sleep(2.0)
    rospy.loginfo("Done")
    rp = RosPack()
    pkg_path = rp.get_path('occupancy_grid_python')
    global_costmap_msg = cPickle.load(
        open(pkg_path + '/data/global_costmap.pickle'))
    local_costmap_msg = cPickle.load(
        open(pkg_path + '/data/local_costmap.pickle'))
    map_msg = cPickle.load(
        open(pkg_path + '/data/map.pickle'))
    global_costmap_pub.publish(global_costmap_msg)
    local_costmap_pub.publish(local_costmap_msg)
    map_pub.publish(map_msg)

    rospy.loginfo("Done publishing example data")

    ogm = OccupancyGridManager('/move_base/global_costmap/costmap',
                               subscribe_to_updates=False)

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

    # Print all the Grid
    # il = range(0, ogm.height)
    # # reverse the list as the origin coordinate is bottom left
    # il.reverse()
    # for i in il:
    #     accum = ''
    #     l = range(0, ogm.width)
    #     # l.reverse()
    #     for j in l:
    #         accum += str(ogm.get_cost_from_costmap_x_y(i, j)) + ' '
    #         # print(ogm.get_cost_from_costmap_x_y(i, 270))
    #     print accum

    rospy.loginfo("Getting closest x y")
    closest_x, closest_y, cost = ogm.get_closest_cell_under_cost(
        ogm.height / 2, ogm.width / 2, 2, 5)
    rospy.loginfo("closest x y cost: " + str((closest_x, closest_y, cost)))

    ogm = OccupancyGridManager('/move_base/local_costmap/costmap',
                               subscribe_to_updates=False)

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

    # il = range(0, ogm.height)
    # # reverse the list as the origin coordinate is bottom left
    # il.reverse()
    # for i in il:
    #     accum = ''
    #     l = range(0, ogm.width)
    #     # l.reverse()
    #     for j in l:
    #         accum += str(ogm.get_cost_from_costmap_x_y(i, j)) + ' '
    #         # print(ogm.get_cost_from_costmap_x_y(i, 270))
    #     print accum

    rospy.loginfo("Getting closest x y")
    closest_x, closest_y, cost = ogm.get_closest_cell_under_cost(
        ogm.height / 2 + 20, ogm.width / 2 + 20, 255, 5)
    rospy.loginfo("closest x y cost: " + str((closest_x, closest_y, cost)))

    ogm = OccupancyGridManager('/map',
                               subscribe_to_updates=False)

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

    # il = range(0, ogm.height)
    # # reverse the list as the origin coordinate is bottom left
    # il.reverse()
    # for i in il:
    #     accum = ''
    #     l = range(0, ogm.width)
    #     # l.reverse()
    #     for j in l:
    #         accum += str(ogm.get_cost_from_costmap_x_y(i, j)) + ' '
    #         # print(ogm.get_cost_from_costmap_x_y(i, 270))
    #     print accum

    rospy.loginfo("Getting closest x y")
    closest_x, closest_y, cost = ogm.get_closest_cell_under_cost(
        ogm.height / 2 + 20, ogm.width / 2 + 20, 100, 5)
    rospy.loginfo("closest x y cost: " + str((closest_x, closest_y, cost)))

    rospy.loginfo("radius 1")
    rospy.loginfo("Getting closest x y over (look point: " + str((0, 0)))
    closest_x, closest_y, cost = ogm.get_closest_cell_over_cost(
        0, 0, 253, 1)
    rospy.loginfo("closest x y cost over: " +
                  str((closest_x, closest_y, cost)))

    rospy.loginfo("radius 2")
    rospy.loginfo("Getting closest x y over (look point: " + str((0, 0)))
    closest_x, closest_y, cost = ogm.get_closest_cell_over_cost(
        0, 0, 253, 2)
    rospy.loginfo("closest x y cost over: " +
                  str((closest_x, closest_y, cost)))

    rospy.loginfo("radius 3")
    rospy.loginfo("Getting closest x y over (look point: " + str((0, 0)))
    closest_x, closest_y, cost = ogm.get_closest_cell_over_cost(
        0, 0, 253, 3)
    rospy.loginfo("closest x y cost over: " +
                  str((closest_x, closest_y, cost)))

    rospy.loginfo("radius 4")
    rospy.loginfo("Getting closest x y over (look point: " + str((0, 0)))
    closest_x, closest_y, cost = ogm.get_closest_cell_over_cost(
        0, 0, 253, 4)
    rospy.loginfo("closest x y cost over: " +
                  str((closest_x, closest_y, cost)))

    rospy.loginfo("radius 5")
    rospy.loginfo("Getting closest x y over (look point: " + str((0, 0)))
    closest_x, closest_y, cost = ogm.get_closest_cell_over_cost(
        0, 0, 253, 5)
    rospy.loginfo("closest x y cost over: " +
                  str((closest_x, closest_y, cost)))
