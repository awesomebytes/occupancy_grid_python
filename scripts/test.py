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
