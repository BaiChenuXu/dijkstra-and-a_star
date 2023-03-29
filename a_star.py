import numpy as np
import operator as op
import copy


# the main function of a_star algorithm
def a_star_main(map_input, start1, end1):
    # initialize the close set (obtain) idx and value
    obtain_idx = copy.deepcopy(start1)
    obtain_value = [0]
    # the list store the length from start node to current node
    obtain_value_run = [0]
    # call the heuristic function
    [map_h, candi_idx, candi_value] = heuristic(map_input, start1, end1)
    # use the "map_input" to initialize (deepcopy) the "map_result" which will store the length of shortest trajectory
    # from start node to the corresponding node
    map_result = copy.deepcopy(map_input)
    while obtain_idx[-1] != end1:
        # base on the current node get the accessible search node
        idx_search_1 = neighbor_idx(obtain_idx[-1], len(map_input), len(map_input[1]))
        for k in range(len(idx_search_1)):
            j = idx_search_1[k][0]
            i = idx_search_1[k][1]
            # judge if the node is accessible node and not yet become the element of close set
            temp = (map_input[j][i] == 0 and not exist(obtain_idx, [j, i]))
            if temp:
                # compare current value of current node of open set (candi) and the value from parent node to current
                # node (consist of heuristic value, "obtain_value_run" and "1")
                # use the small one update the open set value
                candi_value[candi_idx.index([j, i])] = \
                    min(candi_value[candi_idx.index([j, i])], obtain_value_run[-1] + map_h[j][i] + 1)
        # find the minimum value in the open set and get corresponding index
        pop_idx = candi_value.index(min(candi_value))
        # delete the value and index from the open set, and add them to the close set
        inter_value = candi_value.pop(pop_idx)
        obtain_value.append(inter_value)
        inter_idx = candi_idx.pop(pop_idx)
        obtain_idx.append(inter_idx)
        # update the "obtain_value_run"
        obtain_value_run.append(obtain_value[-1] - map_h[inter_idx[0]][inter_idx[1]])
        # update the “map_result” which help us to get the shortest trajectory index array
        map_result[obtain_idx[-1][0]][obtain_idx[-1][1]] = obtain_value[-1]
    # when while loop finish，get the length value of shortest trajectory
    tra_long = obtain_value_run[-1]
    # re initialize the start node in the "map_result"
    map_result[start1[0][0]][start1[0][1]] = 0.01
    # let the end node as the begin node，find minimum value of neighbor nodes in the “map_result” to connect the
    # trajectory in reverse direction，that is similar to the Dynamic Programming
    raj_final = [end1]
    while raj_final[0] != start1[0]:
        idx_search_2 = neighbor_idx(raj_final[0], len(map_input), len(map_input[1]))
        value_array = []
        idx_array = []
        for k in range(len(idx_search_2)):
            j = idx_search_2[k][0]
            i = idx_search_2[k][1]
            temp = (map_result[j][i] != 0 and map_result[j][i] != -1)
            if temp:
                value_array.append(map_result[j][i])
                idx_array.append([j, i])
        idx_min = value_array.index(min(value_array))
        idx_add = idx_array.pop(idx_min)
        # from the header，insert the index to the shortest trajectory index array
        raj_final.insert(0, idx_add)
    return tra_long, raj_final

# calculate the heuristic value, and deliver the value to the "map_h"
# get the idx of the open set (candi) and the value(999) of the open set (candi)
def heuristic(map_input, start1, end1):
    map_h = copy.deepcopy(map_input)
    map_input[start1[0][0]][start1[0][1]] = -1
    length_1 = len(map_input)
    length_2 = len(map_input[1])
    candi_idx = []
    candi_value = []
    for j in range(length_1):
        for i in range(length_2):
            if map_input[j][i] != -1:
                idx = [j, i]
                map_h[j][i] = int(dist(idx, end1))
                candi_value.append(999)
                candi_idx.append([j, i])
    map_h[start1[0][0]][start1[0][1]] = 0
    map_input[start1[0][0]][start1[0][1]] = 0
    return map_h, candi_idx, candi_value

# get neighbor nodes that the robot can move to there from current node
def neighbor_idx(idx_now, len1, len2):
    # not consider the limitation of grating map, the accessible node are the "up", "down", "left" and "right" nodes
    idx_search = [[-1 + idx_now[0], idx_now[1]], [1 + idx_now[0], idx_now[1]],
                  [idx_now[0], -1 + idx_now[1]], [idx_now[0], 1 + idx_now[1]]]
    # build a list that store the node of not moving to (the node belong to the "idx_search"), due to
    # the limitation of grating map
    # there are 2 limitations, first is the area that in the map range but the value of map is "-1"
    # second is the map boundary (the node could not exceed the map range)
    idx_del = []
    for i in range(len(idx_search)):
        for j in [0, 1]:
            if idx_search[i][j] == -1:
                idx_del.append(i)
        if idx_search[i][0] == len1:
            idx_del.append(i)
        if idx_search[i][1] == len2:
            idx_del.append(i)
    # delete the node of the "idx_search" list that exist in the "idx_del" at the same time
    # and then get accessible area list "idx_search_f"
    idx_search_f = []
    for i in range(len(idx_search)):
        if not exist(idx_del, i):
            idx_search_f.append(idx_search[i])
    return idx_search_f

# calculate the Euclidean distance from the x to y(end node)
def dist(x, y):
    length_3 = len(x)
    sum_ = 0
    for k in range(length_3):
        sum_ = sum_ + (x[k] - y[k]) ** 2
    dist_ = np.sqrt(sum_)
    return dist_

# check if there is the given element in the list
def exist(obtain_idx, element):
    exi = 0
    for i in obtain_idx:
        if i == element:
            exi = 1
    return exi


# use nested list to establish grating map, "0" is driveable area, otherwise the area is "-1"
map_list = [[0,  0, 0, -1, -1, -1, -1],
            [0, -1, 0, -1, -1, -1, -1],
            [0, -1, 0,  0,  0,  0,  0],
            [0, -1, 0, -1, -1, -1,  0],
            [0, -1, 0, -1,  0,  0,  0],
            [0, -1, 0, -1,  0, -1, -1],
            [0,  0, 0,  0,  0,  0,  0]]
# start node of the trajectory
start = [[2, 2]]
# final node of the trajectory
end = [6, 6]

# call the a_star algorithm search the grating map and get the shortest trajectory
[tra_long1, raj_final1] = a_star_main(map_list, start, end)

print('the distance of the shortest trajectory = ', tra_long1)
print('the idx array of the shortest trajectory = ', raj_final1)
