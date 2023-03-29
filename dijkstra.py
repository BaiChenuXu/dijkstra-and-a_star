import numpy as np

# the main function of dijkstra
def dijkstra(graph, obtain, candi, start1, end1):
    key_array = []
    raj_final = []
    value_array = []
    while candi:
        # get close set end（current） node dictionary-key information
        key_m = list(obtain.keys())[-1]
        # get the information of which nodes can reach from current node and the corresponding distance
        dict_m = graph[key_m]
        index_1 = np.array(np.nonzero(list(dict_m.values())))
        k = 0
        for i in index_1[0]:
            # put accessible nodes into the key_array，if the accessible node value (trajectory length) less than
            # the value of child node value of current node, update the child node value
            key_array.append(list(dict_m.keys())[i])
            candi[key_array[k]] = min(candi[key_array[k]], obtain[key_m] + graph[key_m][key_array[k]])
            k = k + 1
        candi_list_m = list(candi.values())
        # select the index of the minimum value in the open set
        idx_m = candi_list_m.index(min(candi_list_m))
        key_in = list(candi.keys())[idx_m]
        # push the index and the value into the close set
        obtain[key_in] = candi.pop(key_in)
        key_array = []
    # let the end node as the begin node，find minimum value of accessible nodes in the close set to connect the
    # trajectory in reverse direction，that is similar to the Dynamic Programming
    raj_final.insert(0, end1)
    while raj_final[0] != start1:
        k = 0
        for i in graph:
            if graph[i][raj_final[0]] != 0:
                value_array.append(obtain[i])
                key_array.append(list(obtain.keys())[k])
            k = k + 1
        raj_final.insert(0, key_array[value_array.index(min(value_array))])
        key_array = []
        value_array = []
    print('the shortest trajectory value from start node to every node = ', obtain)
    print('reach the end node，the shortest trajectory node array  = ', raj_final)


# use dictionary to establish directed graph
graph_dict = {'A': {'A': 0, 'B': 2, 'C': 5, 'D': 0, 'E': 0, 'F': 0, 'G': 0},
              'B': {'A': 0, 'B': 0, 'C': 6, 'D': 1, 'E': 3, 'F': 0, 'G': 0},
              'C': {'A': 0, 'B': 0, 'C': 0, 'D': 0, 'E': 0, 'F': 8, 'G': 0},
              'D': {'A': 0, 'B': 0, 'C': 0, 'D': 0, 'E': 4, 'F': 0, 'G': 0},
              'E': {'A': 0, 'B': 0, 'C': 0, 'D': 0, 'E': 0, 'F': 0, 'G': 9},
              'F': {'A': 0, 'B': 0, 'C': 0, 'D': 0, 'E': 0, 'F': 0, 'G': 7},
              'G': {'A': 0, 'B': 0, 'C': 0, 'D': 0, 'E': 0, 'F': 0, 'G': 0}}
# start node
start = 'A'
# end node
end = 'G'
# initialize the close set
obtain_dict = {'A': 0}
# initialize the open set
candi_dict = {'B': 999, 'C': 999, 'D': 999, 'E': 999, 'F': 999, 'G': 999}

# call the dijkstra to get the shortest trajectory of directed graph
dijkstra(graph_dict, obtain_dict, candi_dict, start, end)
