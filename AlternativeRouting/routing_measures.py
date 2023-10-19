import igraph


def distinct_edges_traveled(path_list):
    
    return set(edge for path in path_list for edge in path)


def redundancy(path_list):
    
    distinct_edges = distinct_edges_traveled(path_list)
    
    total_number_of_edges = sum([len(p) for p in path_list])
    
    redundancy_score = (total_number_of_edges/len(distinct_edges))
    
    #if normalize:
        #redundancy_score = (redundancy_score-1)/(len(path_list)-1)
    
    return redundancy_score


def normalized_jaccard_coefficient(list1, list2):
    
    # Find the intersection of the two lists
    intersection = set(list1).intersection(list2)
    
    # Find the union of the two lists
    union = set(list1).union(list2)
    
    # Calculate and return the Jaccard coefficient
    return len(intersection) / (len(list1)+ len(list2) - len(intersection))



def compute_edge_load(path_list, edge_list):
    
    edge_load = {e:0 for e in edge_list}
    
    for path in path_list:
        for edge in path:
            edge_load[edge]+=1
    
    return edge_load



def compute_edge_capacity(sumo_edges):

    conversion_factor = 2.2369362912
    q = 0.5

    edge_capacity = {}

    for edge in sumo_edges:

        speed_m_s = edge.getSpeed()
        sl = speed_m_s*conversion_factor

        num_lanes = edge.getLaneNumber()

        # when the speed limit of a road segment sl≤45, it is defined as an arterial road
        if sl <= 45:
            capacity = 1900*num_lanes*q
        # when the speed limit of a road segment 45<sl<60, it is defined as a highway
        elif 45<sl<60:
            capacity = (1000+20*sl)*num_lanes
        # when the speed limit of a road segment sl ≥60, it is defined as a freeway
        elif sl>=60:
            capacity = (1700+10*sl)*num_lanes

        edge_capacity[edge.getID()] = capacity
        
    return edge_capacity



def compute_voc(edge_load, edge_capacity):

    edge_voc = {}

    for edge in edge_load:

        volume = edge_load[edge]
        capacity = edge_capacity[edge]

        voc = volume/capacity

        edge_voc[edge] = voc

    return edge_voc



def dis(G, path0, path1, attribute):

    intersection = set(path0).intersection(path1)
    sum_intersection = sum(G.es[intersection][attribute])
    
    union = set(path0).union(path1)
    sum_union = sum(G.es[union][attribute])

    return 1 - (sum_intersection/sum_union)



def div(G, path_list, attribute):
    
    dis_list = []
    
    if len(path_list) == 0:
        return 0
    
    for i, pi in enumerate(path_list):
        for j, pj in enumerate(path_list):
            if i<j:
                dis_list.append(dis(G, pi, pj, attribute)) 
                
    return min(dis_list)


def compute_driver_sources(list_routes, edge_tile_dict, origin=True):
        
    ds_dict = {}
    
    if origin:
        s = 0
    else:
        s = -1
    
    for route in list_routes:
        
        edges = route
        
        origin = edges[s]
        tile = edge_tile_dict[origin]
        
        for edge in edges:
            if edge in ds_dict:
                if tile in ds_dict[edge]:
                    ds_dict[edge][tile] += 1
                else:
                    ds_dict[edge][tile] = 1
                
            else:
                ds_dict[edge] = {}
                ds_dict[edge][tile] = 1
                
    return ds_dict


def compute_MDS(ds_dict, traffic_threshold):
    mds_dict = {}
    
    for edge, ds in ds_dict.items():
        # driver sources sorted by flow
        ds_list = sorted(ds.items(), key=lambda x: x[1], reverse=True)
        ds_flow = sum(x[1] for x in ds_list)
        tmp_sum = 0
        i = 0
        mds_edge_dict = {}
        while tmp_sum <= ds_flow*traffic_threshold:
            mds_edge_dict[ds_list[i][0]] = ds_list[i][1]
            tmp_sum += ds_list[i][1]
            i += 1

        mds_dict[edge] = mds_edge_dict
        
    return mds_dict


def compute_k_road(mds):
    k_road = {}
    for edge, mds_dict in mds.items():
        k_road[edge] = len(mds_dict)
        
    return k_road



def split_interval_sliding(a, b, s, slide):
    intervals = []
    current = a

    while current + s <= b:
        intervals.append([current, current + s])
        current += slide

    intervals[-1][1] += 1

    return intervals


def compute_temp_redundancy_sliding(dict_md, dict_path_tmp_red, window_s, slide):

    # compute high and low
    tt_list = [dict_md[k]["time"] for k in dict_md]
    
    low = min(tt_list)
    high = max(tt_list)
      
    intervals = split_interval_sliding(low, high, window_s, slide)
        
    red_list = []
    
    for (i, j) in intervals:
        
        vehicles_id_split = [int(k.split("_")[1]) for k in dict_md if i<=dict_md[k]["time"]<j]
        paths_split = [dict_path_tmp_red[k] for k in vehicles_id_split]
        
        red_list.append(redundancy(paths_split))
                
    return red_list
