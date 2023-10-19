import igraph
from igraph import Graph
import sumolib
import numpy as np
import folium
import warnings
import seaborn as sns
from tqdm.notebook import tqdm

from typing import List

# for the ellipse
import math
from matplotlib.patches import Ellipse


""" utilities for road routing """


def from_sumo_to_igraph_network(road_network):
    
    """
    Converts a SUMO road network to an igraph network.

    Parameters:
    -----------
    road_network : SUMO road network
        A SUMO road network object.

    Returns:
    --------
    G : igraph graph
        An igraph graph representing the road network.
    """
    
    
    nodes_dict = {}
    edges_dict = {}
    node_2_conn_nodes = {}
    connections_list = []
    conn_attr = {"id":[], "length":[], "speed_limit":[], "traveltime":[], "coordinates":[], "center_coord" : []}
    
    for node in road_network.getNodes():
        
        in_edges = [edge for edge in list(node.getIncoming())]
        out_edges = [edge for edge in list(node.getOutgoing())]
        
        # to store the connection nodes
        node_2_conn_nodes[node.getID()] = []
        
        # compute length connection
        unique_connections = set()

        for c in node.getConnections():
            p = (c.getFrom().getID(), c.getTo().getID())
            unique_connections.add(p)
        
        # Fully connected nodes
        if len(in_edges)*len(out_edges) == len(unique_connections):
            
            nodes_dict[node.getID()] = {"in": in_edges, "out": out_edges, "fc": 1}
            
            node_2_conn_nodes[node.getID()]+=[ein.getID()+"_to" for ein in in_edges]
            node_2_conn_nodes[node.getID()]+=[eout.getID()+"_from" for eout in out_edges]
            
            for e in in_edges:
                edge = e.getID()
                if edge in edges_dict:
                    edges_dict[edge]["to"] = node.getID()
                    edges_dict[edge]["coordinates"]['to'] = road_network.convertXY2LonLat(*e.getToNode().getCoord())
                    
                    to_coords = edges_dict[edge]["coordinates"]['to']
                    from_coords = edges_dict[edge]["coordinates"]['from']
                    edges_dict[edge]['center_coord'] = (to_coords[0] + from_coords[0]) / 2, (to_coords[1] + from_coords[1]) / 2
                    
                else:
                    edges_dict[edge] = {"to": node.getID()}
                    edges_dict[edge]["id"] = edge
                    edges_dict[edge]["length"] = e.getLength()
                    edges_dict[edge]["speed_limit"] = e.getSpeed()
                    edges_dict[edge]["traveltime"] = e.getLength()/e.getSpeed()
                    
                    if "coordinates" not in edges_dict[edge].keys():
                        edges_dict[edge]["coordinates"] = {'from' : (0, 0), 
                                                           'to' : road_network.convertXY2LonLat(*e.getToNode().getCoord())}
                    
                    
            for e in out_edges:
                edge = e.getID()
                if edge in edges_dict:
                    edges_dict[edge]["from"] = node.getID()
                    edges_dict[edge]["coordinates"]['from'] = road_network.convertXY2LonLat(*e.getFromNode().getCoord())

                    to_coords = edges_dict[edge]["coordinates"]['to']
                    from_coords = edges_dict[edge]["coordinates"]['from']
                    edges_dict[edge]['center_coord'] = (to_coords[0] + from_coords[0]) / 2, (to_coords[1] + from_coords[1]) / 2

                else:
                    edges_dict[edge] = {"from": node.getID()}
                    edges_dict[edge]["id"] = edge
                    edges_dict[edge]["length"] = e.getLength()
                    edges_dict[edge]["speed_limit"] = e.getSpeed()
                    edges_dict[edge]["traveltime"] = e.getLength()/e.getSpeed()
                    
                    if "coordinates" not in edges_dict[edge].keys():
                        edges_dict[edge]["coordinates"] = {'from' : road_network.convertXY2LonLat(*e.getFromNode().getCoord()), 
                                                           'to' : (0, 0)}
                    
                    
        # Nodes with connections
        else:
            # add new connection nodes
            for e in in_edges:
                edge = e.getID()
                
                node_id = edge+"_to"
                nodes_dict[node_id] = {"in": [edge], "fc": 0}
                node_2_conn_nodes[node.getID()].append(node_id)
                
                
                if edge in edges_dict:
                    edges_dict[edge]["to"] = node_id
                    edges_dict[edge]["coordinates"]['to'] = road_network.convertXY2LonLat(*e.getToNode().getCoord())
                    
                    to_coords = edges_dict[edge]["coordinates"]['to']
                    edges_dict[edge]['center_coord'] = to_coords, to_coords

                else:
                    edges_dict[edge] = {"to": node_id}
                    edges_dict[edge]["id"] = edge
                    edges_dict[edge]["length"] = e.getLength()
                    edges_dict[edge]["speed_limit"] = e.getSpeed()
                    edges_dict[edge]["traveltime"] = e.getLength()/e.getSpeed()
                    
                    if "coordinates" not in edges_dict[edge].keys():
                        edges_dict[edge]["coordinates"] = {'from': (0, 0),
                                                           'to' : road_network.convertXY2LonLat(*e.getToNode().getCoord())}
                                                           
                    

            for e in out_edges:
                
                edge = e.getID()
                node_id = edge+"_from"
                nodes_dict[node_id] = {"out": [edge], "fc": 0}
                node_2_conn_nodes[node.getID()].append(node_id)
                
                if edge in edges_dict:
                    edges_dict[edge]["from"] = node_id
                    edges_dict[edge]["coordinates"]['from'] = road_network.convertXY2LonLat(*e.getFromNode().getCoord())

                    from_coords = edges_dict[edge]["coordinates"]['from']
                    edges_dict[edge]['center_coord'] = from_coords, from_coords
                else:
                    edges_dict[edge] = {"from": node_id}
                    edges_dict[edge]["id"] = edge
                    edges_dict[edge]["length"] = e.getLength()
                    edges_dict[edge]["speed_limit"] = e.getSpeed()
                    edges_dict[edge]["traveltime"] = e.getLength()/e.getSpeed()
                    
                    if "coordinates" not in edges_dict[edge].keys():
                        edges_dict[edge]["coordinates"] = {'from' : road_network.convertXY2LonLat(*e.getFromNode().getCoord()), 
                                                           'to' : (0, 0)}
                    
                    
            for conn in node.getConnections():
                from_edge = conn.getFrom().getID()
                to_edge = conn.getTo().getID()

                connections_list.append([from_edge+"_to", to_edge+"_from"])
                conn_attr["id"].append("connection")
                conn_attr["length"].append(0)
                conn_attr["speed_limit"].append(-1)
                conn_attr["traveltime"].append(0)
                conn_attr["coordinates"].append({'from' : road_network.convertXY2LonLat(*node.getCoord()), 
                                                'to' : road_network.convertXY2LonLat(*node.getCoord())})
                conn_attr["center_coord"].append(road_network.convertXY2LonLat(*node.getCoord()))
                
    edges_list = []
    edges_attr = {"id":[], "length":[], "speed_limit":[], "traveltime":[], "coordinates":[], "center_coord":[]}
    
    for edge in edges_dict.keys():
        edges_list.append((edges_dict[edge]["from"], edges_dict[edge]["to"]))
        edges_attr["id"].append(edge)
        edges_attr["length"].append(edges_dict[edge]["length"])
        edges_attr["speed_limit"].append(edges_dict[edge]["speed_limit"])
        edges_attr["traveltime"].append(edges_dict[edge]["traveltime"])
        edges_attr["coordinates"].append(edges_dict[edge]["coordinates"])
        edges_attr["center_coord"].append(edges_dict[edge]["center_coord"])
        
    G_igraph_new = igraph.Graph(directed=True)
    G_igraph_new.add_vertices(list(nodes_dict.keys()))
    G_igraph_new.add_edges(edges_list, edges_attr)
    G_igraph_new.add_edges(connections_list, conn_attr)
    G_igraph_new.es['original_id'] = range(len(G_igraph_new.es))
    G_igraph_new.vs['original_id'] = range(len(G_igraph_new.vs))
    
    G_igraph_new['edge_sumo_ig'] = dict() # to store the current edge index
    G_igraph_new['edge_vertices'] = dict() # to convert edges to vertices
    G_igraph_new['vertices_edge'] = dict() # to convert vertices to edges
    G_igraph_new['vertices_to_subvertices'] = dict() # to convert vertices and the expanded vertices (for connection)
    G_igraph_new['connection_edges'] = set() # to store the connection edges
    G_igraph_new['vertices_coords'] = set() # to store the location of vertices (for the ellipse)

    for e in G_igraph_new.es:
        if e['id'] != 'connection':
            G_igraph_new['edge_sumo_ig'][e['id']] = e.index
            G_igraph_new['edge_vertices'][e['id']] = {'from' : e.source, 'to' : e.target}
        else:
            G_igraph_new['connection_edges'].add(e.index)
        
        G_igraph_new['vertices_edge'][(e.source, e.target)] = {'length' : e['length'], 'traveltime' : e['traveltime'], 'id' : G_igraph_new.get_eid(e.source, e.target)}

        try:
            G_igraph_new['vertices_edge'][(e.target, e.source)] = {'length' : e['length'], 'traveltime' : e['traveltime'], 'id' : G_igraph_new.get_eid(e.target, e.source)}
        except:
            pass
        
        G_igraph_new['vertices_coords'].add((e.source, e['coordinates']['from']))
        G_igraph_new['vertices_coords'].add((e.target, e['coordinates']['to']))
    
    G_igraph_new['vertices_coords'] = np.array(sorted(G_igraph_new['vertices_coords']), dtype=object)
    
    G_igraph_new['vertices_to_subvertices'] = node_2_conn_nodes
        
    return G_igraph_new


def get_shortest_path(G, from_edge, to_edge, attribute):
    """
    Find the shortest path between two edges in a igraph graph, and translate it to SUMO format.

    Parameters:
        G: The igraph graph
        from_edge: ID of the edge where the path starts
        to_edge: ID of the edge where the path ends
        optimize: The edge attribute to optimize the path on.

    Returns:
        A dictionary with the following keys:
        - 'sumo' (List[str]): The edges of the path in SUMO format
        - 'ig' (List[str]): The igraph edges ID of the path
        - 'cost' (float): The total cost of the path
    """
    
    edge_from = G.es[G["edge_sumo_ig"][from_edge]]
    edge_to = G.es[G["edge_sumo_ig"][to_edge]]
    
    index_from = edge_from.target
    index_to = edge_to.source
        
    id_ig_edge_from = edge_from.index
    id_ig_edge_to = edge_to.index
    
    path = G.get_shortest_paths(index_from, index_to, weights=attribute, output="epath") 
    
    if len(path[0]) == 0:
        return {"sumo": [], "ig": [], "cost": -1}
        
    edges_ig = [id_ig_edge_from]+path[0]+[id_ig_edge_to]

    total_cost = compute_path_cost(G, edges_ig, attribute)

    edges_sumo = [from_edge]+[e for e in G.es[path[0]]["id"] if e != "connection"]+[to_edge]

    return {"sumo": edges_sumo, "ig": edges_ig, "cost": total_cost}




def compute_path_cost(G: igraph.Graph, edge_list: List[str], attribute: str) -> float:     
    """
    This function is used to compute the cost of a path in a graph.

    Parameters:
    G (igraph graph): An igraph graph representing the road network.
    edge_list (list): A list of edges that form the path.
    attribute (str): The key to use to compute the cost, the key must be in the edge_data

    Returns:
    total_cost (float): The total cost of the path
    """

    total_cost = sum(G.es[edge_list][attribute])

    return total_cost




def test_sumo_ig_shortest_paths(road_network_sumo, G, n_tests, attribute, th=1e-4):
    
    
    sumo_edges = road_network_sumo.getEdges()
    
    fastest = True if attribute == "traveltime" else False
    
    for i in tqdm(range(n_tests)):
    
        from_edge = sumo_edges[np.random.randint(0,len(sumo_edges))]
        to_edge = sumo_edges[np.random.randint(0,len(sumo_edges))]
        
        res_sumo = road_network_sumo.getOptimalPath(from_edge, to_edge, fastest=fastest)
        
        if res_sumo[0] is not None:
            edges_sumo = []
            for e in res_sumo[0]:
                edges_sumo.append(e.getID())

            cost_sumo = res_sumo[1]

            res_ig = get_shortest_path(G, from_edge, to_edge, attribute)

            edges_sumo_ig = res_ig["sumo"]
            cost_ig = res_ig["cost"]


            if len(edges_sumo_ig)==0 and len(edges_sumo)==0:
                continue

            assert(abs(cost_ig-cost_sumo)<th)
            
            
            


def visualize_paths(path_list, road_network, colors=None, opacity=1, map_f=None, dotted=False, show_markers=False):
        
    if colors is None:
        color_list = ["blue"]*len(path_list)
    elif isinstance(colors, str) and colors!="rand":
        color_list = [colors]*len(path_list)
    elif colors == "rand":
        color_list = sns.color_palette("colorblind", len(path_list)).as_hex()
        
        
    # transform to GPS points
    paths_gps = [edge_list_to_gps_list(edge_list, road_network) for edge_list in path_list]

    if map_f is None:
        map_f = folium.Map(location=[paths_gps[0][1][1], paths_gps[0][1][0]], tiles='cartodbpositron', zoom_start=13)

    for path, col in zip(paths_gps, color_list): 
          
          folium.PolyLine(locations=[list(reversed(coord)) for coord in path], 
                          weigth=4, color=col, opacity=opacity, 
                          dash_array='10' if dotted else None).add_to(map_f)

    if show_markers:
        lon_from, lat_from = paths_gps[0][0]
        lon_to, lat_to = paths_gps[0][-1]
        folium.Marker(location=[lat_from, lon_from], icon = folium.Icon(color='lightgreen'), popup='ORIGIN').add_to(map_f)
        folium.Marker(location=[lat_to, lon_to], icon = folium.Icon(color='lightred'), popup='TARGET').add_to(map_f)

    return map_f


def edge_list_to_gps_list(edge_list, road_network):
    
    gps_points = []

    for edge_id in edge_list:
        
        sumo_edge = road_network.getEdge(edge_id)

        x, y = sumo_edge.getFromNode().getCoord()
        lon_from, lat_from = road_network.convertXY2LonLat(x, y)

        x, y = sumo_edge.getToNode().getCoord()
        lon_to, lat_to = road_network.convertXY2LonLat(x, y)

        gps_points.append((lon_from, lat_from))
        gps_points.append((lon_to, lat_to))
        
        
    return gps_points

def compute_ellipse(G, from_edge, to_edge, phi = 1.5, eta = 2, min_dist = 0.05):
    
    def get_center(s_lon, s_lat, t_lon, t_lat):
        return (s_lon + t_lon) / 2, (s_lat + t_lat) / 2

    def get_pythagoras(s_lon, s_lat, t_lon, t_lat):
        # Calculate the distance using the Pythagorean theorem
        return math.sqrt((t_lon - s_lon)**2 + (t_lat - s_lat)**2)

    def get_angle(s_lon, s_lat, t_lon, t_lat):
        # Calculate the angle in radians
        angle_rad = math.atan2(t_lat - s_lat, t_lon - s_lon)
        # Convert the angle to degrees
        angle_deg = math.degrees(angle_rad)
        return angle_deg

    if type(from_edge) != str or type(to_edge) != str:
        from_edge, to_edge = from_edge.getID(), to_edge.getID()

    s_lon, s_lat = G.es.find(id = from_edge)['coordinates']['to']
    t_lon, t_lat = G.es.find(id = to_edge)['coordinates']['from']
    
    center_point = get_center(s_lon, s_lat, t_lon, t_lat)
    st_dist = get_pythagoras(s_lon, s_lat, t_lon, t_lat)
    angle = get_angle(s_lon, s_lat, t_lon, t_lat)
    
    if st_dist <= min_dist:
        st_dist = min_dist
        eta = 1

    # draw the ellipse using matplotlib
    ellipse = Ellipse(center_point, st_dist * phi, st_dist * phi / eta, angle)

    return ellipse

def ellipse_subgraph(G, from_edge, to_edge, phi = 1.5, eta = 2, min_dist = 0.05):
    ellipse = compute_ellipse(G, from_edge, to_edge, phi = phi, eta = eta, min_dist = min_dist)
    vertices_coords = G['vertices_coords']
    idx_in_ellipse = ellipse.contains_points(vertices_coords[:,1].tolist())
    ellipsed_vert = vertices_coords[:,0][idx_in_ellipse].tolist()
    
    return ellipse, G.subgraph(ellipsed_vert)
