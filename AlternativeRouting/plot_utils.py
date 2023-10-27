import sumolib
import geopandas as gpd
import numpy as np
import itertools
from shapely.geometry import LineString, Point



def gdf_from_sumo_network(sumo_road_network, crs="EPSG:4326", edge_list=None):
    
    if edge_list is None:
        edges_to_plot = sumo_road_network.getEdges()
    else:
        #edges_to_plot = [e for e in sumo_road_network.getEdges() if e.getID() in edge_list]
        edges_to_plot = [sumo_road_network.getEdge(eid) for eid in edge_list]
    
    list_line_strings = []
    
    for edge in edges_to_plot:       
        edge_gps_list = [sumo_road_network.convertXY2LonLat(x[0], x[1]) for x in edge.getShape()]
        list_line_strings.append(LineString(edge_gps_list))
        
    df_net = gpd.GeoDataFrame(geometry=list_line_strings, crs=crs)  
    
    return df_net



def edges_to_gps_list(road_network, edge_list):
    
    list_gps = []

    for e in edge_list:

        shape_lon_lat = [road_network.convertXY2LonLat(x[0], x[1]) for x in road_network.getEdge(e).getShape()]
        list_gps+=shape_lon_lat
    
    return list_gps


def plot_paths(sumo_road_network, gdf_net, paths_2_plot, figsize=(6, 6), plot_network=True, bbox="auto", ax=None, hide_axis=True):
    
    if ax is None:
        fig, ax = plt.subplots(nrows=1, ncols=1, figsize=figsize);
    
    min_x, min_y = np.inf, np.inf
    max_x, max_y = -np.inf, -np.inf
    
    if plot_network:
        gdf_net.plot(ax=ax, linewidth=1, color="black", alpha=.07);
    
    for path in paths_2_plot:

        gdf_edge = gpd.GeoDataFrame(geometry=[LineString(edges_to_gps_list(sumo_road_network, path["edges"]))])
        gdf_edge.crs = "EPSG:4326"
        gdf_edge.plot(ax=ax, linewidth=1, color="blue", alpha=0.5);
    
    
        if bbox=="auto":
            min_x = min(min_x, gdf_edge.bounds["minx"].values[0])
            max_x = max(max_x, gdf_edge.bounds["maxx"].values[0])
            min_y = min(min_y, gdf_edge.bounds["miny"].values[0])
            max_y = max(max_y, gdf_edge.bounds["maxy"].values[0])

    if bbox=="auto":
        ax.set_xlim(min_x-5e-3, max_x+5e-3)
        ax.set_ylim(min_y-5e-3, max_y+5e-3)
    else:
        ax.set_xlim(*bbox[0])
        ax.set_ylim(*bbox[1])
        
    if hide_axis:
        ax.axis("off")
        
        
def plot_bar_paths_cost(paths, y_label="", ax=None, figsize=(6,6)):
    
    if ax is None:
        fig, ax = plt.subplots(nrows=1, ncols=1, figsize=figsize);
    
    ax.bar([f"path {i}" for i in range(1, len(paths)+1)], [path["original_cost"] for path in paths])
    ax.set_ylabel(y_label)