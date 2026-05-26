import json
from collections import defaultdict
import math
import heapq
import random
import subprocess
import sys
import webbrowser
import os

from AstarAlgorithms import a_star_shortest_search, a_star_fastest_search, a_star_eco_search
from NodeGraphing import visualize_all_routes, visualize_route
from AstarGraphUtils import haversine, find_nodes_by_road_name, get_largest_component

# Load the raw Overpass JSON
map_file_path = os.path.join("data", "ManMap.json")
with open(map_file_path, 'r', encoding='utf-8') as f:
    data = json.load(f)




DEFAULT_SPEEDS = {
    'motorway': 70,
    'trunk': 60,
    'primary': 30,
    'secondary': 30,
    'tertiary': 30,
    'unclassified': 30,
    'residential': 20, 
    'service': 10
}


# Build map of node coordinates and your graph network
node_coords = {}
graph = {}

def setup_graph_from_osm_data(data):
    # First pass: Extract all node locations
    for element in data['elements']:
        if element['type'] == 'node':
            node_coords[element['id']] = (element['lat'], element['lon'])

    # Second pass: Connect nodes using ways
    for element in data['elements']:
        if element['type'] == 'way':
            way_nodes = element.get('nodes', [])
            tags = element.get('tags', {})
            
            # Determine speed limit
            highway_type = tags.get('highway', 'residential')
            speed_mph = DEFAULT_SPEEDS.get(highway_type, 30) 
            if 'maxspeed' in tags:
                try:
                    speed_mph = int(''.join(filter(str.isdigit, tags['maxspeed'])))
                except ValueError:
                    pass
            
            speed_mps = speed_mph * 0.44704  # Speed in meters per second
            
            # Connect sequential nodes
            for i in range(len(way_nodes) - 1):
                node_a = way_nodes[i]
                node_b = way_nodes[i+1]
                
                if node_a in node_coords and node_b in node_coords:
                    dist_meters = haversine(node_coords[node_a], node_coords[node_b])
                    travel_time_seconds = dist_meters / speed_mps
                    
                    # CALCULATE VEHICLE ENERGY (WATT-HOURS) ---
                    # Physics parameters for a standard mid-sized car:
                    mass_kg = 1500
                    g = 9.81
                    c_r = 0.015       # Rolling resistance coefficient
                    c_d = 0.3         # Aerodynamic drag coefficient
                    rho = 1.2         # Air density (kg/m^3)
                    a_front = 2.2     # Frontal surface area of car (m^2)
                    
                    # Power required to maintain speed (Watts = Joules/sec)
                    power_rolling = c_r * mass_kg * g * speed_mps
                    power_aerodynamic = 0.5 * rho * c_d * a_front * (speed_mps ** 3)
                    total_power_watts = power_rolling + power_aerodynamic
                    
                    # Energy = Power * Time (Convert Joules to Watt-hours by dividing by 3600)
                    energy_wh = (total_power_watts * travel_time_seconds) / 3600.0
                    # --------------------------------------------------
                    
                    if node_a not in graph: graph[node_a] = []
                    if node_b not in graph: graph[node_b] = []
                    
                    # Store all three properties on the edge dictionary
                    edge_data = {
                        'to': node_b,
                        'distance': dist_meters,
                        'time': travel_time_seconds,
                        'energy': energy_wh       
                    }
                    reverse_edge_data = {
                        'to': node_a,
                        'distance': dist_meters,
                        'time': travel_time_seconds,
                        'energy': energy_wh
                    }
                    
                    graph[node_a].append(edge_data)
                    graph[node_b].append(reverse_edge_data)



def get_random_node_from_main_network(main_network):
    if len(main_network) > 1:
        # Pick two random nodes STRICTLY from the main network
        sample_start, sample_destination = random.sample(main_network, 2)
        
        print("========================================")
        print(f"Guaranteed A* Route Test:")
        print(f"  Start Node ID: {sample_start}")
        print(f"  End Node ID:   {sample_destination}")
        print("========================================")

        # Run A*
        nodes_1 = a_star_shortest_search(graph, node_coords, sample_start, sample_destination)
        nodes_2 = a_star_fastest_search(graph, node_coords, sample_start, sample_destination)
        nodes_3 = a_star_eco_search(graph, node_coords, sample_start, sample_destination)
        if nodes_1:
            print(f"SUCCESS! Path found consisting of {len(nodes_1)} nodes.")
        else:
            print("This should theoretically never happen now!")

        if nodes_2:
            print(f"SUCCESS! Fastest path found consisting of {len(nodes_2)} nodes.")   
        
        short_dist = sum(e['distance'] for n in nodes_1 for e in graph.get(n, []) if e['to'] in nodes_1) / 2
        short_time = sum(e['time'] for n in nodes_1 for e in graph.get(n, []) if e['to'] in nodes_1) / 2

        # Compute total distance and total time for the FASTEST path
        fast_dist = sum(e['distance'] for n in nodes_2 for e in graph.get(n, []) if e['to'] in nodes_2) / 2
        fast_time = sum(e['time'] for n in nodes_2 for e in graph.get(n, []) if e['to'] in nodes_2) / 2

        # Compute total distance and total time for the ECO path
        eco_dist = sum(e['distance'] for n in nodes_3 for e in graph.get(n, []) if e['to'] in nodes_3) / 2
        eco_time = sum(e['time'] for n in nodes_3 for e in graph.get(n, []) if e['to'] in nodes_3) / 2

        print(f"Shortest Path Results -> Distance: {short_dist:.1f}m | Time: {short_time:.1f}s")
        print(f"Fastest Path Results  -> Distance: {fast_dist:.1f}m | Time: {fast_time:.1f}s")
        print(f"Eco-Friendly Path Results -> Distance: {eco_dist:.1f}m | Time: {eco_time:.1f}s")

        visualize_all_routes(nodes_1, nodes_2, nodes_3, node_coords, "route_comparison.html")
    else:
        print("Error: Could not find a valid connected network.")





def test_specific_road_to_road_route(start_road_query, goal_road_query, main_network):
    # Gather all raw node entries matching the street names
    start_road_nodes = find_nodes_by_road_name(data, start_road_query)
    goal_road_nodes  = find_nodes_by_road_name(data, goal_road_query)

    # Extract types based on how your graph dictionary keys are structured
    # (Inspects if graph uses '12345' strings or 12345 integers natively)
    sample_graph_key = list(graph.keys())[0] if graph else ""
    is_string_schema = isinstance(sample_graph_key, str)

    # Create a strict, correctly-typed network lookup pool
    if is_string_schema:
        clean_network_set = {str(n) for n in main_network}
        valid_starts = [str(n) for n in start_road_nodes if str(n) in clean_network_set]
        valid_goals  = [str(n) for n in goal_road_nodes if str(n) in clean_network_set]
    else:
        clean_network_set = {int(n) for n in main_network}
        valid_starts = [int(n) for n in start_road_nodes if int(n) in clean_network_set]
        valid_goals  = [int(n) for n in goal_road_nodes if int(n) in clean_network_set]

    # Check for validity
    if not valid_starts:
        print(f"Error: Could not find any valid connected graph entries for '{start_road_query}'")
        return
    if not valid_goals:
        print(f"Error: Could not find any valid connected graph entries for '{goal_road_query}'")
        return

    # ROBUST MATCHING LOOP: Try street nodes until a valid routing combo matches
    # This prevents getting stuck on a dead-end boundary node at index [0]
    traf_nodes_1, traf_nodes_2, traf_nodes_3 = None, None, None
    final_start_node, final_goal_node = None, None

    # Limit search optimization scan so it doesn't loop infinitely across huge arrays
    for s_node in valid_starts[:5]: 
        for g_node in valid_goals[:5]:
            if s_node == g_node:
                continue
                
            # Try running the base search engine
            res_1 = a_star_shortest_search(graph, node_coords, s_node, g_node)
            if res_1:
                traf_nodes_1 = res_1
                final_start_node = s_node
                final_goal_node = g_node
                # Calculate remaining profiles using the matched pair
                traf_nodes_2 = a_star_fastest_search(graph, node_coords, s_node, g_node)
                traf_nodes_3 = a_star_eco_search(graph, node_coords, s_node, g_node)
                break
        if traf_nodes_1:
            break

    # Output metrics matching your working random test block template
    if traf_nodes_1 and traf_nodes_2 and traf_nodes_3:
        print("========================================")
        print(f"Executing Road-to-Road Routing [SUCCESS]:")
        print(f"  Origin Street:      {start_road_query} (Node: {final_start_node})")
        print(f"  Destination Street: {goal_road_query} (Node: {final_goal_node})")
        print("========================================")

        short_dist = sum(e['distance'] for n in traf_nodes_1 for e in graph.get(n, []) if e['to'] in traf_nodes_1) / 2
        short_time = sum(e['time'] for n in traf_nodes_1 for e in graph.get(n, []) if e['to'] in traf_nodes_1) / 2

        fast_dist = sum(e['distance'] for n in traf_nodes_2 for e in graph.get(n, []) if e['to'] in traf_nodes_2) / 2
        fast_time = sum(e['time'] for n in traf_nodes_2 for e in graph.get(n, []) if e['to'] in traf_nodes_2) / 2

        eco_dist = sum(e['distance'] for n in traf_nodes_3 for e in graph.get(n, []) if e['to'] in traf_nodes_3) / 2
        eco_time = sum(e['time'] for n in traf_nodes_3 for e in graph.get(n, []) if e['to'] in traf_nodes_3) / 2

        print(f"Shortest Path Results -> Distance: {short_dist:.1f}m | Time: {short_time:.1f}s")
        print(f"Fastest Path Results  -> Distance: {fast_dist:.1f}m | Time: {fast_time:.1f}s")
        print(f"Eco-Friendly Path Results -> Distance: {eco_dist:.1f}m | Time: {eco_time:.1f}s")

        visualize_all_routes(traf_nodes_1, traf_nodes_2, traf_nodes_3, node_coords, "trafford_route_comparison.html")
    else:
        print(f"Routing failed: No valid connected paths could link the segments between '{start_road_query}' and '{goal_road_query}'.")



setup_graph_from_osm_data(data)

main_network = get_largest_component(graph)
print(f"Main connected network size: {len(main_network)} nodes")



get_random_node_from_main_network(main_network)
#get_random_node_from_main_network(main_network)

test_specific_road_to_road_route("Chester Street", "Barton Dock Road", main_network)
