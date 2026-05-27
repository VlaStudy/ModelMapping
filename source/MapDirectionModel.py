import json
from collections import defaultdict
import math
import heapq
import random
import subprocess
import sys
import webbrowser
import os
import time
import tracemalloc
try:
    from codecarbon import EmissionsTracker
except ImportError:
    print("CodeCarbon not found. Installing it automatically...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "codecarbon"])
    from codecarbon import EmissionsTracker



from AstarAlgorithms import a_star_shortest_search, a_star_fastest_search, a_star_eco_search
from NodeGraphing import visualize_all_routes, visualize_route
from AstarGraphUtils import haversine, find_nodes_by_road_name, get_largest_component, calculate_route_metrics, profile_algorithm_compute, validate_and_extract_nodes

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
        



        tracemalloc.start()
        start = time.time()

        tracker = EmissionsTracker()
        tracker.start()

        nodes_1 = a_star_shortest_search(graph, node_coords, sample_start, sample_destination)
        nodes_2 = a_star_fastest_search(graph, node_coords, sample_start, sample_destination)
        nodes_3 = a_star_eco_search(graph, node_coords, sample_start, sample_destination)
        
        
        emissions = tracker.stop()
        print(f"Estimated CO2 emissions: {emissions} kg")
        end = time.time()
        print(f"Total execution time: {end - start:.2f} seconds")
        current, peak = tracemalloc.get_traced_memory()
        print(f"Current memory usage: {current / 10**6:.2f} MB; Peak: {peak / 10**6:.2f} MB")
        tracemalloc.stop()

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
    valid_starts, valid_goals = validate_and_extract_nodes(graph, data, main_network, start_road_query, goal_road_query)
    if not valid_starts or not valid_goals:
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


def evaluate_and_compare_routes(graph, node_coords, start_node, goal_node, file_name="route_comparison.html"):
    print("\n==================================================")
    print(f"ROUTING METRICS & EMISSIONS TEST")
    print(f"From Node: {start_node} -> To Node: {goal_node}")
    print("==================================================")

    # PROFILE COMPUTE CARBON + RUN ENGINE BACK-TO-BACK
    path_short, comp_short = profile_algorithm_compute(a_star_shortest_search, graph, node_coords, start_node, goal_node, "Shortest")
    path_fast,  comp_fast  = profile_algorithm_compute(a_star_fastest_search,  graph, node_coords, start_node, goal_node, "Fastest")
    path_eco,   comp_eco   = profile_algorithm_compute(a_star_eco_search,      graph, node_coords, start_node, goal_node, "Eco")

    if not (path_short and path_fast and path_eco):
        print("Routing error: One or more engines could not compute a complete path.")
        return

    # CALCULATE PHYSICAL DRIVING IMPACTS
    d_short, t_short, e_short, co2_short = calculate_route_metrics(path_short, graph)
    d_fast,  t_fast,  e_fast,  co2_fast  = calculate_route_metrics(path_fast, graph)
    d_eco,   t_eco,   e_eco,   co2_eco   = calculate_route_metrics(path_eco, graph)

    # PRINT RENDERED COMPARISON MATRIX
    print(f"\n[1] SHORTEST PATH (Distance Optimized):")
    print(f"    • Trip Profile:  {d_short:.1f} meters | {t_short:.1f} seconds")
    print(f"    • Driving CO2:   {co2_short:.4f} kg CO2")
    print(f"    • Code Compute:  {comp_short:.4f} mg CO2")

    print(f"\n[2] FASTEST PATH (Time Optimized):")
    print(f"    • Trip Profile:  {d_fast:.1f} meters | {t_fast:.1f} seconds")
    print(f"    • Driving CO2:   {co2_fast:.4f} kg CO2")
    print(f"    • Code Compute:  {comp_fast:.4f} mg CO2")

    print(f"\n[3] ECO-FRIENDLY PATH (Energy Optimized):")
    print(f"    • Trip Profile:  {d_eco:.1f} meters | {t_eco:.1f} seconds")
    print(f"    • Driving CO2:   {co2_eco:.4f} kg CO2  <-- Savings Target!")
    print(f"    • Code Compute:  {comp_eco:.4f} mg CO2")
    print("==================================================")

    print(f"Eco route reduces driving CO2 by {(co2_short - co2_eco) / co2_short * 100:.1f}% compared to shortest path.")
    print(f"Code compute reduction: {(comp_short - comp_eco) / comp_short * 100:.1f}%")
    print(f"Total CO2 reduction (driving + compute): {(co2_short + comp_short/1e6 - co2_eco - comp_eco/1e6) / (co2_short + comp_short/1e6) * 100:.1f}%")
    print(f"Time increase for eco route: {(t_eco - t_short) / t_short * 100:.1f}%")
    print(f"Distance increase for eco route: {(d_eco - d_short) / d_short * 100:.1f}%")
    print("==================================================")
    print(f"Eco route reduces CO2 by {(co2_fast - co2_eco) / co2_fast * 100:.1f}% compared to fastest path.")
    print(f"Code compute reduction: {(comp_fast - comp_eco) / comp_fast * 100:.1f}%")
    print(f"Total CO2 reduction (driving + compute): {(co2_fast +   comp_fast/1e6 - co2_eco - comp_eco/1e6) / (co2_fast + comp_fast/1e6) * 100:.1f}%")
    print(f"Time increase for eco route vs fastest: {(t_eco - t_fast) / t_fast * 100:.1f}%")
    print(f"Distance increase for eco route vs fastest: {(d_eco - d_fast) / d_fast * 100:.1f}%")


    visualize_all_routes(path_short, path_fast, path_eco, graph, node_coords, file_name, comp_short, comp_fast, comp_eco)



setup_graph_from_osm_data(data)

main_network = get_largest_component(graph)
print(f"Main connected network size: {len(main_network)} nodes")


sample_start, sample_destination = random.sample(main_network, 2)
evaluate_and_compare_routes(graph, node_coords, sample_start, sample_destination)
#get_random_node_from_main_network(main_network)
#get_random_node_from_main_network(main_network)

valid_starts, valid_goals = validate_and_extract_nodes(graph, data, main_network, "Sheepfoot Lane", "Barlow Hall Road")
if  valid_starts and valid_goals:
        evaluate_and_compare_routes(graph, node_coords, valid_starts[0], valid_goals[0], "north_to_south.html")

# test_specific_road_to_road_route("Sheepfoot Lane", "Barlow Hall Road", main_network)
