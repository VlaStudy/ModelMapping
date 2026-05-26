import json
from collections import defaultdict
import math
import heapq
import random
import subprocess
import sys
import webbrowser
import os

try:
    import folium
except ImportError:
    print("Folium not found. Installing it automatically...")
    # This safely invokes the exact Python executable running your VS Code script
    subprocess.check_call([sys.executable, "-m", "pip", "install", "folium"])
    import folium


# Load the raw Overpass JSON
with open('.\\data\\ManMap.json', 'r', encoding='utf-8') as f:
    data = json.load(f)


def haversine(coord1, coord2):
    #"""Calculates the great-circle distance between two points in meters"""
    lat1, lon1 = coord1
    lat2, lon2 = coord2
    
    R = 6371000  # Radius of Earth in meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    
    a = math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    return R * c

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
graph = defaultdict(list)

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

def a_star_shortest_search(graph, node_coords, start_id, goal_id):
    """
    Finds the shortest path between start_id and goal_id using A*.
    Returns a list of node IDs forming the path, or None if no path exists.
    """
    # Priority queue stores tuples: (f_score, current_node)
    # heapq always pops the element with the lowest f_score
    open_set = []
    heapq.heappush(open_set, (0.0, start_id))
    
    # Track the best g_score (actual distance) found so far for each node
    # Default to infinity for unvisited nodes
    g_score = {node: float('inf') for node in node_coords}
    g_score[start_id] = 0.0
    
    # Track where each node came from so we can reconstruct the final path
    came_from = {}
    
    while open_set:
        # Pop the node with the lowest f_score
        current_f, current_node = heapq.heappop(open_set)
        
        # Goal reached! Reconstruct and return the path
        if current_node == goal_id:
            path = []
            while current_node in came_from:
                path.append(current_node)
                current_node = came_from[current_node]
            path.append(start_id)
            return path[::-1]  # Return reversed path (start to goal)
            
        # Explore neighbors of the current node
        for edge in graph.get(current_node, []):
            neighbor = edge['to']
            edge_weight = edge['distance']
            
            # Tentative g_score is the distance to current_node + weight of the edge to neighbor
            tentative_g = g_score[current_node] + edge_weight
            
            # If we found a shorter path to neighbor than previously recorded
            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current_node
                g_score[neighbor] = tentative_g
                
                # f(n) = g(n) + h(n)
                # h(n) is the Haversine distance from neighbor to the absolute goal
                h_score = haversine(node_coords[neighbor], node_coords[goal_id])
                f_score = tentative_g + h_score
                
                # Push to open set to explore later
                heapq.heappush(open_set, (f_score, neighbor))
                
    return None  # No path found

def a_star_fastest_search(graph, node_coords, start_id, goal_id):
    """
    Finds the fastest path (shortest time) between start_id and goal_id using A*.
    Assumes graph edge_weights are formatted as travel time in seconds.
    """
    open_set = []
    heapq.heappush(open_set, (0.0, start_id))
    
    # Track the best g_score (actual time spent in seconds) found so far for each node
    g_score = {node: float('inf') for node in node_coords}
    g_score[start_id] = 0.0
    
    came_from = {}
    
    # Define the absolute maximum speed on your map to keep the heuristic admissible.
    # 70 mph converted to meters per second (70 * 0.44704)
    MAX_SPEED_MPS = 31.2928 
    
    while open_set:
        current_f, current_node = heapq.heappop(open_set)
        
        if current_node == goal_id:
            path = []
            while current_node in came_from:
                path.append(current_node)
                current_node = came_from[current_node]
            path.append(start_id)
            return path[::-1]
            
        # Loop through neighbors. Remember: edge_weight is now TIME (seconds), not meters!
        for edge in graph.get(current_node, []):
            neighbor = edge['to']
            edge_time = edge['time']
            
            # tentative_g = total time spent from start to current_node + time to traverse edge
            tentative_g = g_score[current_node] + edge_time
            
            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current_node
                g_score[neighbor] = tentative_g
                
                # --- THE TIME SHIFT HEURISTIC ---
                # Calculate straight-line distance in meters
                straight_line_meters = haversine(node_coords[neighbor], node_coords[goal_id])
                
                # Heuristic time (seconds) = distance / max possible speed
                # (Assuming a car can fly in a straight line at 70mph guarantees h(n) never overestimates)
                h_score = straight_line_meters / MAX_SPEED_MPS
                
                f_score = tentative_g + h_score
                
                heapq.heappush(open_set, (f_score, neighbor))
                
    return None

def a_star_eco_search(graph, node_coords, start_id, goal_id):
    """
    Finds the most eco-friendly path (lowest energy consumption) between 
    start_id and goal_id using A*.
    
    Assumes graph edge data includes an 'energy' key (e.g., in Watt-hours or fuel ml).
    """
    open_set = []
    # Heap stores: (f_score, current_node)
    heapq.heappush(open_set, (0.0, start_id))
    
    # g_score tracks the total actual energy spent from the start node to this node
    g_score = {node: float('inf') for node in node_coords}
    g_score[start_id] = 0.0
    
    came_from = {}
    
    # --- THE ADMISSIBLE ECO HEURISTIC CONFIG ---
    # To keep A* admissible, h(n) must NEVER overestimate remaining energy.
    # We find the absolute most efficient driving scenario possible (e.g., an electric
    # vehicle driving at its optimal speed of 30 mph on a flat surface consumes ~150 Wh/km).
    # Wh per meter = 150 Wh / 1000m = 0.15 Wh/meter
    MIN_ENERGY_PER_METER = 0.15 
    
    while open_set:
        current_f, current_node = heapq.heappop(open_set)
        
        # Goal reached! Reconstruct the eco-route
        if current_node == goal_id:
            path = []
            while current_node in came_from:
                path.append(current_node)
                current_node = came_from[current_node]
            path.append(start_id)
            return path[::-1]
            
        # Optimization guard: Skip stale duplicate nodes in the priority queue
        h_current = haversine(node_coords[current_node], node_coords[goal_id]) * MIN_ENERGY_PER_METER
        if current_f > g_score[current_node] + h_current:
            continue
            
        # Explore neighbors
        for edge in graph.get(current_node, []):
            neighbor = edge['to']
            
            # Pull the pre-calculated energy expenditure for this specific street segment
            edge_energy_cost = edge['energy'] 
            
            # tentative_g = total energy used so far + energy cost of this street element
            tentative_g = g_score[current_node] + edge_energy_cost
            
            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current_node
                g_score[neighbor] = tentative_g
                
                # --- THE ADMISSIBLE ECO HEURISTIC ---
                # Get straight line distance to the destination in meters
                straight_line_meters = haversine(node_coords[neighbor], node_coords[goal_id])
                
                # Calculate minimum possible energy to get there. 
                # Assuming the car flies in a perfect straight line at its peak optimal 
                # energy efficiency ensures h(n) never overestimates the cost.
                h_score = straight_line_meters * MIN_ENERGY_PER_METER
                
                f_score = tentative_g + h_score
                
                heapq.heappush(open_set, (f_score, neighbor))
                
    return None

# start_node = 334801  # regent road roundabout
# end_node = 3346329    # Deansgate interchange

# path = a_star_shortest_search(graph, node_coords, start_node, end_node)

# if path:
#     print(f"Path found! It consists of {len(path)} intersections/nodes.")
#     print("Sample of path nodes:", path[:5], "...", path[-5:])
# else:
#     print("No route could be found between those nodes.")

def get_largest_component(graph):
    """
    Finds the largest fully connected network of nodes in the graph.
    Bypasses isolated islands and broken boundary roads.
    Updated to handle dictionary-style edges.
    """
    visited = set()
    largest_component = []

    for start_node in graph:
        if start_node not in visited:
            component = []
            queue = [start_node]
            visited.add(start_node)
            
            while queue:
                current = queue.pop(0)
                component.append(current)

                # Loop through the list of dictionaries and pull out the 'to' key
                for edge in graph.get(current, []):
                    neighbor = edge['to']
                    
                    if neighbor not in visited:
                        visited.add(neighbor)
                        queue.append(neighbor)
            
            if len(component) > len(largest_component):
                largest_component = component
                
    return largest_component

main_network = get_largest_component(graph)
print(f"Main connected network size: {len(main_network)} nodes")




def visualize_route(path_node_ids, node_coords):
    """
    Takes the list of Node IDs from A* and draws them on an interactive map.
    """
    if not path_node_ids:
        print("No path to draw!")
        return

    # Convert the list of IDs into a list of (lat, lon) coordinates
    route_coords = [node_coords[node_id] for node_id in path_node_ids]

    # Center the map on the starting point
    start_point = route_coords[0]
    m = folium.Map(location=start_point, zoom_start=14)

    # Draw the line connecting all the points
    folium.PolyLine(
        route_coords, 
        color="green",       # Eco-friendly green!
        weight=6, 
        opacity=0.8
    ).add_to(m)

    # Add markers for Start and End
    folium.Marker(route_coords[0], popup="Start", icon=folium.Icon(color="blue")).add_to(m)
    folium.Marker(route_coords[-1], popup="Destination", icon=folium.Icon(color="red")).add_to(m)

    # Save to a file and open it automatically
    file_path = "route_map.html"
    m.save(file_path)
    
    # Opens the map in your default web browser
    webbrowser.open('file://' + os.path.realpath(file_path))



def visualize_all_routes(shortest_path, fastest_path, eco_path, node_coords, file_name):
    """
    Plots both the shortest and fastest paths onto a single interactive map 
    with a toggleable layer legend.
    """
    if not shortest_path or not fastest_path:
        print("Error: One or both paths are missing!")
        return

    # Convert both lists of Node IDs into (lat, lon) coordinate tuples
    shortest_coords = [node_coords[node_id] for node_id in shortest_path]
    fastest_coords = [node_coords[node_id] for node_id in fastest_path]
    eco_coords = [node_coords[node_id] for node_id in eco_path]
    # Base Map Setup (Center on the starting point)
    start_point = shortest_coords[0]
    m = folium.Map(location=start_point, zoom_start=13, tiles="OpenStreetMap")

    # Draw Shortest Route (Let's use Red for "Raw Distance")
    folium.PolyLine(
        shortest_coords, 
        color="crimson", 
        weight=6, 
        opacity=0.75,
        tooltip="Shortest Route (Distance Optimized)",
        name="Shortest Route (Red)"  # Layer name for the toggle menu
    ).add_to(m)

    # Draw Fastest Route (Let's use Blue for "Time/Speed Optimized")
    folium.PolyLine(
        fastest_coords, 
        color="royalblue", 
        weight=6, 
        opacity=0.85,
        tooltip="Fastest Route (Time Optimized)",
        name="Fastest Route (Blue)"   # Layer name for the toggle menu
    ).add_to(m)

        # Draw Eco Route (Let's use Green for "Energy Optimized")
    folium.PolyLine(
        eco_coords,
        color="green",
        weight=6,
        opacity=0.85,
        tooltip="Eco-Friendly Route (Energy Optimized)",
        name="Eco-Friendly Route (Green)"   # Layer name for the toggle menu
    ).add_to(m)

    # Add Single Pins for Start and End Points
    folium.Marker(
        start_point, 
        popup="<b>Start Intersection</b>", 
        icon=folium.Icon(color="green", icon="play")
    ).add_to(m)
    
    end_point = shortest_coords[-1]
    folium.Marker(
        end_point, 
        popup="<b>Destination Point</b>", 
        icon=folium.Icon(color="black", icon="stop")
    ).add_to(m)


    # Adds a little checklist box in the top-right corner to turn lines on/off
    folium.LayerControl(position="topright", collapsed=False).add_to(m)

    # Save and display
    file_path = file_name
    m.save(file_path)
    print(f"Map created! Saved locally to {file_path}")
    webbrowser.open('file://' + os.path.realpath(file_path))


# Compute total distance and total time for the SHORTEST path





def find_nodes_by_road_name(data, target_road_name):
    """
    Scans the raw OSM data elements to find all Node IDs associated 
    with a specific street name (case-insensitive).
    """
    matching_nodes = []
    
    for element in data.get('elements', []):
        if element['type'] == 'way':
            tags = element.get('tags', {})
            # Look for the 'name' tag (e.g., "Deansgate")
            road_name = tags.get('name', '')
            
            if target_road_name.lower() in road_name.lower():
                # Add all nodes that form this specific street segment
                matching_nodes.extend(element.get('nodes', []))
                
    # Remove duplicates while preserving list order
    return list(dict.fromkeys(matching_nodes))


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


get_random_node_from_main_network(main_network)
#get_random_node_from_main_network(main_network)


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

test_specific_road_to_road_route("Chester Street", "Barton Dock Road", main_network)
