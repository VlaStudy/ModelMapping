

import sys
import subprocess

try:
    from codecarbon import OfflineEmissionsTracker
except ImportError:
    print("CodeCarbon not found. Installing it automatically...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "codecarbon"])
    from codecarbon import OfflineEmissionsTracker

import math


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


def calculate_route_metrics(path, graph):
    """
    Accurately sums up the real physical distance, travel time, 
    and vehicle CO2 emissions by traversing the exact sequential path edges.
    """
    total_dist = 0.0
    total_time = 0.0
    total_energy_wh = 0.0
    
    # UK electricity grid carbon intensity factor (~120g CO2 per kWh)
    KG_CO2_PER_WH = 0.00012 

    for i in range(len(path) - 1):
        current_node = path[i]
        next_node = path[i+1]
        
        # Find the explicit edge connecting current_node -> next_node
        found_edge = False
        for edge in graph.get(current_node, []):
            if edge['to'] == next_node:
                total_dist += edge['distance']
                total_time += edge['time']
                total_energy_wh += edge['energy']
                found_edge = True
                break
                
    vehicle_co2_kg = total_energy_wh * KG_CO2_PER_WH
    return total_dist, total_time,total_energy_wh, vehicle_co2_kg


def profile_algorithm_compute(search_function, graph, node_coords, start_id, goal_id, name):
    """
    Runs an A* search variant while isolating and capturing its exact 
    computational CPU energy and CO2 footprint.
    """
    # Initialize an isolated, quiet offline tracker for the country code
    tracker = OfflineEmissionsTracker(
        country_iso_code="GBR",
        save_to_file=False,      # Stops it from cluttering folder with CSVs
        log_level="error"        # Keeps terminal clean of background messages
    )
    
    tracker.start()
    calculated_path = search_function(graph, node_coords, start_id, goal_id)
    emissions_kg = tracker.stop() # Returns total kg of CO2 emitted by this execution block
    
    # Convert kg to milligrams for display, since single A* passes are highly efficient!
    emissions_mg = emissions_kg * 1_000_000 if emissions_kg is not None else 0.0
    
    return calculated_path, emissions_mg

def validate_and_extract_nodes(graph, data, main_network, start_road_query, goal_road_query):
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
        return False
    if not valid_goals:
        print(f"Error: Could not find any valid connected graph entries for '{goal_road_query}'")
        return False
    return valid_starts, valid_goals