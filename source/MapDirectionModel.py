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
with open('C:\\Users\\23674569\\Downloads\\ModelMapping\\data\\ManMap.json', 'r', encoding='utf-8') as f:
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
        
        # Connect sequential nodes in the way
        for i in range(len(way_nodes) - 1):
            node_a = way_nodes[i]
            node_b = way_nodes[i+1]
            
            # Ensure both nodes exist in our coordinate map
            if node_a in node_coords and node_b in node_coords:
                # Calculate physical distance (g-cost weight)
                dist = haversine(node_coords[node_a], node_coords[node_b])
                
                # Assuming two-way street for routing. 
                # (You can check element.get('tags', {}).get('oneway') if needed)
                graph[node_a].append((node_b, dist))
                graph[node_b].append((node_a, dist))


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
        for neighbor, edge_weight in graph[current_node]:
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
    """
    visited = set()
    largest_component = []

    for start_node in graph:
        if start_node not in visited:
            # Explore this specific island/component using BFS
            component = []
            queue = [start_node]
            visited.add(start_node)
            
            while queue:
                current = queue.pop(0)
                component.append(current)
                
                for neighbor, _ in graph[current]:
                    if neighbor not in visited:
                        visited.add(neighbor)
                        queue.append(neighbor)
            
            # Keep track of whichever component is the biggest
            if len(component) > len(largest_component):
                largest_component = component
                
    return largest_component

main_network = get_largest_component(graph)
print(f"Main connected network size: {len(main_network)} nodes")

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

    if nodes_1:
        print(f"SUCCESS! Path found consisting of {len(nodes_1)} nodes.")
    else:
        print("This should theoretically never happen now!")
else:
    print("Error: Could not find a valid connected network.")



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
    file_path = "eco_route_map.html"
    m.save(file_path)
    
    # Opens the map in your default web browser
    webbrowser.open('file://' + os.path.realpath(file_path))


visualize_route(nodes, node_coords)