import json
from collections import defaultdict
import math
import heapq

# Load the raw Overpass JSON
with open('C:\\Users\\23674569\\Downloads\\ModelMapping\\data\\export.json', 'r', encoding='utf-8') as f:
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


def a_star_search(graph, node_coords, start_id, goal_id):
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

# path = a_star_search(graph, node_coords, start_node, end_node)

# if path:
#     print(f"Path found! It consists of {len(path)} intersections/nodes.")
#     print("Sample of path nodes:", path[:5], "...", path[-5:])
# else:
#     print("No route could be found between those nodes.")

test_way = None
for element in data['elements']:
    if element['type'] == 'way' and 'nodes' in element and len(element['nodes']) > 1:
        test_way = element
        break

if test_way:
    connected_nodes = test_way['nodes']
    print(f"Success! Found a street (Way ID: {test_way['id']})")
    print(f"Use these two IDs to test your A* algorithm:")
    print(f"  Start Node ID: {connected_nodes[0]}")
    print(f"  End Node ID:   {connected_nodes[-1]}")
else:
    print("Error: No 'way' elements with 'nodes' arrays found. Try updating the Overpass query.")

sample_start = list(graph.keys())[0]
sample_destination = graph[sample_start][0][0]