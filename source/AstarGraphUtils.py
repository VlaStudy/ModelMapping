

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



