

import heapq
from AstarGraphUtils import haversine


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