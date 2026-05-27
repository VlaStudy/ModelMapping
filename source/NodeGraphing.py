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

from AstarGraphUtils import calculate_route_metrics

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
    output_dir = "MapOutputs"
    
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    absolute_file_path = os.path.abspath(os.path.join(output_dir, file_path))


    m.save(absolute_file_path)
    print(f"Map created! Saved locally to: {absolute_file_path}")
    
    webbrowser.open('file://' + absolute_file_path)
    
    
def visualize_route(path_node_ids, node_coords, file_name):
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
        color="green",       
        weight=6, 
        opacity=0.8
    ).add_to(m)

    # Add markers for Start and End
    folium.Marker(route_coords[0], popup="Start", icon=folium.Icon(color="blue")).add_to(m)
    folium.Marker(route_coords[-1], popup="Destination", icon=folium.Icon(color="red")).add_to(m)

    # Save to a file and open it automatically
    file_path = file_name
    output_dir = "MapOutputs"
    
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    absolute_file_path = os.path.abspath(os.path.join(output_dir, file_path))


    m.save(absolute_file_path)
    print(f"Map created! Saved locally to: {absolute_file_path}")
    
    webbrowser.open('file://' + absolute_file_path)



def _create_html_dashboard(title, color, dist, time, energy, co2):
    """Generates a clean HTML dashboard card for map tooltips."""
    return f"""
    <div style="font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; min-width: 220px; padding: 5px;">
        <h4 style="margin: 0 0 8px 0; color: {color}; font-size: 14px;">{title}</h4>
        <table style="width: 100%; font-size: 12px; border-collapse: collapse;">
            <tr style="border-bottom: 1px solid #eee;"><td style="padding: 3px 0;"><b>Distance:</b></td><td style="text-align: right;">{dist:.1f} m</td></tr>
            <tr style="border-bottom: 1px solid #eee;"><td style="padding: 3px 0;"><b>Est. Time:</b></td><td style="text-align: right;">{time:.1f} s</td></tr>
            <tr style="border-bottom: 1px solid #eee;"><td style="padding: 3px 0;"><b>Energy Used:</b></td><td style="text-align: right;">{energy:.2f} Wh</td></tr>
            <tr><td style="padding: 3px 0; color: {color};"><b>CO2 Footprint:</b></td><td style="text-align: right; font-weight: bold; color: {color};">{co2:.4f} kg</td></tr>
        </table>
    </div>
    """

def visualize_all_routes(shortest_path, fastest_path, eco_path, graph, node_coords, file_name):
    """
    Plots routes with live embedded carbon and energy dashboard overlays.
    """
    if not shortest_path or not fastest_path or not eco_path:
        print("Error: Missing path profiles. Cannot generate telemetry map.")
        return

    # Extract coordinates safely
    short_coords = [node_coords[n] for n in shortest_path if n in node_coords]
    fast_coords  = [node_coords[n] for n in fastest_path if n in node_coords]
    eco_coords    = [node_coords[n] for n in eco_path if n in node_coords]

    if not short_coords:
        print("Error: Empty coordinates array.")
        return

    # Generate the metrics data cards
    d1, t1, e1, c1 = calculate_route_metrics(shortest_path, graph)
    d2, t2, e2, c2 = calculate_route_metrics(fastest_path, graph)
    d3, t3, e3, c3 = calculate_route_metrics(eco_path, graph)

    # Initialize Base Map
    m = folium.Map(location=short_coords[0], zoom_start=13, tiles="OpenStreetMap")

    # Shortest Path Layer (Crimson)
    dash_short = _create_html_dashboard("📍 Shortest Route (Distance)", "crimson", d1, t1, e1, c1)
    folium.PolyLine(
        short_coords, color="crimson", weight=6, opacity=0.75,
        tooltip=folium.Tooltip(dash_short), name="Shortest Route (Red)"
    ).add_to(m)

    # Fastest Path Layer (Royal Blue)
    dash_fast = _create_html_dashboard("⚡ Fastest Route (Time)", "royalblue", d2, t2, e2, c2)
    folium.PolyLine(
        fast_coords, color="royalblue", weight=6, opacity=0.85,
        tooltip=folium.Tooltip(dash_fast), name="Fastest Route (Blue)"
    ).add_to(m)

    # Eco Path Layer (Green)
    dash_eco = _create_html_dashboard("🌿 Eco-Friendly Route (Energy)", "green", d3, t3, e3, c3)
    folium.PolyLine(
        eco_coords, color="green", weight=7, opacity=0.9,
        tooltip=folium.Tooltip(dash_eco), name="Eco-Friendly Route (Green)"
    ).add_to(m)

    # Pin Start and Destination Checkpoints
    folium.Marker(short_coords[0], popup="<b>Start</b>", icon=folium.Icon(color="green", icon="play")).add_to(m)
    folium.Marker(short_coords[-1], popup="<b>Destination</b>", icon=folium.Icon(color="black", icon="stop")).add_to(m)

    # Add Layer Controller Box
    folium.LayerControl(position="topright", collapsed=False).add_to(m)

    # Save absolute compilation pipeline
    output_dir = "MapOutputs"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    abs_path = os.path.abspath(os.path.join(output_dir, file_name))
    m.save(abs_path)
    print(f"Interactive dashboard generated successfully at: {abs_path}")
    webbrowser.open('file://' + abs_path)