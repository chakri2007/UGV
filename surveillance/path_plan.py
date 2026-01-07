import json
import math
from xml.etree import ElementTree as ET

def parse_kml(file_path):
    tree = ET.parse(file_path)
    root = tree.getroot()
    ns = {'kml': 'http://www.opengis.net/kml/2.2'}
    coords_elem = root.find('.//kml:coordinates', ns)
    if coords_elem is None:
        raise ValueError("No coordinates found in KML file.")
    coords_str = coords_elem.text.strip()
    coords = []
    for point in coords_str.split():
        parts = point.split(',')
        if len(parts) < 2:
            continue
        lon, lat = map(float, parts[:2])  # ignore alt if present
        coords.append((lat, lon))
    if len(coords) < 3:
        raise ValueError("Insufficient coordinates for a polygon.")
    return coords

def generate_lawnmower_waypoints(coords, spacing_meters):
    min_lat = min(lat for lat, lon in coords)
    max_lat = max(lat for lat, lon in coords)
    min_lon = min(lon for lat, lon in coords)
    max_lon = max(lon for lat, lon in coords)
    
    avg_lat = (min_lat + max_lat) / 2
    meters_per_deg_lat = 111320
    meters_per_deg_lon = 111320 * math.cos(math.radians(avg_lat))
    
    delta_lat_m = (max_lat - min_lat) * meters_per_deg_lat
    delta_lon_m = (max_lon - min_lon) * meters_per_deg_lon
    
    is_horizontal = delta_lon_m >= delta_lat_m  # choose direction with longer dimension
    
    waypoints = []
    id_counter = 1
    
    if is_horizontal:
        # Horizontal lines (zigzag left-right)
        step_lat = spacing_meters / meters_per_deg_lat
        current_lat = min_lat
        direction = 1  # 1: left to right, -1: right to left
        while current_lat <= max_lat + 1e-10:
            if direction == 1:
                waypoints.append({"id": id_counter, "lat": current_lat, "longitude": min_lon, "reached": False})
                id_counter += 1
                waypoints.append({"id": id_counter, "lat": current_lat, "longitude": max_lon, "reached": False})
                id_counter += 1
            else:
                waypoints.append({"id": id_counter, "lat": current_lat, "longitude": max_lon, "reached": False})
                id_counter += 1
                waypoints.append({"id": id_counter, "lat": current_lat, "longitude": min_lon, "reached": False})
                id_counter += 1
            current_lat += step_lat
            direction *= -1
    else:
        # Vertical lines (zigzag up-down)
        step_lon = spacing_meters / meters_per_deg_lon
        current_lon = min_lon
        direction = 1  # 1: bottom to top, -1: top to bottom
        while current_lon <= max_lon + 1e-10:
            if direction == 1:
                waypoints.append({"id": id_counter, "lat": min_lat, "longitude": current_lon, "reached": False})
                id_counter += 1
                waypoints.append({"id": id_counter, "lat": max_lat, "longitude": current_lon, "reached": False})
                id_counter += 1
            else:
                waypoints.append({"id": id_counter, "lat": max_lat, "longitude": current_lon, "reached": False})
                id_counter += 1
                waypoints.append({"id": id_counter, "lat": min_lat, "longitude": current_lon, "reached": False})
                id_counter += 1
            current_lon += step_lon
            direction *= -1
    
    return waypoints

if __name__ == "__main__":
    # Hardcoded values - replace these with your actual paths and spacing
    input_file = 'area.kml'  # e.g., '/path/to/your/input.kml'
    output_file = 'waypoints.json'  # e.g., '/path/to/save/waypoints.json'
    spacing = 10.0  # Spacing in meters
    
    try:
        coords = parse_kml(input_file)
        waypoints = generate_lawnmower_waypoints(coords, spacing)
        with open(output_file, 'w') as f:
            json.dump(waypoints, f, indent=4)
        print(f"Waypoints generated and saved to {output_file}")
    except Exception as e:
        print(f"Error: {str(e)}")