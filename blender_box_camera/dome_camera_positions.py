import numpy as np
from scipy.spatial.transform import Rotation as R

def spherical_to_cartesian(radius, lat_deg, lon_deg):
    """
    Converts spherical coordinates to Cartesian.
    Latitude: 0 (equator) to 90 (north pole)
    Longitude: 0 to 360 degrees
    """
    lat = np.radians(lat_deg)
    lon = np.radians(lon_deg)
    
    #x = radius * np.cos(lat) * np.cos(lon); y = radius * np.cos(lat) * np.sin(lon); z = radius * np.sin(lat)
    x = radius * np.sin(lat) * np.cos(lon)
    y = radius * np.sin(lat) * np.sin(lon)
    z = radius * np.cos(lat)
    return np.array([x, y, z])
'''
def look_at_rotation_blender(camera_pos, target):
    """
    Computes Euler rotation (XYZ order) for Blender camera at `camera_pos`
    to look at `target`, considering Blender's -Z forward axis.
    """
    direction = target - camera_pos
    forward = -direction / np.linalg.norm(direction)  # Blender looks along -Z
    up = np.array([0.0, 0.0, 1.0])
    right = np.cross(up, forward)
    right /= np.linalg.norm(right)
    up_corrected = np.cross(forward, right)
    rot_matrix = np.stack([right, up_corrected, forward], axis=1)
    r = R.from_matrix(rot_matrix)
    return r.as_euler('xyz')
'''
def look_at_rotation_blender(camera_pos, target):
    direction = target - camera_pos
    forward = -direction / np.linalg.norm(direction)

    # Default up vector (Z-up)
    up = np.array([0.0, 0.0, 1.0])
    right = np.cross(up, forward)
    right_norm = np.linalg.norm(right)

    if right_norm < 1e-6:
        # forward is nearly parallel to up ⇒ choose alternative up vector
        up = np.array([1.0, 0.0, 0.0])
        right = np.cross(up, forward)
        right /= np.linalg.norm(right)
        up_corrected = np.cross(forward, right)
    else:
        right /= right_norm
        up_corrected = np.cross(forward, right)

    rot_matrix = np.stack([right, up_corrected, forward], axis=1)
    r = R.from_matrix(rot_matrix)
    return r.as_euler('xyz')

# Camera setup
radius = 1
min_lat = 0; max_lat = 70; step_lat = 70
min_lon = 0; max_lon = 370; step_lon = 360

target = np.array([0.0, 0.0, 0.15])  # or adjust target if needed
poses = []

cam_pos = np.array([0.0, 0.0, radius]) #0.5])
cam_rot = np.array([0.0, 0.0, 0.0])
pose_line = list(cam_pos) + list(cam_rot)
poses.append(pose_line)

# Generate spherical camera positions
for lat in range(min_lat, max_lat + 1, step_lat):
    for lon in range(min_lon, max_lon, step_lon):
        if lat == 0 and lon == 0:
            continue

        print(lat,lon)
        cam_pos = spherical_to_cartesian(radius, lat, lon)
        cam_rot = look_at_rotation_blender(cam_pos, target)
        pose_line = list(cam_pos) + list(cam_rot)
        poses.append(pose_line)

# Save to file
output_file = "camera_positions.txt"
with open(output_file, "w") as f:
    for pose in poses:
        line = " ".join([f"{val:.6f}" for val in pose])
        f.write(line + "\n")

print(f"Saved {len(poses)} spherical camera poses to {output_file}")

# Save to file
output_file2 = "dome_lat_lon.txt"
with open(output_file2, "w") as f:
    f.write(str(radius) + "\n")
    f.write(str(min_lat) + "\n"); f.write(str(max_lat) + "\n"); f.write(str(step_lat) + "\n")
    f.write(str(min_lon) + "\n"); f.write(str(max_lon) + "\n"); f.write(str(step_lon) + "\n")

print(f"Saved lat and lon data to {output_file2}")

# Save to file
output_file3 = "ring_radius_height.txt"
with open(output_file3, "w") as f: 
    f.write("")

print(f"Saved no ring radius and height to {output_file3}")