import numpy as np
from scipy.spatial.transform import Rotation as R

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

# Parameters
N = 5                # Number of camera positions
radius = 1           # Radius of the circle
height = 1           # Fixed z height
target = np.array([0.0, 0.0, 0.15])  # Point the camera looks at

# Generate camera poses
poses = []

cam_pos = np.array([0.0, 0.0, height]) #0.5])
cam_rot = np.array([0.0, 0.0, 0.0])
pose_line = list(cam_pos) + list(cam_rot)
poses.append(pose_line)

for i in range(N):
    angle = 2 * np.pi * i / N
    x = radius * np.cos(angle)
    y = radius * np.sin(angle)
    z = height
    cam_pos = np.array([x, y, z])
    cam_rot = look_at_rotation_blender(cam_pos, target)
    pose_line = list(cam_pos) + list(cam_rot)
    poses.append(pose_line)

# Save to file
output_file = "camera_positions.txt"
with open(output_file, "w") as f:
    for pose in poses:
        line = " ".join([f"{val:.6f}" for val in pose])
        f.write(line + "\n")

print(f"Saved {len(poses)} circular camera poses to {output_file}")

# Save to file
output_file2 = "ring_radius_height.txt"
with open(output_file2, "w") as f:
    f.write(str(N) + "\n"); f.write(str(radius) + "\n"); f.write(str(height) + "\n")

print(f"Saved ring number, radius, and height to {output_file2}")

# Save to file
output_file3 = "dome_lat_lon.txt"
with open(output_file3, "w") as f:
    f.write("")

print(f"Saved no lat and lon data to {output_file3}")