import numpy as np

def random_camera_position():
    x = np.random.uniform(-0.25, 0.25)
    y = np.random.uniform(-0.25, 0.25)
    z = np.random.uniform(0.3, 0.5)
    return np.array([x, y, z])

'''
def look_at_rotation(camera_pos, target):
    """ Compute Euler rotation from camera position to look-at target. """
    direction = target - camera_pos
    forward = direction / np.linalg.norm(direction)

    # Basic look-at using up-vector [0, 0, 1]
    up = np.array([0, 0, 1])
    right = np.cross(up, forward)
    right /= np.linalg.norm(right)
    up = np.cross(forward, right)

    # Construct rotation matrix
    rot_matrix = np.array([right, up, forward]).T

    # Convert rotation matrix to Euler angles (XYZ)
    from scipy.spatial.transform import Rotation as R
    r = R.from_matrix(rot_matrix)
    euler = r.as_euler('xyz', degrees=False)
    return euler
'''

def look_at_rotation_blender(camera_pos, target):
    """
    Computes Euler rotation (XYZ order) to make a Blender camera at `camera_pos`
    look at `target`. Blender cameras look along their -Z axis.
    """
    direction = target - camera_pos
    forward = -direction / np.linalg.norm(direction)  # Blender camera looks along -Z

    up = np.array([0.0, 0.0, 1.0])
    right = np.cross(up, forward)
    right /= np.linalg.norm(right)
    up_corrected = np.cross(forward, right)

    rot_matrix = np.stack([right, up_corrected, forward], axis=1)

    from scipy.spatial.transform import Rotation as R
    r = R.from_matrix(rot_matrix)
    return r.as_euler('xyz')


# Generate 10 camera poses
target = np.array([0.0, 0.0, 0.15])
poses = []

cam_pos = np.array([0.0, 0.0, 0.5])
cam_rot = np.array([0.0, 0.0, 0.0])
pose_line = list(cam_pos) + list(cam_rot)
poses.append(pose_line)

for _ in range(10):
    cam_pos = random_camera_position()
    cam_rot = look_at_rotation_blender(cam_pos, target)
    pose_line = list(cam_pos) + list(cam_rot)
    poses.append(pose_line)

# Save to text file
with open("camera_positions.txt", "w") as f:
    for pose in poses:
        line = " ".join([f"{val:.6f}" for val in pose])
        f.write(line + "\n")

print("Saved 10 camera poses to camera_positions.txt")
