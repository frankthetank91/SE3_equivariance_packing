import blenderproc as bproc
import argparse
import numpy as np
import matplotlib.pyplot as plt
import os

parser = argparse.ArgumentParser()
parser.add_argument('camera', nargs='?', default="camera_positions.txt", help="Path to the camera file")
parser.add_argument('scene', nargs='?', default="my_scene.blend", help="Path to the scene.obj file")
parser.add_argument('output_dir', nargs='?', default="output", help="Path to where the final files, will be saved")
args = parser.parse_args()

bproc.init()

# load the objects into the scene
objs = bproc.loader.load_blend(args.scene)

# define a light and set its location and energy level
light = bproc.types.Light()
light.set_type("POINT")
light.set_location([0.5, -0.5, 2])
light.set_energy(100)

# define the camera intrinsics
bproc.camera.set_resolution(512, 512)

# read the camera positions file and convert into homogeneous camera-world transformation
with open(args.camera, "r") as f:
    for line in f.readlines():
        line = [float(x) for x in line.split()]
        position, euler_rotation = line[:3], line[3:6]
        matrix_world = bproc.math.build_transformation_mat(position, euler_rotation)
        bproc.camera.add_camera_pose(matrix_world)

for i, obj in enumerate(objs):
    obj.set_cp("category_id", i)  # Each object gets a unique ID

# activate depth rendering
bproc.renderer.enable_depth_output(activate_antialiasing=False)
# enable segmentation masks (per class and per instance)
bproc.renderer.enable_segmentation_output(map_by=["category_id", "instance", "name"])


# --------------------------
# Render and save outputs per camera
# --------------------------
#for i in range(len(bproc.camera.get_camera_pose())):
#    print(f"Rendering frame {i}...")
data = bproc.renderer.render()

depth = np.array(data["depth"])
# Inspect depth values
if "depth" in data:
    print("Depth min:", np.min(data["depth"]))
    print("Depth max:", np.max(data["depth"]))
else:
    print("WARNING: Depth data was not rendered.")

max_depth = np.max(data["depth"]) # Adjust based on your scene's scale

# Mask and normalize depth
depth_masked = np.where(depth > max_depth, np.nan, depth)
depth_min = np.nanmin(depth_masked)
depth_max_val = np.nanmax(depth_masked)

if depth_max_val - depth_min > 1e-5:
    depth_normalized = (depth_masked - depth_min) / (depth_max_val - depth_min) * 20.0
else:
    depth_normalized = np.zeros_like(depth_masked)

data["depth"] = depth_normalized

# --------------------------
# Save .hdf5 file
# --------------------------
hdf5_path = os.path.join(args.output_dir)#, f"frame_{i:03}.hdf5")
bproc.writer.write_hdf5(hdf5_path, data)
print(f"Saved HDF5: {hdf5_path}")

print("Rendering complete.")

'''
# render the whole pipeline
data = bproc.renderer.render()
print("Data keys:", data.keys())

depth = np.array(data["depth"])
#depth_masked = np.where(depth > 1, np.nan, depth)
# Define the expected maximum depth
max_depth = 1.0  # Adjust this value based on your scene's scale
depth_masked = np.where(depth > max_depth, np.nan, depth)

# Normalize to range [0, 20] ignoring NaNs
depth_min = np.nanmin(depth_masked)
depth_max = np.nanmax(depth_masked)

# Avoid divide-by-zero if min and max are equal
if depth_max - depth_min > 1e-5:
    depth_normalized = (depth_masked - depth_min) / (depth_max - depth_min) * 20.0
else:
    depth_normalized = np.zeros_like(depth_masked)

data["depth"] = depth_normalized

# Inspect depth values
if "depth" in data:
    print("Depth min:", np.min(data["depth"]))
    print("Depth max:", np.max(data["depth"]))
else:
    print("WARNING: Depth data was not rendered.")

import matplotlib.pyplot as plt
depth_clamped = np.clip(depth, 0, 0.5)
plt.imshow(np.squeeze(depth_clamped), cmap='gray',  vmin=0, vmax=0.5)
plt.colorbar()
plt.savefig("depth_visualization.png")

    
# write the data to a .hdf5 container
bproc.writer.write_hdf5(args.output_dir, data)
'''


