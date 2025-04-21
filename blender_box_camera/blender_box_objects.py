import blenderproc as bproc
import bpy
import math
import os
import numpy as np
import mathutils

# Initialize BlenderProc
bproc.init()

'''
# Load your objects (make sure names match those saved from PyBullet)
obj1 = bproc.loader.load_obj("power_drill/textured.obj")
obj2 = bproc.loader.load_obj("scissors/textured.obj")
obj3 = bproc.loader.load_obj("shampoo/textured.obj")
obj4 = bproc.loader.load_obj("small_clamp/textured.obj")
objs = [obj1,obj2,obj3,obj4]
'''

# Load and name objects properly
objects = {}
for name in ["power_drill", "scissors", "shampoo", "small_clamp"]:
    obj = bproc.loader.load_obj(f"{name}/textured.obj")[0]  # take first from list
    obj.set_name(name)
    objects[name] = obj


def set_origin_to_bottom(obj):
    # Switch to object mode (required)
    bpy.ops.object.mode_set(mode='OBJECT')
    
    # Deselect all
    bpy.ops.object.select_all(action='DESELECT')
    
    # Select only the object
    obj.blender_obj.select_set(True)
    bpy.context.view_layer.objects.active = obj.blender_obj

    # Set origin to bottom
    bpy.ops.object.origin_set(type='ORIGIN_CURSOR', center='BOUNDS')
    
    # Move cursor to bottom of bounding box
    bbox = [mathutils.Vector(corner) for corner in obj.blender_obj.bound_box]
    min_corner = mathutils.Vector((min(v[0] for v in bbox),
                                   min(v[1] for v in bbox),
                                   min(v[2] for v in bbox)))
    center_x = (max(v[0] for v in bbox) + min(v[0] for v in bbox)) / 2
    center_y = (max(v[1] for v in bbox) + min(v[1] for v in bbox)) / 2
    bpy.context.scene.cursor.location = (center_x, center_y, min_corner.z)

    # Re-run origin set now that cursor is moved
    bpy.ops.object.origin_set(type='ORIGIN_CURSOR')

# Example usage:
#for obj in objects.values():
 #   set_origin_to_bottom(obj)


def set_origin_to_center(bproc_obj):
    """Sets the origin of a BlenderProc object to its bounding box center using bpy."""
    
    # Get the raw Blender object
    obj = bproc_obj.blender_obj

    # Deselect all
    bpy.ops.object.select_all(action='DESELECT')

    # Select and activate the target object
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj

    # Ensure in OBJECT mode
    if bpy.context.object.mode != 'OBJECT':
        bpy.ops.object.mode_set(mode='OBJECT')

    # Compute bounding box corners (in object space)
    bbox = [mathutils.Vector(corner) for corner in obj.bound_box]

    # Compute the center of the bounding box
    center = sum(bbox, mathutils.Vector()) / 8.0
    center_world = obj.matrix_world @ center

    # Move 3D cursor to the bounding box center
    bpy.context.scene.cursor.location = center_world

    # Set origin to 3D cursor
    bpy.ops.object.origin_set(type='ORIGIN_CURSOR')

for obj in objects.values():
    set_origin_to_center(obj)


# Load saved poses
pose_data = np.load("pybullet_poses.npz", allow_pickle=True)

for name, obj in objects.items():
    if name in pose_data:
        location = pose_data[name].item()["location"]
        quaternion = pose_data[name].item()["quaternion_xyzw"]
        obj.set_location(location)
        obj.set_rotation_quaternion(quaternion)

# Box Scaling
Xscale = 1.3
Yscale = 1.5
Zscale = 0.5
box_scale = [Xscale,Yscale,Zscale]
box_location = [0,0,Zscale]

# Create Open Box
def create_box(Xscale,Yscale,Zscale):
    # Clear existing objects in the scene
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)

    # Create the outer box (the container)
    bpy.ops.mesh.primitive_cube_add(size=2, location=(0, 0, Zscale))
    outer_box = bpy.context.object
    outer_box.name = 'Outer_Box'

    # Scale the outer box to give it the proportions of a box
    outer_box.scale = (Xscale, Yscale, Zscale)

    # Create the inner box (the hollow part of the container)
    bpy.ops.mesh.primitive_cube_add(size=2, location=(0, 0, Zscale))
    inner_box = bpy.context.object
    inner_box.name = 'Inner_Box'

    # Scale the inner box slightly smaller to create the hollow part
    inner_box.scale = (Xscale-0.1, Yscale-0.1, Zscale)

    # Set the inner box as a child of the outer box
    inner_box.select_set(True)
    outer_box.select_set(True)
    bpy.context.view_layer.objects.active = outer_box
    bpy.ops.object.parent_set(type='OBJECT')

    # Add a Boolean modifier to the outer box to subtract the inner box (making it hollow)
    bpy.ops.object.modifier_add(type='BOOLEAN')
    outer_box.modifiers["Boolean"].operation = 'DIFFERENCE'
    outer_box.modifiers["Boolean"].use_self = False
    outer_box.modifiers["Boolean"].object = inner_box

    # Apply the Boolean modifier to create the open box
    bpy.ops.object.modifier_apply(modifier="Boolean")

    # Deselect all objects
    bpy.ops.object.select_all(action='DESELECT')

    # Select only the inner box to delete it
    inner_box.select_set(True)
    bpy.ops.object.delete(use_global=False)

    # Add a bottom face manually (creating a mesh that will act as the bottom of the box)
    bpy.ops.mesh.primitive_plane_add(size=2, location=(0, 0, 0))
    bottom_face = bpy.context.object
    bottom_face.name = 'Bottom_Face'
    bottom_face.scale = (Xscale, Yscale, 0.1)

    # Optionally, smooth the outer surface of the container (you can remove this if you prefer sharp edges)
    bpy.ops.object.shade_smooth()

    # Optionally join the bottom face to the outer box
    outer_box.select_set(True)
    bottom_face.select_set(True)
    bpy.context.view_layer.objects.active = outer_box
    bpy.ops.object.join()

create_box(Xscale,Yscale,Zscale)

'''
lbb_box = [0.1, -0.15, 0]  # Left-Bottom-Back corner
dimensions = [0.44,0.6,0.2] #[0.22, 0.3, 0.1]  # Dimensions [L, W, H]
box_center = np.array(lbb_box) + np.array(dimensions) / 2

wall_thickness = 0.01

def create_wall(pos, scale):
    wall = bproc.object.create_primitive("CUBE", scale=scale, location=pos)
    wall.set_cp("physics", {"type": "static"})
    return wall

x, y, z = lbb_box
lx, ly, lz = dimensions

# Bottom (optional visual only)
create_wall(
    pos=[x + lx / 2, y + ly / 2, z + wall_thickness / 2],
    scale=[lx, ly, wall_thickness]
)

# Side walls
create_wall([x + wall_thickness / 2, y + ly / 2, z + lz / 2], [wall_thickness, ly, lz])  # Left
create_wall([x + lx - wall_thickness / 2, y + ly / 2, z + lz / 2], [wall_thickness, ly, lz])  # Right
create_wall([x + lx / 2, y + wall_thickness / 2, z + lz / 2], [lx, wall_thickness, lz])  # Back
create_wall([x + lx / 2, y + ly - wall_thickness / 2, z + lz / 2], [lx, wall_thickness, lz])  # Front
'''


            # camera orientation



def direction_to_quaternion(direction):
    """
    Convert a direction vector into a quaternion that makes the camera point towards the target.
    """
    # Forward vector (camera faces along the -Z axis)
    forward = mathutils.Vector((0, 0, -1))

    # Calculate the rotation needed to align the forward vector with the direction vector
    axis = forward.cross(direction)  # Axis of rotation (perpendicular vector)
    angle = forward.angle(direction)  # Angle between forward and direction

    # If the angle is zero (meaning the vectors are already aligned), return identity quaternion
    if angle == 0.0:
        return mathutils.Quaternion((1, 0, 0, 0))

    # Create the quaternion representing the rotation
    rotation_quaternion = mathutils.Quaternion(axis, angle)
    
    return rotation_quaternion

def point_at_target(camera, target_location):
    """
    Make the camera point towards a target location using matrix calculations.
    """
    # Get the vector from the camera to the target
    direction = (target_location[0] - camera.location[0],
                target_location[1] - camera.location[1],
                target_location[2] - camera.location[2])
    
    # Normalize the direction vector
    length = math.sqrt(direction[0]**2 + direction[1]**2 + direction[2]**2)
    direction = (direction[0] / length, direction[1] / length, direction[2] / length)

    # Create a quaternion from the direction vector
    rotation_quaternion = direction_to_quaternion(direction)

    # Apply the quaternion rotation to the camera
    camera.rotation_mode = 'QUATERNION'  # Set rotation mode to quaternion
    camera.rotation_quaternion = rotation_quaternion


# Set up the camera (ensure there is a camera object in the scene)
if "Camera" not in bpy.data.objects:
    bpy.ops.object.camera_add(location=(0, 0, 5))  # Add a camera at (0, 0, 10) if it doesn't exist
camera = bpy.data.objects["Camera"]

# Set up the scene to use the camera
bpy.context.scene.camera = camera

camera.location = (0, 0, 2)

point_at_target(camera, (0, 0, 0))

# Render the image
output_path = os.path.join("output", f"1.png")
bpy.context.scene.render.filepath = output_path
bpy.ops.render.render(write_still=True)


'''
# Define camera location and target
#cam_location = mathutils.Vector([box_center[0],box_center[1],2])
cam_location = mathutils.Vector([0,0,2])
look_at = mathutils.Vector([0, 0, 0])
forward_vec = (look_at - cam_location).normalized()

# Get rotation matrix from forward vector (returns NumPy array)
rotation_np = bproc.camera.rotation_from_forward_vec(forward_vec)

# Convert NumPy array to mathutils.Matrix
rotation_blender = mathutils.Matrix([list(rotation_np[i]) for i in range(3)]).transposed()

# Build full camera pose matrix
cam_pose = mathutils.Matrix.Translation(cam_location) @ rotation_blender.to_4x4()


# Add to BlenderProc
bproc.camera.add_camera_pose(cam_pose)

# Render the image
output_path = os.path.join("outputs", f"1.png")
bpy.context.scene.render.filepath = output_path
bpy.ops.render.render(write_still=True)
'''