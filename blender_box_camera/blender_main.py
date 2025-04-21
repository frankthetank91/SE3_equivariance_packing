import blenderproc as bproc
import bpy
import imageio
import random
import numpy as np
from PIL import Image
import os
import math
import mathutils


obj_path = "/Users/mikaylalahr/Desktop/BlenderProc-main/my_code/textured.obj"
obj_pos = (0,0.2,0)
obj_rot = (0,0,0)
texture_path = "/Users/mikaylalahr/Desktop/BlenderProc-main/my_code/texture_map.jpg"

obj_two_path = "/Users/mikaylalahr/Desktop/BlenderProc-main/my_code/small_clamp/textured.obj"
obj_two_pos = (0,-0.2,0)
obj_two_rot = (0,0,0)
texture_two_path = "/Users/mikaylalahr/Desktop/BlenderProc-main/my_code/small_clamp/texture_map.jpg"

obj_three_path = "/Users/mikaylalahr/Desktop/BlenderProc-main/my_code/shampoo/textured.obj"
obj_three_pos = (0,0,0)
obj_three_rot = (0,0,0)
texture_three_path = "/Users/mikaylalahr/Desktop/BlenderProc-main/my_code/shampoo/texture_map.jpg"

obj_four_path = "/Users/mikaylalahr/Desktop/BlenderProc-main/my_code/scissors/textured.obj"
obj_four_pos = (0,0,0)
obj_four_rot = (0,0,0)
texture_four_path = "/Users/mikaylalahr/Desktop/BlenderProc-main/my_code/scissors/texture_map.png"

# Box Scaling
Xscale = 1.3
Yscale = 1.5
Zscale = 0.5
box_scale = [Xscale,Yscale,Zscale]
box_location = [0,0,Zscale]
scale = (3,3,3) # object scale

# Camera Setup
radius = 6
min_lat = 0; max_lat = 70; step_lat = 10 # Latitude
min_lon = 0; max_lon = 360; step_lon = 30 # Longitude

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

# Random Position In Box
def random_position_within_open_box(box_location, box_scale, rx_range=(0, np.pi/2), ry_range=(0, np.pi/2), rz_range=(0, np.pi/2)):
    """
    Generate a random position and rotation within the confines of an open box (no top).

    Args:
        box_location (tuple): The center coordinates of the box (x, y, z).
        box_scale (tuple): The scale of the box (L, W, H).
        rx_range (tuple): Range for random rotation around the X-axis.
        ry_range (tuple): Range for random rotation around the Y-axis.
        rz_range (tuple): Range for random rotation around the Z-axis.

    Returns:
        tuple: (x, y, z, rx, ry, rz) random position and rotation.
    """
    # Unpack the location and scale
    x_center, y_center, z_center = box_location
    L, W, H = box_scale

    # Generate random position within the box
    x = random.uniform(x_center - L / 2, x_center + L / 2)  # x-range from center - L/2 to center + L/2
    y = random.uniform(y_center - W / 2, y_center + W / 2)  # y-range from center - W/2 to center + W/2
    z = random.uniform(z_center, z_center + H / 2)  # z-range from center to center + H/2 (open top)

    # Generate random rotations within the specified ranges
    rx = random.uniform(*rx_range)
    ry = random.uniform(*ry_range)
    rz = random.uniform(*rz_range)

    return (x, y, z, rx, ry, rz)

# Load Objects with Texture
def load_obj_texture_one(obj_path,obj_pos,obj_rot,texture_path,scale):
    #bpy.ops.mesh.primitive_monkey_add()

    # Object 1
    obj = bproc.loader.load_obj(obj_path)
    obj[0].set_location(obj_pos)
    #obj[0].set_rotation_euler(obj_rot)
    obj[0].set_rotation_euler(obj_rot)

    # Set the object's scale
    obj[0].set_scale(scale)
    
    material = bpy.data.materials.new(name="TexturedMaterial")
    material.use_nodes = True  # Enable the use of nodes for the material

    nodes = material.node_tree.nodes
    texture_node = nodes.new(type='ShaderNodeTexImage')

    image = bpy.data.images.load(texture_path)
    texture_node.image = image

    principled_bsdf = nodes.get("Principled BSDF")
    material.node_tree.links.new(texture_node.outputs["Color"], principled_bsdf.inputs["Base Color"])

    obj = bpy.context.active_object  
    if obj.data.materials:
        obj.data.materials[0] = material  # If the object already has a material, replace the first one
    else:
        obj.data.materials.append(material) 

def load_obj_texture_two(obj_path,obj_pos,obj_rot,texture_path,scale):
    #bpy.ops.mesh.primitive_monkey_add()

    # Object 2
    obj = bproc.loader.load_obj(obj_path)
    obj[0].set_location(obj_pos)
    obj[0].set_rotation_euler(obj_rot)
    obj[0].set_scale(scale)
    
    material = bpy.data.materials.new(name="TexturedMaterial")
    material.use_nodes = True  # Enable the use of nodes for the material

    nodes = material.node_tree.nodes
    texture_node = nodes.new(type='ShaderNodeTexImage')

    image = bpy.data.images.load(texture_path)
    texture_node.image = image

    principled_bsdf = nodes.get("Principled BSDF")
    material.node_tree.links.new(texture_node.outputs["Color"], principled_bsdf.inputs["Base Color"])

    obj = bpy.context.active_object  
    if obj.data.materials:
        obj.data.materials[0] = material  # If the object already has a material, replace the first one
    else:
        obj.data.materials.append(material) 
  
def load_obj_texture_three(obj_path,obj_pos,obj_rot,texture_path,scale):
    #bpy.ops.mesh.primitive_monkey_add()

    # Object 2
    obj = bproc.loader.load_obj(obj_path)
    obj[0].set_location(obj_pos)
    obj[0].set_rotation_euler(obj_rot)
    obj[0].set_scale(scale)
    
    material = bpy.data.materials.new(name="TexturedMaterial")
    material.use_nodes = True  # Enable the use of nodes for the material

    nodes = material.node_tree.nodes
    texture_node = nodes.new(type='ShaderNodeTexImage')

    image = bpy.data.images.load(texture_path)
    texture_node.image = image

    principled_bsdf = nodes.get("Principled BSDF")
    material.node_tree.links.new(texture_node.outputs["Color"], principled_bsdf.inputs["Base Color"])

    obj = bpy.context.active_object  
    if obj.data.materials:
        obj.data.materials[0] = material  # If the object already has a material, replace the first one
    else:
        obj.data.materials.append(material) 

def load_obj_texture_four(obj_path,obj_pos,obj_rot,texture_path,scale):
    #bpy.ops.mesh.primitive_monkey_add()

    # Object 2
    obj = bproc.loader.load_obj(obj_path)
    obj[0].set_location(obj_pos)
    obj[0].set_rotation_euler(obj_rot)
    obj[0].set_scale(scale)
    
    material = bpy.data.materials.new(name="TexturedMaterial")
    material.use_nodes = True  # Enable the use of nodes for the material

    nodes = material.node_tree.nodes
    texture_node = nodes.new(type='ShaderNodeTexImage')

    image = bpy.data.images.load(texture_path)
    texture_node.image = image

    principled_bsdf = nodes.get("Principled BSDF")
    material.node_tree.links.new(texture_node.outputs["Color"], principled_bsdf.inputs["Base Color"])

    obj = bpy.context.active_object  
    if obj.data.materials:
        obj.data.materials[0] = material  # If the object already has a material, replace the first one
    else:
        obj.data.materials.append(material) 

# Lighting
def setup_lighting():
    light_data = bpy.data.lights.new('light',type='POINT')
    light = bpy.data.objects.new('light',light_data)
    bpy.context.collection.objects.link(light)
    light.location=[2,-2,2]
    light.data.energy = 300
    
# Move Camera and Capture Images
def move_camera_in_dome_using_lat_long(radius, min_lat, max_lat, step_lat, min_lon, max_lon, step_lon, output_folder="renders"):
    """
    Move the camera around the point [0, 0, 0] in a dome shape using longitude and latitude, capturing images at each frame.

    Args:
        radius (float): The distance from the camera to the center point (default 10).
        num_frames (int): The number of frames (images) to capture (default 36).
        latitude_range (tuple): The range of latitude angles (default (30, 90)).
        longitude_range (tuple): The range of longitude angles (default (0, 360)).
        output_folder (str): The folder where the images will be saved (default "renders").
    """
    # Create output folder if it doesn't exist
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Set up the camera (ensure there is a camera object in the scene)
    if "Camera" not in bpy.data.objects:
        bpy.ops.object.camera_add(location=(0, 0, 5))  # Add a camera at (0, 0, 10) if it doesn't exist
    camera = bpy.data.objects["Camera"]

    # Set up the scene to use the camera
    bpy.context.scene.camera = camera

    longitude_steps = range(min_lon, max_lon, step_lon)  
    latitude_steps = range(min_lat, max_lat, step_lat)  

    for longitude in longitude_steps:
        for latitude in latitude_steps:
            # Calculate camera position using spherical coordinates
            x = radius * math.sin(latitude*(math.pi/180)) * math.cos(longitude*(math.pi/180))
            y = radius * math.sin(latitude*(math.pi/180)) * math.sin(longitude*(math.pi/180))
            z = radius * math.cos(latitude*(math.pi/180))

            # Set camera position
            camera.location = (x, y, z)

            # camera orientation
            point_at_target(camera, (0, 0, 0))

            # Render the image
            output_path = os.path.join(output_folder, f"lat_{latitude}_lon_{longitude}.png")
            bpy.context.scene.render.filepath = output_path
            bpy.ops.render.render(write_still=True)

            '''
            # Render animation to video
            # Ensure output path exists
            os.makedirs(output_folder, exist_ok=True)

            # Set file path for video
            bpy.context.scene.render.filepath = os.path.join(output_folder, video_name)

            # Set render resolution and FPS
            bpy.context.scene.render.resolution_x = resolution[0]
            bpy.context.scene.render.resolution_y = resolution[1]
            bpy.context.scene.render.fps = fps

            # Set frame range
            bpy.context.scene.frame_start = start_frame
            bpy.context.scene.frame_end = end_frame

            # Set file format to video
            bpy.context.scene.render.image_settings.file_format = 'FFMPEG'
            bpy.context.scene.render.ffmpeg.format = 'MPEG4'           # Container: .mp4
            bpy.context.scene.render.ffmpeg.codec = 'H264'             # Codec: H.264
            bpy.context.scene.render.ffmpeg.constant_rate_factor = 'HIGH'
            bpy.context.scene.render.ffmpeg.ffmpeg_preset = 'GOOD'
            bpy.context.scene.render.image_settings.color_mode = 'RGB'
            bpy.ops.render.render(animation=True)
            '''


    print(f"Rendered images to {output_folder}")

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

def cross_product(v1, v2):
    """
    Compute the cross product of two 3D vectors.
    """
    return (v1[1] * v2[2] - v1[2] * v2[1],
            v1[2] * v2[0] - v1[0] * v2[2],
            v1[0] * v2[1] - v1[1] * v2[0])

def quat_to_euler(quat_xyzw):
    # Convert to mathutils Quaternion
    quat = mathutils.Quaternion(quat_xyzw)

    # Convert to Euler angles (in radians)
    euler = quat.to_euler('XYZ')  # You can use 'XYZ', 'ZXY', etc.

    return euler

def convert_pybullet_position_to_blender(pos):
    # PyBullet (x, y, z) --> Blender (x, z, -y) OR (x, -z, y)
    #return (pos[0], pos[2], -pos[1])
    return (pos[0], -pos[2], pos[1])

def convert_pybullet_quaternion_to_blender(quat):
    # Convert PyBullet XYZW quaternion to Blender, accounting for Y-up to Z-up
    quat_pb = mathutils.Quaternion(quat)  # PyBullet xyzw
    # Create a rotation to convert Y-up to Z-up
    rot_matrix = mathutils.Matrix.Rotation(-math.pi/2, 4, 'X')  # -90 degrees around X
    rot_blender = (rot_matrix.to_quaternion() @ quat_pb)
    return rot_blender.to_euler('XYZ')


video_name = "camera_sweep.mp4"
start_frame = 1
end_frame = 150
resolution = (1920, 1080)
fps = 30

# Initialize BlenderProc
bproc.init()

create_box(Xscale,Yscale,Zscale)

# Load saved poses
pose_data = np.load("pybullet_poses.npz", allow_pickle=True)
print("Objects in file:", list(pose_data.keys()))

#(x,y,z,rx,ry,rz) = random_position_within_open_box(box_location, box_scale, rx_range=(0, np.pi/2), ry_range=(0, np.pi/2), rz_range=(0, np.pi/2))
pose = pose_data["obj_0"].item()
obj_pos = (pose["location"])  #(location) #(x,y,z)
q = pose["quaternion_xyzw"]
obj_rot = quat_to_euler(q) #(rx,ry,rz)
#obj_pos = convert_pybullet_position_to_blender(pose["location"])
#rot = convert_pybullet_quaternion_to_blender(pose["quaternion_xyzw"])
#obj_rot = (rot.x,rot.y,rot.z)
#obj_pos = (0,0,0); 
#obj_rot = (0,0,0)
print(obj_pos); print(obj_rot)
load_obj_texture_one(obj_path,obj_pos,obj_rot,texture_path,scale)

#(x,y,z,rx,ry,rz) = random_position_within_open_box(box_location, box_scale, rx_range=(0, np.pi/2), ry_range=(0, np.pi/2), rz_range=(0, np.pi/2))
pose = pose_data["obj_1"].item()
obj_two_pos = (pose["location"][0]-0.3, pose["location"][1]-0.3, pose["location"][2]) #(pose["location"])  #(location) #(x,y,z)
q = pose["quaternion_xyzw"]
obj_two_rot = quat_to_euler(q)
#obj_two_pos = convert_pybullet_position_to_blender(pose["location"])
load_obj_texture_two(obj_two_path,obj_two_pos,obj_two_rot,texture_two_path,scale)

#(x,y,z,rx,ry,rz) = random_position_within_open_box(box_location, box_scale, rx_range=(0, np.pi/2), ry_range=(0, np.pi/2), rz_range=(0, np.pi/2))
pose = pose_data["obj_2"].item()
obj_three_pos = (pose["location"][0]-0.3, pose["location"][1]-0.5, pose["location"][2]) #(pose["location"])  #(location) #(x,y,z)
q = pose["quaternion_xyzw"]
obj_three_rot = quat_to_euler(q)
#obj_three_pos = convert_pybullet_position_to_blender(pose["location"])
load_obj_texture_three(obj_three_path,obj_three_pos,obj_three_rot,texture_three_path,scale)

#(x,y,z,rx,ry,rz) = random_position_within_open_box(box_location, box_scale, rx_range=(0, np.pi/2), ry_range=(0, np.pi/2), rz_range=(0, np.pi/2))
pose = pose_data["obj_3"].item()
obj_four_pos = (pose["location"][0]-0.5, pose["location"][1]+0.5, pose["location"][2]) #(pose["location"])  #(location) #(x,y,z)
q = pose["quaternion_xyzw"]
obj_four_rot = quat_to_euler(q)
#obj_four_pos = convert_pybullet_position_to_blender(pose["location"])
load_obj_texture_four(obj_four_path,obj_four_pos,obj_four_rot,texture_four_path,scale)


setup_lighting()
move_camera_in_dome_using_lat_long(radius, min_lat, max_lat, step_lat, min_lon, max_lon, step_lon, output_folder="renders")