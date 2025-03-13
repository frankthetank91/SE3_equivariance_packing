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

lbb_box = [0, 0, 0]  # Left-Bottom-Back corner
scale1 = 30
dimensions = [0.22*scale1, 0.3*scale1, 0.05*scale1]  # Dimensions [L, W, H]
scale2 = scale1 


Xscale = 1.3
Yscale = 1.5
Zscale = 0.5
box_scale = [Xscale,Yscale,Zscale]
box_location = [0,0,Zscale]

scale = (3,3,3)

# Function to create an open box using LBB corner and dimensions
def create_open_box(lbb_box, dimensions):
    # Unpack the dimensions
    L, W, H = dimensions
    x, y, z = lbb_box

    # Create the bottom plane (it will be a rectangle, L x W)
    bpy.ops.mesh.primitive_plane_add(size=1, location=(x, y, z))
    bottom = bpy.context.object
    bottom.scale = (L/2, W/2, H/2)
    bottom.name = 'Bottom'

    # Create 4 sides of the box
    # Front Side
    bpy.ops.mesh.primitive_cube_add(size=1, location=(x + L/4, y, z + H/4))
    front = bpy.context.object
    front.scale = (0.1, W/2, 1.5)
    front.name = 'Front'

    # Back Side
    bpy.ops.mesh.primitive_cube_add(size=1, location=(x - L/4, y, z + H/4))
    back = bpy.context.object
    back.scale = (0.1, W/2, 1.5)
    back.name = 'Back'

    # Left Side
    bpy.ops.mesh.primitive_cube_add(size=1, location=(x, y + W/4, z + H/4))
    left = bpy.context.object
    left.scale = (L/1.94, 0.1, 1.5)
    left.name = 'Left'

    # Right Side
    bpy.ops.mesh.primitive_cube_add(size=1, location=(x, y - W/4, z + H/4))
    right = bpy.context.object
    right.scale = (L/1.94, 0.1, 1.5)
    right.name = 'Right'

    # Return the created objects (box sides and bottom)
    return [bottom, front, back, left, right]

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


def load_obj_texture_one(obj_path,obj_pos,obj_rot,texture_path,scale):
    #bpy.ops.mesh.primitive_monkey_add()

    # Object 1
    obj = bproc.loader.load_obj(obj_path)
    obj[0].set_location(obj_pos)
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


def random_position_within_box(lbb_box, dimensions, rx_range=(0, np.pi/2), ry_range=(0, np.pi/2), rz_range=(0, np.pi/2)):
    """
    Generate a random position and rotation within the confines of an open box (no top).

    Args:
        lbb_box (tuple): The coordinates of the lower-back-bottom corner (x, y, z).
        dimensions (tuple): The dimensions of the box (L, W, H).
        rx_range (tuple): Range for random rotation around the X-axis.
        ry_range (tuple): Range for random rotation around the Y-axis.
        rz_range (tuple): Range for random rotation around the Z-axis.

    Returns:
        tuple: (x, y, z, rx, ry, rz) random position and rotation.
    """
    # Unpack the center and dimensions
    x_center, y_center, z_center = lbb_box
    L, W, H = dimensions

    # Generate random position within the box (centered at the given center)
    x = random.uniform(x_center - L / 4, x_center + L / 4)
    y = random.uniform(y_center - W / 4, y_center + W / 4)
    z = random.uniform(z_center - H / 4, z_center + H / 4)

    # Generate random rotations within the specified ranges
    rx = random.uniform(*rx_range)
    ry = random.uniform(*ry_range)
    rz = random.uniform(*rz_range)

    return (x, y, z, rx, ry, rz)

def random_position(x_range=(-0.5, 0.5), y_range=(-0.5, 0.5), z_range=(0, 0.5),rx_range=(0,np.pi/2), ry_range=(0,np.pi/2), rz_range=(0,np.pi/2)):
    x = random.uniform(*x_range)
    y = random.uniform(*y_range)
    z = random.uniform(*z_range)
    rx = random.uniform(*rx_range)
    ry = random.uniform(*ry_range)
    rz = random.uniform(*rz_range)
    return (x,y,z,rx,ry,rz)
    
def setup_lighting():
    light_data = bpy.data.lights.new('light',type='POINT')
    light = bpy.data.objects.new('light',light_data)
    bpy.context.collection.objects.link(light)
    light.location=[2,-2,2]
    light.data.energy = 300
    
def cam_setup():
    cam_data = bpy.data.cameras.new('camera')
    cam = bpy.data.objects.new('camera', cam_data)
    bpy.context.collection.objects.link(cam)
    cam.location = [0,0,5]
    cam.rotation_euler = [0, 0, 0]

    # Register camera pose in BlenderProc
    cam_pose = bproc.math.build_transformation_mat(cam.location, cam.rotation_euler)
    bproc.camera.add_camera_pose(cam_pose)

def capture_images():
    bproc.renderer.enable_depth_output(activate_antialiasing=False)
    bproc.renderer.enable_normals_output()

    rgbd_data = bproc.renderer.render()

    # Write the rendering into a hdf5 file
    bproc.writer.write_hdf5("output/", rgbd_data)

    # Extract the color, depth, and normals images
    rgb_image = np.array(rgbd_data['colors'][0])  # RGB image
    depth_image = np.array(rgbd_data['depth'][0])  # Depth map
    normal_image = np.array(rgbd_data['normals'][0])  # Normal map

    # Handle NaN and inf values in depth image before converting to uint16
    #depth_image = np.nan_to_num(depth_image, nan=0.0, posinf=65535, neginf=0)

    # Normalize normal image to [0, 255] for saving as PNG
    normal_image_normalized = ((normal_image + 1) * 127.5).astype(np.uint8)

    # Save the RGB image as PNG
    rgb_image_pil = Image.fromarray(rgb_image)
    rgb_image_pil.save(os.path.join("output/rgb/", "0.png"))

    # Save the depth image as PNG
    depth_image_pil = Image.fromarray(depth_image)  # Depth is usually in 16-bit
    depth_image_pil.save(os.path.join("output/depth/", "0.tiff"))
    imageio.imwrite("output/depth_image.tiff", depth_image)

    # Save the normal map as PNG
    normal_image_pil = Image.fromarray(normal_image_normalized)
    normal_image_pil.save(os.path.join("output/normal/", "0.png"))



def capture_images_around_box(box_location, box_scale, num_frames=36, radius=10, output_folder="renders"):
    """
    Capture images around the box by rotating the camera and rendering images at each angle.

    Args:
        box_location (tuple): The center coordinates of the box (x, y, z).
        box_scale (tuple): The scale of the box (L, W, H).
        num_frames (int): The number of frames (images) to capture (default 36).
        radius (float): The distance from the camera to the center of the box (default 10).
        output_folder (str): The folder where the images will be saved (default "renders").
    """
    # Create output folder if it doesn't exist
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Set up the camera (ensure there is a camera object in the scene)
    if "Camera" not in bpy.data.objects:
        bpy.ops.object.camera_add(location=(0, 0, 5))
    camera = bpy.data.objects["Camera"]
    camera.rotation_euler = (0, 0, 0)  # Camera facing down (adjust as needed)

    # Set up the scene to use the camera
    bpy.context.scene.camera = camera

    # Loop through angles and capture images
    for i in range(num_frames):
        angle = (i / num_frames) * 2 * math.pi  # Calculate angle for this frame

        # Calculate new camera position based on angle
        x = box_location[0] + radius * math.cos(angle)
        y = box_location[1] + radius * math.sin(angle)
        z = box_location[2] + box_scale[2] / 2  # Camera should be at the height of the box

        # Set camera position
        camera.location = (x, y, z)

        # Point the camera towards the center of the box (look at box_location)
        direction = (box_location[0] - x, box_location[1] - y, box_location[2] - z)
        camera.rotation_euler = direction_to_rotation(direction)

        # Render the image
        output_path = os.path.join(output_folder, f"frame_{i:03d}.png")
        bpy.context.scene.render.filepath = output_path
        bpy.ops.render.render(write_still=True)

    print(f"Rendered {num_frames} images to {output_folder}")


def move_camera_in_dome(radius, num_frames, height, dome_angle_range, output_folder="renders"):
    """
    Move the camera around the point [0, 0, 0] in a dome shape and capture images at each frame.

    Args:
        radius (float): The distance from the camera to the center point (default 10).
        num_frames (int): The number of frames (images) to capture (default 36).
        height (float): The height of the dome from which the camera starts (default 5).
        dome_angle_range (tuple): The range of angles for the dome (default (0, pi/2)).
        output_folder (str): The folder where the images will be saved (default "renders").
    """
    # Create output folder if it doesn't exist
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Set up the camera (ensure there is a camera object in the scene)
    if "Camera" not in bpy.data.objects:
        bpy.ops.object.camera_add(location=(0, 0, 5))
    camera = bpy.data.objects["Camera"]
    camera.rotation_euler = (0, 0, 0)  # Camera facing down (adjust as needed)

    # Set up the scene to use the camera
    bpy.context.scene.camera = camera

    # Loop through angles and capture images
    for i in range(num_frames):
        # Calculate the dome angle (vertical)
        dome_angle = dome_angle_range[0] + (i / (num_frames - 1)) * (dome_angle_range[1] - dome_angle_range[0])
        
        # Calculate the horizontal angle (around the center)
        horizontal_angle = (i / num_frames) * 2 * math.pi  # Full circle (2*pi)

        # Calculate camera position
        x = radius * math.cos(horizontal_angle) * math.sin(dome_angle)  # x is a combination of horizontal and dome angles
        y = radius * math.sin(horizontal_angle) * math.sin(dome_angle)  # y is a combination of horizontal and dome angles
        z = height + radius * math.cos(dome_angle)  # z is based on dome angle (height is offset)

        # Set camera position
        camera.location = (x, y, z)

        # Point the camera towards the center [0, 0, 0]
        direction = (0 - x, 0 - y, 0 - z)
        length = math.sqrt(direction[0]**2 + direction[1]**2 + direction[2]**2)
        direction = (direction[0] / length, direction[1] / length, direction[2] / length)
        pitch = math.asin(-direction[2])  # Rotate around X (Up/Down)
        yaw = math.atan2(direction[1], direction[0])  # Rotate around Z (Left/Right)
        camera.rotation_euler = (pitch, 0, yaw)
        #camera.rotation_euler = direction_to_rotation(direction)

        cam_pose = bproc.math.build_transformation_mat(camera.location, camera.rotation_euler)
        bproc.camera.add_camera_pose(cam_pose)

        print(cam_pose)

        # Render the image
        output_path = os.path.join(output_folder, f"frame_{i:03d}.png")
        rgbd_data = bproc.renderer.render()
        rgb_image = np.array(rgbd_data['colors'][0])  # RGB image
        rgb_image_pil = Image.fromarray(rgb_image)
        rgb_image_pil.save(os.path.join(output_path))
        #bpy.context.scene.render.filepath = output_path
        #bpy.ops.render.render(write_still=True)

    print(f"Rendered {num_frames} images to {output_folder}")


def move_camera_in_dome_using_lat_long(radius, num_frames, min_lat, max_lat, step_lat, min_lon, max_lon, step_lon, output_folder="renders"):
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

    longitude_steps = range(min_lon, max_lon, step_lon)  # 0 to 360 degrees, step of 10 degrees (longitude)
    latitude_steps = range(min_lat, max_lat, step_lat)  # 0 to 90 degrees, step of 10 degrees (latitude)

    # Store results
    empty_results = []

    for longitude in longitude_steps:
        for latitude in latitude_steps:

    # Loop through frames and capture images
    #for i in range(num_frames):
        # Calculate the longitude and latitude for this frame
        #longitude = math.radians(longitude_range[0] + (i / (num_frames - 1)) * (longitude_range[1] - longitude_range[0]))  # Convert to radians
        #latitude = math.radians(latitude_range[0] + (i / (num_frames - 1)) * (latitude_range[1] - latitude_range[0]))  # Convert to radians

            # Calculate camera position using spherical coordinates
            x = radius * math.sin(latitude*(math.pi/180)) * math.cos(longitude*(math.pi/180))
            y = radius * math.sin(latitude*(math.pi/180)) * math.sin(longitude*(math.pi/180))
            z = radius * math.cos(latitude*(math.pi/180))

            # Set camera position
            camera.location = (x, y, z)

            # Point the camera towards the center [0, 0, 0]
            #direction = (0 - x, 0 - y, 0 - z)
            #camera.rotation_euler = direction_to_rotation(direction)

            point_at_target(camera, (0, 0, 0))

            # Render the image
            output_path = os.path.join(output_folder, f"lat_{latitude*(180/math.pi)}_lon_{longitude*(180/math.pi)}.png")
            bpy.context.scene.render.filepath = output_path
            bpy.ops.render.render(write_still=True)

    print(f"Rendered {num_frames} images to {output_folder}")

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

    # Create a rotation matrix that makes the camera look at the target
    # We use the vector pointing from the camera to the target to determine the rotation
    #rotation_matrix = direction_to_rotation_matrix(direction)

    # Apply the matrix to the camera
    #camera.rotation_euler = rotation_matrix.to_euler()

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


def direction_to_rotation_matrix(direction):
    """
    Convert a direction vector into a rotation matrix that makes the camera point towards the target.
    """
    # Normalize the direction
    length = math.sqrt(direction[0]**2 + direction[1]**2 + direction[2]**2)
    direction = (direction[0] / length, direction[1] / length, direction[2] / length)

    # Get the forward vector (camera looks along -Z axis)
    forward = (0, 0, -1)
    up = (0, 1, 0)  # World "up" vector (Y axis)

    # Create the rotation matrix
    right = cross_product(up, forward)
    up = cross_product(forward, right)

    # Build the matrix
    rotation_matrix = mathutils.Matrix((
        (right[0], right[1], right[2], 0),
        (up[0], up[1], up[2], 0),
        (-direction[0], -direction[1], -direction[2], 0),
        (0, 0, 0, 1)
    ))

    return rotation_matrix

def cross_product(v1, v2):
    """
    Compute the cross product of two 3D vectors.
    """
    return (v1[1] * v2[2] - v1[2] * v2[1],
            v1[2] * v2[0] - v1[0] * v2[2],
            v1[0] * v2[1] - v1[1] * v2[0])

def direction_to_rotation(direction):
    """
    Convert a direction vector into a rotation (Euler angles) for the camera.
    """
    # Normalize direction
    length = math.sqrt(direction[0]**2 + direction[1]**2 + direction[2]**2)
    direction = (direction[0] / length, direction[1] / length, direction[2] / length)

    # Calculate the pitch (rotation around x-axis) and yaw (rotation around z-axis)
    pitch = math.asin(-direction[2])  # Rotate around X (Up/Down)
    yaw = math.atan2(direction[1], direction[0])  # Rotate around Z (Left/Right)

    return (pitch, 0, yaw)



# Initialize BlenderProc
bproc.init()

create_box(Xscale,Yscale,Zscale)
(x,y,z,rx,ry,rz) = random_position_within_open_box(box_location, box_scale, rx_range=(0, np.pi/2), ry_range=(0, np.pi/2), rz_range=(0, np.pi/2))
obj_pos = (x,y,z)
obj_rot = (rx,ry,rz)
print(obj_pos)
load_obj_texture_one(obj_path,obj_pos,obj_rot,texture_path,scale)

(x,y,z,rx,ry,rz) = random_position_within_open_box(box_location, box_scale, rx_range=(0, np.pi/2), ry_range=(0, np.pi/2), rz_range=(0, np.pi/2))
obj_two_pos = (x,y,z)
obj_two_rot = (rx,ry,rz)
load_obj_texture_two(obj_two_path,obj_two_pos,obj_two_rot,texture_two_path,scale)

(x,y,z,rx,ry,rz) = random_position_within_open_box(box_location, box_scale, rx_range=(0, np.pi/2), ry_range=(0, np.pi/2), rz_range=(0, np.pi/2))
obj_three_pos = (x,y,z)
obj_three_rot = (rx,ry,rz)
load_obj_texture_three(obj_three_path,obj_three_pos,obj_three_rot,texture_three_path,scale)

(x,y,z,rx,ry,rz) = random_position_within_open_box(box_location, box_scale, rx_range=(0, np.pi/2), ry_range=(0, np.pi/2), rz_range=(0, np.pi/2))
obj_four_pos = (x,y,z)
obj_four_rot = (rx,ry,rz)
load_obj_texture_four(obj_four_path,obj_four_pos,obj_four_rot,texture_four_path,scale)


setup_lighting()
move_camera_in_dome_using_lat_long(radius=6, num_frames=5, min_lat=0, max_lat=70, step_lat=30, min_lon=0, max_lon=360, step_lon=60, output_folder="renders")
#move_camera_in_dome(radius=1, num_frames=2, height=5, dome_angle_range=(0, math.pi / 2), output_folder="renders")
#capture_images_around_box(box_location, box_scale, num_frames=36, radius=15, output_folder="renders")


'''
create_open_box(lbb_box,dimensions)

(x,y,z,rx,ry,rz) = random_position_within_box(lbb_box, dimensions)
obj_pos = (x,y,z)
obj_rot = (rx,ry,rz)
print(obj_pos)
load_obj_texture_one(obj_path,obj_pos,obj_rot,texture_path)

(x,y,z,rx,ry,rz) = random_position_within_box(lbb_box, dimensions)
obj_two_pos = (x,y,z)
obj_two_rot = (rx,ry,rz)
load_obj_texture_two(obj_two_path,obj_two_pos,obj_two_rot,texture_two_path)

(x,y,z,rx,ry,rz) = random_position_within_box(lbb_box, dimensions)
obj_three_pos = (x,y,z)
obj_three_rot = (rx,ry,rz)
load_obj_texture_three(obj_three_path,obj_three_pos,obj_three_rot,texture_three_path)

(x,y,z,rx,ry,rz) = random_position_within_box(lbb_box, dimensions)
obj_four_pos = (x,y,z)
obj_four_rot = (rx,ry,rz)
load_obj_texture_four(obj_four_path,obj_four_pos,obj_four_rot,texture_four_path)


cam_setup()
setup_lighting()
capture_images()

'''

'''

bproc.init()

(x,y,z,rx,ry,rz) = random_position()
obj_pos = (x,y,z)
obj_rot = (rx,ry,rz)
print(obj_pos)
load_obj_texture_one(obj_path,obj_pos,obj_rot,texture_path)

(x,y,z,rx,ry,rz) = random_position()
obj_two_pos = (x,y,z)
obj_two_rot = (rx,ry,rz)
load_obj_texture_two(obj_two_path,obj_two_pos,obj_two_rot,texture_two_path)

(x,y,z,rx,ry,rz) = random_position()
obj_three_pos = (x,y,z)
obj_three_rot = (rx,ry,rz)
load_obj_texture_three(obj_three_path,obj_three_pos,obj_three_rot,texture_three_path)

(x,y,z,rx,ry,rz) = random_position()
obj_four_pos = (x,y,z)
obj_four_rot = (rx,ry,rz)
load_obj_texture_four(obj_four_path,obj_four_pos,obj_four_rot,texture_four_path)

setup_lighting()
cam_setup()
capture_images()
'''

'''
bproc.renderer.enable_depth_output(activate_antialiasing=False)
bproc.renderer.enable_normals_output()

rgbd_data = bproc.renderer.render()

# Write the rendering into a hdf5 file
bproc.writer.write_hdf5("output/", rgbd_data)

# Extract the color, depth, and normals images
rgb_image = np.array(rgbd_data['colors'][0])  # RGB image
depth_image = np.array(rgbd_data['depth'][0])  # Depth map
normal_image = np.array(rgbd_data['normals'][0])  # Normal map

# Handle NaN and inf values in depth image before converting to uint16
#depth_image = np.nan_to_num(depth_image, nan=0.0, posinf=65535, neginf=0)

# Normalize normal image to [0, 255] for saving as PNG
normal_image_normalized = ((normal_image + 1) * 127.5).astype(np.uint8)

# Save the RGB image as PNG
rgb_image_pil = Image.fromarray(rgb_image)
rgb_image_pil.save(os.path.join("output/rgb/", "0.png"))

# Save the depth image as PNG
depth_image_pil = Image.fromarray(depth_image)  # Depth is usually in 16-bit
depth_image_pil.save(os.path.join("output/depth/", "0.tiff"))

# Save the normal map as PNG
normal_image_pil = Image.fromarray(normal_image_normalized)
normal_image_pil.save(os.path.join("output/normal/", "0.png"))
'''