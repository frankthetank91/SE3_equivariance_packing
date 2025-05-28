import blenderproc as bproc
import bpy
import numpy as np
import mathutils
from mathutils import Quaternion

obj_path = "/Users/mikaylalahr/Desktop/BlenderProc-main/my_code/textured.obj"
obj_pos = (0,0,0)
obj_rot = (0,0,0)
texture_path = "/Users/mikaylalahr/Desktop/BlenderProc-main/my_code/texture_map.jpg"

obj_two_path = "/Users/mikaylalahr/Desktop/BlenderProc-main/my_code/small_clamp/textured.obj"
obj_two_pos = (0,0,0)
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
Xscale = 0.34/2 #32 #1.3
Yscale = 0.34/2 #32 #1.5
Zscale = 0.3/2 #0.5
box_scale = [Xscale,Yscale,Zscale]
box_location = [0,0,Zscale]
scale = (1,1,1) #(3,3,3) # object scale

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
    inner_box.scale = (Xscale-0.001, Yscale-0.001, Zscale)

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
    light.location=[0.5,-0.5,2] #[2,-2,2]
    light.data.energy = 100 #300

def quat_to_euler(q_xyzw):
    w, x, y, z = mathutils.Quaternion((q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2])) #quat_wxyz
    quat = mathutils.Quaternion((w, x, y, z))  # Blender expects (w, x, y, z)
    return quat.to_euler('XYZ')

'''
def quat_to_euler(quat_xyzw):
    # Convert to mathutils Quaternion
    quat = mathutils.Quaternion(quat_xyzw)

    # Convert to Euler angles (in radians)
    euler = quat.to_euler('XYZ')  # You can use 'XYZ', 'ZXY', etc.

    return euler
'''
def convert_pybullet_position_to_blender(pos):
    # Swap Y and Z (PyBullet Y-up → Blender Z-up)
    return (pos[0], -pos[2], pos[1])

def convert_pybullet_quaternion_to_blender(quat_xyzw):
    # PyBullet: [x, y, z, w] (Y-up)
    # Blender: Z-up → convert to Euler after axis adjustment
    q_pb = mathutils.Quaternion((quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]))  # w, x, y, z
    # Rotate -90° around X to fix PyBullet Y-up to Blender Z-up
    rot_fix = mathutils.Euler((-np.pi / 2, 0, 0), 'XYZ').to_quaternion()
    q_fixed = rot_fix @ q_pb
    return q_fixed.to_euler('XYZ')


# Initialize BlenderProc
bproc.init()

#bpy.ops.preferences.addon_enable(module="io_scene_obj")
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)

create_box(Xscale,Yscale,Zscale)

# Load the pose data
pose_data = np.load("pybullet_poses.npz", allow_pickle=True)
print("Objects in npz:", list(pose_data.keys()))

# Load saved poses
pose_data = np.load("pybullet_poses.npz", allow_pickle=True)
print("Objects in file:", list(pose_data.keys()))

#(x,y,z,rx,ry,rz) = random_position_within_open_box(box_location, box_scale, rx_range=(0, np.pi/2), ry_range=(0, np.pi/2), rz_range=(0, np.pi/2))
pose = pose_data["obj_0"].item()
obj_pos = (pose["location"][0], pose["location"][1], pose["location"][2])  #(pose["location"])  #(location) #(x,y,z)
q = pose["quaternion_xyzw"]
obj_rot = quat_to_euler(q) #(rx,ry,rz)
#obj_pos = convert_pybullet_position_to_blender(pose["location"])
#rot = convert_pybullet_quaternion_to_blender(pose["quaternion_xyzw"])
#obj_rot = (rot.x,rot.y,rot.z)
#obj_pos = (0,0,0); 
#obj_rot = (0,0,0)
#obj_pos = convert_pybullet_position_to_blender(pose["location"])
#obj_rot = convert_pybullet_quaternion_to_blender(pose["quaternion_xyzw"])
print(obj_pos); print(obj_rot)
load_obj_texture_one(obj_path,obj_pos,obj_rot,texture_path,scale)

#(x,y,z,rx,ry,rz) = random_position_within_open_box(box_location, box_scale, rx_range=(0, np.pi/2), ry_range=(0, np.pi/2), rz_range=(0, np.pi/2))
pose = pose_data["obj_3"].item()
obj_two_pos = (pose["location"][0], pose["location"][1], pose["location"][2]) #(pose["location"][0], pose["location"][1], pose["location"][2]) #(pose["location"])  #(location) #(x,y,z)
q = pose["quaternion_xyzw"]
obj_two_rot = quat_to_euler(q)
#obj_two_pos = convert_pybullet_position_to_blender(pose["location"])
#obj_two_pos = convert_pybullet_position_to_blender(pose["location"])
#obj_two_rot = convert_pybullet_quaternion_to_blender(pose["quaternion_xyzw"])
load_obj_texture_two(obj_two_path,obj_two_pos,obj_two_rot,texture_two_path,scale)

#(x,y,z,rx,ry,rz) = random_position_within_open_box(box_location, box_scale, rx_range=(0, np.pi/2), ry_range=(0, np.pi/2), rz_range=(0, np.pi/2))
pose = pose_data["obj_2"].item()
obj_three_pos = (pose["location"][0], pose["location"][1], pose["location"][2]) #(pose["location"])  #(location) #(x,y,z)
q = pose["quaternion_xyzw"]
obj_three_rot = quat_to_euler(q)
#obj_three_pos = convert_pybullet_position_to_blender(pose["location"])
#obj_three_pos = convert_pybullet_position_to_blender(pose["location"])
#obj_three_rot = convert_pybullet_quaternion_to_blender(pose["quaternion_xyzw"])
load_obj_texture_three(obj_three_path,obj_three_pos,obj_three_rot,texture_three_path,scale)

#(x,y,z,rx,ry,rz) = random_position_within_open_box(box_location, box_scale, rx_range=(0, np.pi/2), ry_range=(0, np.pi/2), rz_range=(0, np.pi/2))
pose = pose_data["obj_1"].item()
obj_four_pos = (pose["location"][0], pose["location"][1], pose["location"][2]) #(pose["location"])  #(location) #(x,y,z)
q = pose["quaternion_xyzw"]
obj_four_rot = quat_to_euler(q)
#obj_four_pos = convert_pybullet_position_to_blender(pose["location"])
#obj_four_pos = convert_pybullet_position_to_blender(pose["location"])
#obj_four_rot = convert_pybullet_quaternion_to_blender(pose["quaternion_xyzw"])
load_obj_texture_four(obj_four_path,obj_four_pos,obj_four_rot,texture_four_path,scale)


#setup_lighting()

# Save the entire scene as a Blender .blend file
#bproc.utility.save_blend("my_scene.blend")
bpy.ops.wm.save_as_mainfile(filepath="my_scene.blend")
print("Saved my_scene.blend with all objects loaded.")
