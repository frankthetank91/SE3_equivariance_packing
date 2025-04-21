import pybullet as p
import pybullet_data
from typing import List, Tuple
import numpy as np
import random
import time
import os
import json


class ObjectManager:
    """Class to handle the creation of the box and loading objects into the simulation."""

    def __init__(self, lbb_box: List[float], dimensions: List[float]):
        """
        Initialize the ObjectManager.

        Args:
            lbb_box (List[float]): Left-Bottom-Back corner of the box.
            dimensions (List[float]): Dimensions of the box [L, W, H].
        """
        self.lbb_box = lbb_box
        self.dimensions = dimensions
        self.box_id = None
        self.object_ids = []

    def setup_box(self):
        """Create a box (open top) in the simulation."""
        color = [0.5, 0.35, 0.05, 1]  # Brown color
        thickness = 0.001

        # Collision shapes
        bottom_id = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=[d / 2 for d in [self.dimensions[0], self.dimensions[1], thickness]]
        )
        yz_id = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=[thickness / 2, self.dimensions[1] / 2, self.dimensions[2] / 2]
        )
        xz_id = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=[self.dimensions[0] / 2, thickness / 2, self.dimensions[2] / 2]
        )
        yz_offset_id = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[thickness / 2, self.dimensions[1] / 2, self.dimensions[2] / 2],
            collisionFramePosition=[self.dimensions[0] / 2, 0, 0],
        )
        xz_offset_id = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[self.dimensions[0] / 2, thickness / 2, self.dimensions[2] / 2],
            collisionFramePosition=[0, self.dimensions[1] / 2, 0],
        )

        # Positions
        translate_bottom = [
            self.lbb_box[0] + self.dimensions[0] / 2,
            self.lbb_box[1] + self.dimensions[1] / 2,
            self.lbb_box[2] + thickness / 2,
        ]
        translate_yz = [
            self.lbb_box[0],
            self.lbb_box[1] + self.dimensions[1] / 2,
            self.lbb_box[2] + self.dimensions[2] / 2,
        ]
        translate_xz = [
            self.lbb_box[0] + self.dimensions[0] / 2,
            self.lbb_box[1],
            self.lbb_box[2] + self.dimensions[2] / 2,
        ]
        translate_yz_offset = [
            self.lbb_box[0] + self.dimensions[0] / 2,
            self.lbb_box[1] + self.dimensions[1] / 2,
            self.lbb_box[2] + self.dimensions[2] / 2,
        ]
        translate_xz_offset = [
            self.lbb_box[0] + self.dimensions[0] / 2,
            self.lbb_box[1] + self.dimensions[1] / 2,
            self.lbb_box[2] + self.dimensions[2] / 2,
        ]

        # Create box parts
        bottom = p.createMultiBody(
            baseMass=-1, baseCollisionShapeIndex=bottom_id, basePosition=translate_bottom
        )
        yz = p.createMultiBody(
            baseMass=-1, baseCollisionShapeIndex=yz_id, basePosition=translate_yz
        )
        xz = p.createMultiBody(
            baseMass=-1, baseCollisionShapeIndex=xz_id, basePosition=translate_xz
        )
        yz_offset = p.createMultiBody(
            baseMass=-1, baseCollisionShapeIndex=yz_offset_id, basePosition=translate_yz_offset
        )
        xz_offset = p.createMultiBody(
            baseMass=-1, baseCollisionShapeIndex=xz_offset_id, basePosition=translate_xz_offset
        )

        # Set visual properties
        for part in [bottom, yz, xz, yz_offset, xz_offset]:
            p.changeVisualShape(part, -1, rgbaColor=color)

    def load_objects(self, N: int = 5):
        """
        Load random objects (cubes) into the box.

        Args:
            N (int, optional): Number of objects to load. Defaults to 5.
        """
        N = 1
        for _ in range(N):
            lbb = np.array(self.lbb_box) + np.array([0.03, 0.03, 0.1])
            max_range = np.array(self.lbb_box) + np.array(self.dimensions) - np.array([0.03, 0.03, 0.1])
            '''
            position = np.array([0.21, 0, 0.03]) #np.random.uniform(lbb, max_range)
            cube = p.loadURDF("cube_small.urdf", position.tolist())
            color = np.array([1, 0, 0, 1]) #np.random.rand(4)
            #color[3]=1
            p.changeVisualShape(cube, -1, rgbaColor=color.tolist())
            self.object_ids.append(cube)
            '''
            '''
            position = np.array([0.28, 0.1, 0.03]) #np.random.uniform(lbb, max_range)
            cube = p.loadURDF("cube_small.urdf", position.tolist())
            color = np.array([1, 0, 0, 1]) #np.random.rand(4)
            #color[3]=1
            p.changeVisualShape(cube, -1, rgbaColor=color.tolist())
            self.object_ids.append(cube)
            
            position = np.array([0.28, -0.1, 0.03]) #np.random.uniform(lbb, max_range)
            cube = p.loadURDF("cube_small.urdf", position.tolist())
            color = np.array([1, 0, 0, 1]) #np.random.rand(4)
            #color[3]=1
            p.changeVisualShape(cube, -1, rgbaColor=color.tolist())
            self.object_ids.append(cube)
            
            position = np.array([0.14, 0.1, 0.03]) #np.random.uniform(lbb, max_range)
            cube = p.loadURDF("cube_small.urdf", position.tolist())
            color = np.array([1, 0, 0, 1]) #np.random.rand(4)
            #color[3]=1
            p.changeVisualShape(cube, -1, rgbaColor=color.tolist())
            self.object_ids.append(cube)
            
            position = np.array([0.14, -0.1, 0.03]) #np.random.uniform(lbb, max_range)
            cube = p.loadURDF("cube_small.urdf", position.tolist())
            color = np.array([1, 0, 0, 1]) #np.random.rand(4)
            #color[3]=1
            p.changeVisualShape(cube, -1, rgbaColor=color.tolist())
            self.object_ids.append(cube)
            '''


# Connect to physics server
p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load a plane
plane_id = p.loadURDF("plane.urdf")

# Create a simple box for containment (walls)
def create_walls():
    wall_thickness = 0.1
    wall_height = 1.0
    box_size = 1.0

    wall_urdf = "cube.urdf"
    wall_ids = []

    wall_positions = [
        (box_size, 0, wall_height / 2),
        (-box_size, 0, wall_height / 2),
        (0, box_size, wall_height / 2),
        (0, -box_size, wall_height / 2),
    ]

    for pos in wall_positions:
        wall_id = p.loadURDF(
            wall_urdf,
            basePosition=pos,
            globalScaling=2,
            useFixedBase=True
        )
        wall_ids.append(wall_id)

#create_walls()

# Setup Box and Load Objects
lbb_box = [0.1, -0.15, 0]  # Left-Bottom-Back corner
dimensions = [0.44,0.6,0.2] #[0.22, 0.3, 0.1]  # Dimensions [L, W, H]
object_manager = ObjectManager(lbb_box, dimensions)
object_manager.setup_box()
#object_manager.load_objects(5)
box_center = np.array(lbb_box) + np.array(dimensions) / 2

'''
# Load objects randomly
object_paths = ["cube_small.urdf", "sphere_small.urdf", "duck_vhacd.urdf"]
loaded_objects = []

for i in range(5):  # number of objects
    obj_path = random.choice(object_paths)
    pos = [random.uniform(box_center[0]-0.2, box_center[0]+0.2), random.uniform(box_center[1]-0.2, box_center[1]+0.2), random.uniform(0.25, 0.75)]
    orn = p.getQuaternionFromEuler([random.uniform(0, 3.14) for _ in range(3)])
    obj_id = p.loadURDF(obj_path, basePosition=pos, baseOrientation=orn)
    loaded_objects.append(obj_id)
'''


# Load random URDF objects
YCB_PATH = "/path/to/urdf-ycb/ycb_models"
object_paths = [
    os.path.join("power_drill/model.urdf"),
    os.path.join("scissors/model.urdf"),
    os.path.join("shampoo/model.urdf"),
    os.path.join("small_clamp/model.urdf"),
]

object_ids = []
object_names = []

for i in range(len(object_paths)):
    urdf_path = object_paths[i]
    obj_id = p.loadURDF(
        urdf_path,
        basePosition=[random.uniform(box_center[0]-0.2, box_center[0]+0.2), random.uniform(box_center[1]-0.2, box_center[1]+0.2), random.uniform(0.25, 0.75)],
        baseOrientation=p.getQuaternionFromEuler([random.uniform(0, 3.14) for _ in range(3)])
    )
    object_ids.append(obj_id)
    object_names.append(f"obj_{i}")

# Step simulation
for _ in range(500):  
    p.stepSimulation()
    time.sleep(1. / 60.)

# Save object poses
poses = {}
for name, obj_id in zip(object_names, object_ids):
    pos, orn = p.getBasePositionAndOrientation(obj_id)
    poses[name] = {
        "location": np.array(pos),
        "quaternion_xyzw": np.array([orn[1], orn[2], orn[3], orn[0]])  # BlenderProc wants xyzw
    }

# Save as npz
np.savez("pybullet_poses.npz", **poses)

p.disconnect()
