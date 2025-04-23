import pybullet as p
import pybullet_data
import time
import numpy as np
import zmq
import struct
import math
from typing import List, Tuple
import cv2
from matplotlib.colors import LinearSegmentedColormap

# Configuration Parameters
URDF_PATH = "/Users/mikaylalahr/Desktop/RobotPerceptionFinal/mycobot_280_m5/mycobot.urdf"
CAMERA_LINK_NAME = "camera_flange"
END_EFFECTOR_LINK_NAME = "pump_head"
TRAJECTORY_SAVING_PATH = "/Users/mikaylalahr/Desktop/RobotPerceptionFinal/IK_solution.npy"
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
FX, FY, CX, CY = 554.26, 554.26, 320, 240  # Camera Intrinsics
CAMERA_FOV = 60
CAMERA_NEAR = 0.02
CAMERA_FAR = 1.
STREAM_PORT = 5555  # ZeroMQ Streaming Port
NUM_OBJECTS = 5  # Number of objects to load into the box
STREAM_FPS = 240  # Streaming Frames Per Second
SIMULATION_STEP_DELAY = 1.0 / 240  # Simulation Step Delay

min_lat = 30; max_lat = 90; step_lat = 10
min_lon = 0; max_lon = 360; step_lon = 30
distance = 0.2

lbb_box = [0.1, -0.15, 0]  # Left-Bottom-Back corner
dimensions = [0.22, 0.3, 0.1]  # Dimensions [L, W, H]

#### INPUTS ####
box_center = np.array(lbb_box) + np.array(dimensions) / 2
rgbd_cam_pos = box_center.copy()
rgbd_cam_pos[2] = 0.5  # Adjust camera height
rgbd_cam_target = rgbd_cam_pos - np.array([0, 0, 0.5])

height = 0.4
radius = 0.2
Nposes = 8


def rotm_to_quat(matrix: np.ndarray) -> np.ndarray:
    """
    Convert a 3x3 rotation matrix to a quaternion.

    Args:
        matrix (np.ndarray): 3x3 rotation matrix.

    Returns:
        np.ndarray: Quaternion [x, y, z, w].
    """
    if matrix.shape != (3, 3):
        raise ValueError("Input matrix must be 3x3.")

    trace = np.trace(matrix)
    if trace > 0:
        S = np.sqrt(trace + 1.0) * 2
        w = 0.25 * S
        x = (matrix[2, 1] - matrix[1, 2]) / S
        y = (matrix[0, 2] - matrix[2, 0]) / S
        z = (matrix[1, 0] - matrix[0, 1]) / S
    elif (matrix[0, 0] > matrix[1, 1]) and (matrix[0, 0] > matrix[2, 2]):
        S = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2
        w = (matrix[2, 1] - matrix[1, 2]) / S
        x = 0.25 * S
        y = (matrix[0, 1] + matrix[1, 0]) / S
        z = (matrix[0, 2] + matrix[2, 0]) / S
    elif matrix[1, 1] > matrix[2, 2]:
        S = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2
        w = (matrix[0, 2] - matrix[2, 0]) / S
        x = (matrix[0, 1] + matrix[1, 0]) / S
        y = 0.25 * S
        z = (matrix[1, 2] + matrix[2, 1]) / S
    else:
        S = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2
        w = (matrix[1, 0] - matrix[0, 1]) / S
        x = (matrix[0, 2] + matrix[2, 0]) / S
        y = (matrix[1, 2] + matrix[2, 1]) / S
        z = 0.25 * S

    return np.array([x, y, z, w])

class PyBulletSimulator:
    """Class to handle PyBullet simulation setup and object management."""

    def __init__(self, urdf_path: str):
        self.urdf_path = urdf_path
        self.robot_id = None
        self.plane_id = None

    def connect(self, gui: bool = True):
        """Connect to the PyBullet physics server."""
        connection_mode = p.GUI if gui else p.DIRECT
        self.client = p.connect(connection_mode)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)

    def load_plane(self):
        """Load a plane into the simulation."""
        self.plane_id = p.loadURDF("plane.urdf")
        # set plane color as gray
        p.changeVisualShape(self.plane_id, -1, rgbaColor=[0.7, 0.7, 0.7, 1])

    def load_robot(self, start_pos: List[float], start_orientation: List[float]):
        """Load the robot URDF into the simulation."""
        self.robot_id = p.loadURDF(
            self.urdf_path, start_pos, start_orientation, useFixedBase=True
        )

    def disconnect(self):
        """Disconnect from the PyBullet physics server."""
        p.disconnect()

    @staticmethod
    def get_link_index(robot_id: int, link_name: str) -> int:
        """
        Retrieve the link index for a given link name.

        Args:
            robot_id (int): The robot's unique ID.
            link_name (str): The name of the link.

        Returns:
            int: The link index.

        Raises:
            ValueError: If the link name is not found.
        """
        num_joints = p.getNumJoints(robot_id)
        for i in range(num_joints):
            info = p.getJointInfo(robot_id, i)
            if info[12].decode("utf-8") == link_name:
                return i
        raise ValueError(f"Link name '{link_name}' not found in the robot URDF.")

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
            
            position = np.array([0.21, 0, 0.03]) #np.random.uniform(lbb, max_range)
            cube = p.loadURDF("cube_small.urdf", position.tolist())
            color = np.array([1, 0, 0, 1]) #np.random.rand(4)
            #color[3]=1
            p.changeVisualShape(cube, -1, rgbaColor=color.tolist())
            self.object_ids.append(cube)
            
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

class RobotController:
    """Class to handle robot kinematics and control."""

    def __init__(self, robot_id: int, ee_link_name: str, cam_link_name: str):
        """
        Initialize the RobotController.

        Args:
            robot_id (int): The robot's unique ID.
            ee_link_name (str): End-effector link name.
            cam_link_name (str): Camera link name.
        """
        self.robot_id = robot_id
        self.ee_link_name = ee_link_name
        self.cam_link_name = cam_link_name
        self.ee_index = PyBulletSimulator.get_link_index(robot_id, ee_link_name)
        self.cam_index = PyBulletSimulator.get_link_index(robot_id, cam_link_name)
        self.ik_solutions = []

    def compute_inverse_kinematics(
        self, target_pos: List[float], target_orn: List[float], lower_limits: List[float],
        upper_limits: List[float], rest_poses: List[float], solver: int = 0
    ) -> List[float]:
        """
        Compute the inverse kinematics for the robot.

        Args:
            target_pos (List[float]): Target position [x, y, z].
            target_orn (List[float]): Target orientation as a quaternion [x, y, z, w].
            lower_limits (List[float]): Lower joint limits.
            upper_limits (List[float]): Upper joint limits.
            rest_poses (List[float]): Rest poses for the joints.
            solver (int, optional): IK solver type. Defaults to 0.

        Returns:
            List[float]: Joint positions.
        """
        ik_solution = p.calculateInverseKinematics(
            self.robot_id,
            endEffectorLinkIndex=self.ee_index,
            targetPosition=target_pos,
            targetOrientation=target_orn,
            lowerLimits=lower_limits,
            upperLimits=upper_limits,
            restPoses=rest_poses,
            solver=solver,
        )
        self.ik_solutions.append(ik_solution)
        return ik_solution

    def apply_joint_positions(self, joint_positions: List[float], num_control_joints: int = 6):
        """
        Apply joint positions to the robot.

        Args:
            joint_positions (List[float]): Joint positions.
            num_control_joints (int, optional): Number of joints to control. Defaults to 6.
        """
        for i in range(num_control_joints):
            p.resetJointState(self.robot_id, i + 1, joint_positions[i])

    #### SAVE DATA TO TEXT FILE ############################################################
    def save_text_file(self,image_index,target_pos,target_orn,actual_pos,actual_orn,per_error_pos,quat_dif,pos_error,orn_error,ik_solution,joint_states):
        with open(f"Data{image_index}", "w") as file:
            file.write(f"image_index: {image_index}\n")
            file.write(f"target_pos: {target_pos}\n")
            file.write(f"target_orn: {target_orn}\n")
            file.write(f"actual_pos: {actual_pos}\n")
            file.write(f"actual_orn: {actual_orn}\n")
            file.write(f"per_error_pos: {per_error_pos}\n")
            file.write(f"quat_dif: {quat_dif}\n")
            #file.write(f"pos_error: {pos_error}\n")
            #file.write(f"orn_error: {orn_error}\n")
            file.write(f"ik_solution: {ik_solution}\n")
            file.write(f"joint_states: {joint_states}\n")

    ########################################################################################

class CameraStreamer:
    """Class to handle camera setup, image capture, and streaming via ZeroMQ."""

    def __init__(
        self,
        robot_id: int,
        cam_link_index: int,
        camera_intrinsics: Tuple[float, float, float, float],
        stream_port: int,
        camera_params: dict
    ):
        """
        Initialize the CameraStreamer.

        Args:
            robot_id (int): The robot's unique ID.
            cam_link_index (int): Camera link index.
            camera_intrinsics (Tuple[float, float, float, float]): (fx, fy, cx, cy).
            stream_port (int): ZeroMQ streaming port.
            camera_params (dict): Additional camera parameters.
        """
        self.robot_id = robot_id
        self.cam_link_index = cam_link_index
        self.fx, self.fy, self.cx, self.cy = camera_intrinsics
        self.stream_port = stream_port
        self.camera_params = camera_params
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(f"tcp://*:{self.stream_port}")
        print(f"Streaming RGBD images on port {self.stream_port}...")
        time.sleep(1)  # Allow subscribers to connect

    def compute_view_projection_matrices(
        self, camera_eye: List[float], camera_target: List[float], up_vector: List[float]
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute the view and projection matrices for the camera.

        Args:
            camera_eye (List[float]): Camera eye position.
            camera_target (List[float]): Camera target position.
            up_vector (List[float]): Up vector.

        Returns:
            Tuple[np.ndarray, np.ndarray]: View matrix and projection matrix.
        """
        view_matrix = p.computeViewMatrix(
            cameraEyePosition=camera_eye,
            cameraTargetPosition=camera_target,
            cameraUpVector=up_vector,
        )
        projection_matrix = p.computeProjectionMatrixFOV(
            fov=self.camera_params["fov"],
            aspect=self.camera_params["aspect"],
            nearVal=self.camera_params["near"],
            farVal=self.camera_params["far"],
        )
        return view_matrix, projection_matrix

    def capture_images(
        self, view_matrix: np.ndarray, projection_matrix: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Capture RGB and Depth images from the simulation.

        Args:
            view_matrix (np.ndarray): View matrix.
            projection_matrix (np.ndarray): Projection matrix.

        Returns:
            Tuple[np.ndarray, np.ndarray]: RGB image and Depth image.
        """
        images = p.getCameraImage(
            width=self.camera_params["width"],
            height=self.camera_params["height"],
            viewMatrix=view_matrix,
            projectionMatrix=projection_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL,
        )
        rgb = np.array(images[2], dtype=np.uint8).reshape(
            (self.camera_params["height"], self.camera_params["width"], 4)
        )[:, :, :3]
        depth = np.array(images[3]).reshape(
            (self.camera_params["height"], self.camera_params["width"])
        )
        return rgb, depth

    def normalize_depth(self, depth: np.ndarray) -> np.ndarray:
        """
        Normalize the depth image for transmission.

        Args:
            depth (np.ndarray): Depth image.

        Returns:
            np.ndarray: Normalized depth image as uint8.
        """
        depth_normalized = (depth - self.camera_params["near"]) / (
            self.camera_params["far"] - self.camera_params["near"]
        )
        depth_uint8 = (depth_normalized * 255).astype(np.uint8)
        return depth_uint8
    
    def close(self):
        """Close the ZeroMQ socket and terminate the context."""
        self.socket.close()
        self.context.term()

    #### Save RGB & Depth Images #############################################################
    def save_images(self,image_index,rgb_uint8,rgbd_rgb_uint8,rgbd_depth_uint16,depth_uint8):
        img_rgb = cv2.cvtColor(rgb_uint8, cv2.COLOR_BGR2RGB) 
        cv2.imwrite(f'rgb_image{image_index}.png', img_rgb)
        
        img_rgbd_rgb = cv2.cvtColor(rgbd_rgb_uint8, cv2.COLOR_BGR2RGB)
        cv2.imwrite(f'rgbd_rgb_image{image_index}.png', img_rgbd_rgb)
        
        img_rgbd_depth = cv2.cvtColor(rgbd_depth_uint16, cv2.COLOR_BGR2RGB)
        cv2.imwrite(f'rgbd_image{image_index}.png', img_rgbd_depth)
        
        img_depth = cv2.cvtColor(depth_uint8, cv2.COLOR_BGR2RGB)
        cv2.imwrite(f'depth_image{image_index}.png', img_depth)
    
    ########################################################################################

    # def serialize_and_send(
    #     self,
    #     rgb: np.ndarray,
    #     depth: np.ndarray,
    #     rgbd_rgb: np.ndarray,
    #     rgbd_depth: np.ndarray,
    # ):
    #     """
    #     Serialize the images and send them via ZeroMQ.

    #     Args:
    #         rgb (np.ndarray): RGB image.
    #         depth (np.ndarray): Depth image.
    #         rgbd_rgb (np.ndarray): RGB image from RGBD camera.
    #         rgbd_depth (np.ndarray): Depth image from RGBD camera.
    #     """
    #     # Serialize images
    #     rgb_bytes = rgb.tobytes()
    #     depth_bytes = self.normalize_depth(depth).tobytes()

    #     # Serialize camera parameters
    #     camera_params_bytes = struct.pack("ff", self.camera_params["near"], self.camera_params["far"])
    #     camera_intrinsics_bytes = struct.pack("ffff", self.fx, self.fy, self.cx, self.cy)

    #     # Serialize RGBD camera images
    #     rgbd_rgb_bytes = rgbd_rgb.tobytes()
    #     # rgbd_depth_bytes = self.normalize_depth(rgbd_depth).tobytes()
    #     rgbd_depth_normalized = (rgbd_depth - self.camera_params["near"]) / (1.0 - self.camera_params["near"])
    #     rgbd_depth_uint16 = (rgbd_depth_normalized * 65535).astype(np.uint16)
    #     rgbd_depth_bytes = rgbd_depth_uint16.tobytes()

    #     # Create header with sizes
    #     header = struct.pack(
    #         "IIIIII",
    #         len(rgb_bytes),
    #         len(depth_bytes),
    #         len(rgbd_rgb_bytes),
    #         len(rgbd_depth_bytes),
    #         len(camera_params_bytes),
    #         len(camera_intrinsics_bytes),
    #     )

    #     # Send multipart message
    #     self.socket.send_multipart(
    #         [
    #             header,
    #             rgb_bytes,
    #             depth_bytes,
    #             rgbd_rgb_bytes,
    #             rgbd_depth_bytes,
    #             camera_params_bytes,
    #             camera_intrinsics_bytes,
    #         ]
    #     )

def create_half_loop_trajectory(box_center, radius, height, Nposes, rgbd_cam_target):
    """
    Create a 'half-loop and return' trajectory: first half of a circular path,
    then reverse back to the starting point.

    Parameters:
    - box_center: 3D coordinates of the box center [x, y, z].
    - radius: Radius of the circular path.
    - height: Fixed height for all poses.
    - Nposes: Total number of poses (only half used + reverse).
    - rgbd_cam_target: 3D point the end-effector should look at.

    Returns:
    - tar_pos: List of target positions.
    - tar_orn: List of target orientations.
    """
    tar_pos = []
    tar_orn = []

    half = Nposes // 2
    angle_increment = 2 * np.pi / Nposes

    for i in range(half):
        theta = math.acos(1 - 2 * (i + 0.5) / Nposes)
        phi = angle_increment * i

        x = box_center[0] + radius * np.sin(theta) * np.cos(phi)
        y = box_center[1] + radius * np.sin(theta) * np.sin(phi)
        z = box_center[2] + height

        target_pos = [x, y, z]

        # Orientation toward the camera target
        view_direction = np.array(rgbd_cam_target) - np.array(target_pos)
        norm = np.linalg.norm(view_direction)
        if norm < 1e-6:
            view_direction = np.array([0, 0, 1])
        else:
            view_direction /= norm

        x_axis = np.cross([0, 0, 1], view_direction)
        x_axis /= np.linalg.norm(x_axis)

        y_axis = np.cross(view_direction, x_axis)
        rotation_matrix = np.array([x_axis, y_axis, view_direction]).T
        target_orn = rotm_to_quat(rotation_matrix)

        tar_pos.append(target_pos)
        tar_orn.append(target_orn)

    # Now reverse back (excluding the end point to avoid duplicate)
    tar_pos += tar_pos[-2::-1]
    tar_orn += tar_orn[-2::-1]

    return tar_pos, tar_orn

def create_opposite_half_trajectory_reversed(box_center, radius, height, Nposes, rgbd_cam_target):
    """
    Create the second half of a circular trajectory starting at the same initial position,
    and going in the reverse direction (opposite angular sweep).

    Parameters:
    - box_center: Center of the circle [x, y, z].
    - radius: Radius of the circular path.
    - height: Fixed height of end-effector.
    - Nposes: Total number of poses (only half will be used).
    - rgbd_cam_target: 3D point the robot should look at.

    Returns:
    - tar_pos: List of target positions (reversed).
    - tar_orn: List of target orientations (reversed).
    """
    tar_pos = []
    tar_orn = []

    half = Nposes // 2
    angle_increment = 2 * np.pi / Nposes

    for i in range(half):
        # This time, count BACKWARD through the second half
        idx = (Nposes - 1 - i) % Nposes
        theta = math.acos(1 - 2 * (idx + 0.5) / Nposes)
        phi = angle_increment * idx

        #x = box_center[0] + radius * np.sin(theta) * np.cos(phi)
        #y = box_center[1] + radius * np.sin(theta) * np.sin(phi)
        #z = box_center[2] + height

        x = box_center[0] + radius * np.cos(phi)
        y = box_center[1] + radius * np.sin(phi)
        z = box_center[2] + height  # constant height

        target_pos = [x, y, z]

        # Orientation
        view_direction = np.array(rgbd_cam_target) - np.array(target_pos)
        norm = np.linalg.norm(view_direction)
        if norm < 1e-6:
            view_direction = np.array([0, 0, 1])
        else:
            view_direction /= norm

        x_axis = np.cross([0, 0, 1], view_direction)
        x_axis /= np.linalg.norm(x_axis)

        y_axis = np.cross(view_direction, x_axis)
        rotation_matrix = np.array([x_axis, y_axis, view_direction]).T
        target_orn = rotm_to_quat(rotation_matrix)

        tar_pos.append(target_pos)
        tar_orn.append(target_orn)

    return tar_pos, tar_orn

def run_empty_simulation(tar_pos,tar_orn):
    """Main function to run the PyBullet simulation and stream RGBD images."""
    # Initialize PyBullet Simulator
    simulator = PyBulletSimulator(URDF_PATH)
    simulator.connect(gui=True)
    simulator.load_plane()

    # Setup Box and Load Objects
    object_manager = ObjectManager(lbb_box, dimensions)
    object_manager.setup_box()

    # Load Robot
    start_pos = [0, 0, 0.03]
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    simulator.load_robot(start_pos, start_orientation)
    robot_id = simulator.robot_id

    # Initialize Robot Controller
    robot_controller = RobotController(robot_id, END_EFFECTOR_LINK_NAME, CAMERA_LINK_NAME)
    ee_index = robot_controller.ee_index
    cam_index = robot_controller.cam_index
    print(f"End-Effector Index: {ee_index}")
    print(f"Camera Link Index: {cam_index}")
    

    # Initialize Camera Streamer (1st streamer for the RGBD camera on top of the box)
    camera_params = {
        "width": CAMERA_WIDTH,
        "height": CAMERA_HEIGHT,
        "fov": CAMERA_FOV,
        "aspect": CAMERA_WIDTH / CAMERA_HEIGHT,
        "near": CAMERA_NEAR,
        "far": CAMERA_FAR,
    }
    
    # camera on the end effector
    ee_camera_streamer = CameraStreamer(
        robot_id,
        cam_index,
        (FX, FY, CX, CY),
        STREAM_PORT,
        camera_params,
    )

    # Define Box Center for Camera Positioning
    box_center = np.array(lbb_box) + np.array(dimensions) / 2
    rgbd_cam_pos = box_center.copy()
    rgbd_cam_pos[2] = 0.5  # Adjust camera height
    rgbd_cam_target = rgbd_cam_pos - np.array([0, 0, 0.5])
    rgbd_up_vector = [1, 0, 0]
    print(f"RGBD Camera Position: {rgbd_cam_pos}")
    print(f"Box Center Ground: {rgbd_cam_target}")
    
    pos_err = []; orn_err = []
    i = 0; j = 0; t = 0
    startT = time.time()
    code_running = True
    try:
        for i in range(len(tar_pos)):#Nposes):
            j = 0
            for j in range(500): #range(Nposes):  # Iterate over the generated trajectory
                #ik_solution = np.array(traj[0])
                #print(ik_solution)
                target_pos = np.array(tar_pos[i])
                #print(target_pos)
                target_orn = np.array(tar_orn[i])
                #print(target_orn)

                # Step Simulation
                p.stepSimulation()

                # Add World Axis Debug Lines
                p.addUserDebugLine([0, 0, 0], [1, 0, 0], [1, 0, 0], 5, 1)
                p.addUserDebugLine([0, 0, 0], [0, 1, 0], [0, 1, 0], 5, 1)
                p.addUserDebugLine([0, 0, 0], [0, 0, 1], [0, 0, 1], 5, 1)

                # Retrieve Joint States
                #joint_states = p.getJointStates(robot_id, range(p.getNumJoints(robot_id))) ### Print angles, write in text file

                # Compute Inverse Kinematics
                ik_solution = robot_controller.compute_inverse_kinematics(
                    target_pos.tolist(),
                    target_orn,
                    lower_limits=[-200 * math.pi / 180] * 6,
                    upper_limits=[200 * math.pi / 180] * 6,
                    rest_poses=[0] * 6,
                    solver=0,
                )
                if j == 499:
                    print("##################")
                    print(target_pos)
                    print(target_orn)
                    print(ik_solution)
                    print("##################")

                # Apply Joint Positions
                robot_controller.apply_joint_positions(ik_solution)

                # Control Streaming Rate
                time.sleep(1.0 / STREAM_FPS)
                t += 1

            # Get Camera and End-Effector Poses
            cam_state = p.getLinkState(robot_id, cam_index)
            cam_pos = cam_state[4]  # World position
            cam_orn = cam_state[5]  # World orientation

            ee_state = p.getLinkState(robot_id, ee_index)
            ee_pos = ee_state[4]
            ee_orn = ee_state[5]

            # Compute Camera Direction and Up Vector
            camera_matrix = np.array(p.getMatrixFromQuaternion(cam_orn)).reshape(3, 3)
            x_vector = camera_matrix[:, 0]
            y_vector = camera_matrix[:, 1]
            z_vector = np.cross(x_vector, y_vector)
            up_vector = z_vector.tolist()

            # Define Camera Eye and Target Positions
            camera_eye_position = cam_pos + 0.042 * x_vector
            camera_target_position = camera_eye_position + 0.1 * y_vector

            # Get the actual end-effector state
            actual_state = p.getLinkState(robot_id, ee_index)
            actual_pos, actual_orn = actual_state[4], actual_state[5]
            # Calculate position and orientation errors
            pos_error = np.linalg.norm(np.array(target_pos) - np.array(actual_pos))
            orn_error = np.linalg.norm(np.array(target_orn) - np.array(actual_orn))

            pos_err.append(pos_error)
            orn_err.append(orn_error)

            # Define View and Projection Matrices
            view_matrix, projection_matrix = ee_camera_streamer.compute_view_projection_matrices(
                camera_eye_position.tolist(),
                camera_target_position.tolist(),
                up_vector,
            )

            # Capture RGB and Depth Images
            rgb, depth = ee_camera_streamer.capture_images(view_matrix, projection_matrix)

            # Capture RGBD Camera Images (if different)
            rgbd_images = p.getCameraImage(
                width=CAMERA_WIDTH,
                height=CAMERA_HEIGHT,
                viewMatrix=p.computeViewMatrix(
                    cameraEyePosition=rgbd_cam_pos.tolist(),
                    cameraTargetPosition=rgbd_cam_target.tolist(),
                    cameraUpVector=rgbd_up_vector,
                ),
                projectionMatrix=p.computeProjectionMatrixFOV(
                    fov=CAMERA_FOV,
                    aspect=float(CAMERA_WIDTH) / CAMERA_HEIGHT,
                    nearVal=CAMERA_NEAR,
                    farVal=0.5,
                ),
                renderer=p.ER_BULLET_HARDWARE_OPENGL,
            )
            rgbd_rgb = np.array(rgbd_images[2], dtype=np.uint8).reshape(
                (CAMERA_HEIGHT, CAMERA_WIDTH, 4)
            )[:, :, :3]
            rgbd_depth = np.array(rgbd_images[3]).reshape((CAMERA_HEIGHT, CAMERA_WIDTH))

            # Normalize RGBD Depth
            rgbd_depth_normalized = (rgbd_depth - CAMERA_NEAR) / (0.5 - CAMERA_NEAR)
            rgbd_depth_uint16 = (rgbd_depth_normalized * 65535).astype(np.uint16)

            # Serialize and Send Images (rgb,depth, rgbd_rgb, rgbd_depth, camera_params, camera_intrinsics)
                    
            # convert rgb and depth to uint8
            rgb_uint8 = rgb.astype(np.uint8)
            depth_uint8 = ee_camera_streamer.normalize_depth(depth)
            #cv2.imwrite('./depth.png',depth_uint8)
            #cv2.imwrite('./rgb.png',rgb_uint8)
            rgbd_rgb_uint8 = rgbd_rgb.astype(np.uint8)
            # for rgbd_depth, we wan't more precision, so convert to uint16
            rgbd_depth_uint16 = rgbd_depth_uint16.astype(np.uint16)
            rgb_bytes = rgb_uint8.tobytes()
            depth_bytes = depth_uint8.tobytes()
            rgbd_rgb_bytes = rgbd_rgb_uint8.tobytes()
            rgbd_depth_bytes = rgbd_depth_uint16.tobytes()
            # Serialize camera parameters
            camera_params_bytes = struct.pack("ff", CAMERA_NEAR, CAMERA_FAR)
            camera_intrinsics_bytes = struct.pack("ffff", FX, FY, CX, CY)
            # Create header with sizes
            header = struct.pack(
                "IIIIII",
                len(rgb_bytes),
                len(depth_bytes),
                len(rgbd_rgb_bytes),
                len(rgbd_depth_bytes),
                len(camera_params_bytes),
                len(camera_intrinsics_bytes),
            )
                    
            # Send multipart message
            ee_camera_streamer.socket.send(header + rgb_bytes + depth_bytes +rgbd_rgb_bytes+rgbd_depth_bytes+ camera_params_bytes + camera_intrinsics_bytes)

            image_index = 0
            ee_camera_streamer.save_images(image_index,rgb_uint8,rgbd_rgb_uint8,rgbd_depth_uint16,depth_uint8)

            # Control Streaming Rate
            time.sleep(1.0 / STREAM_FPS)
            t += 1
        '''
        for i in range(1):
            j = 0
            for j in range(30): #range(Nposes):  # Iterate over the generated trajectory
                #ik_solution = np.array(traj[0])
                #print(ik_solution)
                target_pos = start_pos
                #print(target_pos)
                target_orn = start_orientation
                #print(target_orn)

                # Step Simulation
                p.stepSimulation()

                # Add World Axis Debug Lines
                p.addUserDebugLine([0, 0, 0], [1, 0, 0], [1, 0, 0], 5, 1)
                p.addUserDebugLine([0, 0, 0], [0, 1, 0], [0, 1, 0], 5, 1)
                p.addUserDebugLine([0, 0, 0], [0, 0, 1], [0, 0, 1], 5, 1)

                # Retrieve Joint States
                #joint_states = p.getJointStates(robot_id, range(p.getNumJoints(robot_id))) ### Print angles, write in text file

                # Compute Inverse Kinematics
                ik_solution = robot_controller.compute_inverse_kinematics(
                    target_pos.tolist(),
                    target_orn,
                    lower_limits=[-200 * math.pi / 180] * 6,
                    upper_limits=[200 * math.pi / 180] * 6,
                    rest_poses=[0] * 6,
                    solver=0,
                )
                if j == 29:
                    print("##################")
                    print(target_pos)
                    print(target_orn)
                    print(ik_solution)
                    print("##################")

                # Apply Joint Positions
                robot_controller.apply_joint_positions(ik_solution)

                # Control Streaming Rate
                time.sleep(1.0 / STREAM_FPS)
                t += 1

            # Get Camera and End-Effector Poses
            cam_state = p.getLinkState(robot_id, cam_index)
            cam_pos = cam_state[4]  # World position
            cam_orn = cam_state[5]  # World orientation

            ee_state = p.getLinkState(robot_id, ee_index)
            ee_pos = ee_state[4]
            ee_orn = ee_state[5]

            # Compute Camera Direction and Up Vector
            camera_matrix = np.array(p.getMatrixFromQuaternion(cam_orn)).reshape(3, 3)
            x_vector = camera_matrix[:, 0]
            y_vector = camera_matrix[:, 1]
            z_vector = np.cross(x_vector, y_vector)
            up_vector = z_vector.tolist()

            # Define Camera Eye and Target Positions
            camera_eye_position = cam_pos + 0.042 * x_vector
            camera_target_position = camera_eye_position + 0.1 * y_vector

            # Get the actual end-effector state
            actual_state = p.getLinkState(robot_id, ee_index)
            actual_pos, actual_orn = actual_state[4], actual_state[5]
            # Calculate position and orientation errors
            pos_error = np.linalg.norm(np.array(target_pos) - np.array(actual_pos))
            orn_error = np.linalg.norm(np.array(target_orn) - np.array(actual_orn))

            pos_err.append(pos_error)
            orn_err.append(orn_error)

            # Define View and Projection Matrices
            view_matrix, projection_matrix = ee_camera_streamer.compute_view_projection_matrices(
                camera_eye_position.tolist(),
                camera_target_position.tolist(),
                up_vector,
            )

            # Capture RGB and Depth Images
            rgb, depth = ee_camera_streamer.capture_images(view_matrix, projection_matrix)

            # Capture RGBD Camera Images (if different)
            rgbd_images = p.getCameraImage(
                width=CAMERA_WIDTH,
                height=CAMERA_HEIGHT,
                viewMatrix=p.computeViewMatrix(
                    cameraEyePosition=rgbd_cam_pos.tolist(),
                    cameraTargetPosition=rgbd_cam_target.tolist(),
                    cameraUpVector=rgbd_up_vector,
                ),
                projectionMatrix=p.computeProjectionMatrixFOV(
                    fov=CAMERA_FOV,
                    aspect=float(CAMERA_WIDTH) / CAMERA_HEIGHT,
                    nearVal=CAMERA_NEAR,
                    farVal=0.5,
                ),
                renderer=p.ER_BULLET_HARDWARE_OPENGL,
            )
            rgbd_rgb = np.array(rgbd_images[2], dtype=np.uint8).reshape(
                (CAMERA_HEIGHT, CAMERA_WIDTH, 4)
            )[:, :, :3]
            rgbd_depth = np.array(rgbd_images[3]).reshape((CAMERA_HEIGHT, CAMERA_WIDTH))

            # Normalize RGBD Depth
            rgbd_depth_normalized = (rgbd_depth - CAMERA_NEAR) / (0.5 - CAMERA_NEAR)
            rgbd_depth_uint16 = (rgbd_depth_normalized * 65535).astype(np.uint16)

            # Serialize and Send Images (rgb,depth, rgbd_rgb, rgbd_depth, camera_params, camera_intrinsics)
                    
            # convert rgb and depth to uint8
            rgb_uint8 = rgb.astype(np.uint8)
            depth_uint8 = ee_camera_streamer.normalize_depth(depth)
            #cv2.imwrite('./depth.png',depth_uint8)
            #cv2.imwrite('./rgb.png',rgb_uint8)
            rgbd_rgb_uint8 = rgbd_rgb.astype(np.uint8)
            # for rgbd_depth, we wan't more precision, so convert to uint16
            rgbd_depth_uint16 = rgbd_depth_uint16.astype(np.uint16)
            rgb_bytes = rgb_uint8.tobytes()
            depth_bytes = depth_uint8.tobytes()
            rgbd_rgb_bytes = rgbd_rgb_uint8.tobytes()
            rgbd_depth_bytes = rgbd_depth_uint16.tobytes()
            # Serialize camera parameters
            camera_params_bytes = struct.pack("ff", CAMERA_NEAR, CAMERA_FAR)
            camera_intrinsics_bytes = struct.pack("ffff", FX, FY, CX, CY)
            # Create header with sizes
            header = struct.pack(
                "IIIIII",
                len(rgb_bytes),
                len(depth_bytes),
                len(rgbd_rgb_bytes),
                len(rgbd_depth_bytes),
                len(camera_params_bytes),
                len(camera_intrinsics_bytes),
            )
                    
            # Send multipart message
            ee_camera_streamer.socket.send(header + rgb_bytes + depth_bytes +rgbd_rgb_bytes+rgbd_depth_bytes+ camera_params_bytes + camera_intrinsics_bytes)

            image_index = 0
            ee_camera_streamer.save_images(image_index,rgb_uint8,rgbd_rgb_uint8,rgbd_depth_uint16,depth_uint8)

            # Control Streaming Rate
            time.sleep(1.0 / STREAM_FPS)
            t += 1

        for i in range(5,8): #Nposes/2,Nposes):
            j = 0
            for j in range(30): #range(Nposes):  # Iterate over the generated trajectory
                #ik_solution = np.array(traj[0])
                #print(ik_solution)
                target_pos = np.array(tar_pos[i])
                #print(target_pos)
                target_orn = np.array(tar_orn[i])
                #print(target_orn)

                # Step Simulation
                p.stepSimulation()

                # Add World Axis Debug Lines
                p.addUserDebugLine([0, 0, 0], [1, 0, 0], [1, 0, 0], 5, 1)
                p.addUserDebugLine([0, 0, 0], [0, 1, 0], [0, 1, 0], 5, 1)
                p.addUserDebugLine([0, 0, 0], [0, 0, 1], [0, 0, 1], 5, 1)

                # Retrieve Joint States
                #joint_states = p.getJointStates(robot_id, range(p.getNumJoints(robot_id))) ### Print angles, write in text file

                # Compute Inverse Kinematics
                ik_solution = robot_controller.compute_inverse_kinematics(
                    target_pos.tolist(),
                    target_orn,
                    lower_limits=[-200 * math.pi / 180] * 6,
                    upper_limits=[200 * math.pi / 180] * 6,
                    rest_poses=[0] * 6,
                    solver=0,
                )
                if j == 29:
                    print("##################")
                    print(target_pos)
                    print(target_orn)
                    print(ik_solution)
                    print("##################")

                # Apply Joint Positions
                robot_controller.apply_joint_positions(ik_solution)

                # Control Streaming Rate
                time.sleep(1.0 / STREAM_FPS)
                t += 1

            # Get Camera and End-Effector Poses
            cam_state = p.getLinkState(robot_id, cam_index)
            cam_pos = cam_state[4]  # World position
            cam_orn = cam_state[5]  # World orientation

            ee_state = p.getLinkState(robot_id, ee_index)
            ee_pos = ee_state[4]
            ee_orn = ee_state[5]

            # Compute Camera Direction and Up Vector
            camera_matrix = np.array(p.getMatrixFromQuaternion(cam_orn)).reshape(3, 3)
            x_vector = camera_matrix[:, 0]
            y_vector = camera_matrix[:, 1]
            z_vector = np.cross(x_vector, y_vector)
            up_vector = z_vector.tolist()

            # Define Camera Eye and Target Positions
            camera_eye_position = cam_pos + 0.042 * x_vector
            camera_target_position = camera_eye_position + 0.1 * y_vector

            # Get the actual end-effector state
            actual_state = p.getLinkState(robot_id, ee_index)
            actual_pos, actual_orn = actual_state[4], actual_state[5]
            # Calculate position and orientation errors
            pos_error = np.linalg.norm(np.array(target_pos) - np.array(actual_pos))
            orn_error = np.linalg.norm(np.array(target_orn) - np.array(actual_orn))

            pos_err.append(pos_error)
            orn_err.append(orn_error)

            # Define View and Projection Matrices
            view_matrix, projection_matrix = ee_camera_streamer.compute_view_projection_matrices(
                camera_eye_position.tolist(),
                camera_target_position.tolist(),
                up_vector,
            )

            # Capture RGB and Depth Images
            rgb, depth = ee_camera_streamer.capture_images(view_matrix, projection_matrix)

            # Capture RGBD Camera Images (if different)
            rgbd_images = p.getCameraImage(
                width=CAMERA_WIDTH,
                height=CAMERA_HEIGHT,
                viewMatrix=p.computeViewMatrix(
                    cameraEyePosition=rgbd_cam_pos.tolist(),
                    cameraTargetPosition=rgbd_cam_target.tolist(),
                    cameraUpVector=rgbd_up_vector,
                ),
                projectionMatrix=p.computeProjectionMatrixFOV(
                    fov=CAMERA_FOV,
                    aspect=float(CAMERA_WIDTH) / CAMERA_HEIGHT,
                    nearVal=CAMERA_NEAR,
                    farVal=0.5,
                ),
                renderer=p.ER_BULLET_HARDWARE_OPENGL,
            )
            rgbd_rgb = np.array(rgbd_images[2], dtype=np.uint8).reshape(
                (CAMERA_HEIGHT, CAMERA_WIDTH, 4)
            )[:, :, :3]
            rgbd_depth = np.array(rgbd_images[3]).reshape((CAMERA_HEIGHT, CAMERA_WIDTH))

            # Normalize RGBD Depth
            rgbd_depth_normalized = (rgbd_depth - CAMERA_NEAR) / (0.5 - CAMERA_NEAR)
            rgbd_depth_uint16 = (rgbd_depth_normalized * 65535).astype(np.uint16)

            # Serialize and Send Images (rgb,depth, rgbd_rgb, rgbd_depth, camera_params, camera_intrinsics)
                    
            # convert rgb and depth to uint8
            rgb_uint8 = rgb.astype(np.uint8)
            depth_uint8 = ee_camera_streamer.normalize_depth(depth)
            #cv2.imwrite('./depth.png',depth_uint8)
            #cv2.imwrite('./rgb.png',rgb_uint8)
            rgbd_rgb_uint8 = rgbd_rgb.astype(np.uint8)
            # for rgbd_depth, we wan't more precision, so convert to uint16
            rgbd_depth_uint16 = rgbd_depth_uint16.astype(np.uint16)
            rgb_bytes = rgb_uint8.tobytes()
            depth_bytes = depth_uint8.tobytes()
            rgbd_rgb_bytes = rgbd_rgb_uint8.tobytes()
            rgbd_depth_bytes = rgbd_depth_uint16.tobytes()
            # Serialize camera parameters
            camera_params_bytes = struct.pack("ff", CAMERA_NEAR, CAMERA_FAR)
            camera_intrinsics_bytes = struct.pack("ffff", FX, FY, CX, CY)
            # Create header with sizes
            header = struct.pack(
                "IIIIII",
                len(rgb_bytes),
                len(depth_bytes),
                len(rgbd_rgb_bytes),
                len(rgbd_depth_bytes),
                len(camera_params_bytes),
                len(camera_intrinsics_bytes),
            )
                    
            # Send multipart message
            ee_camera_streamer.socket.send(header + rgb_bytes + depth_bytes +rgbd_rgb_bytes+rgbd_depth_bytes+ camera_params_bytes + camera_intrinsics_bytes)

            image_index = 0
            ee_camera_streamer.save_images(image_index,rgb_uint8,rgbd_rgb_uint8,rgbd_depth_uint16,depth_uint8)

            # Control Streaming Rate
            time.sleep(1.0 / STREAM_FPS)
            t += 1
        '''
    except KeyboardInterrupt:
        print("Streaming stopped by user.")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        print("Disconnecting and cleaning up...")
        # Save IK Solutions
        if robot_controller.ik_solutions:
            np.save(TRAJECTORY_SAVING_PATH, robot_controller.ik_solutions)
            print(f"Saved IK solutions: {len(robot_controller.ik_solutions)} samples.")

        # Cleanup
        ee_camera_streamer.close()
        simulator.disconnect()

    return None #rgb_uint8, rgbd_rgb_uint8, rgbd_depth_uint16, depth_uint8, pos_err, orn_err #rgb, rgbd_rgb, rgbd_depth, depth

# Use create_trajectory to generate the robot's circular trajectory
# First half + return
pos1, orn1 = create_half_loop_trajectory(box_center, radius, height, Nposes, rgbd_cam_target)

# Second half, other side
pos3, orn3 = create_opposite_half_trajectory_reversed(box_center, radius, height, Nposes, rgbd_cam_target)

# Full sequence: half loop → reverse → opposite half
tar_pos= pos1 + pos3
tar_orn = orn1 + orn3

print('TARGET POSE')
print(tar_pos)

print('TARGET ORN')
print(tar_orn)

run_empty_simulation(tar_pos,tar_orn)
