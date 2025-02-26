import pybullet as p
import time
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt


# Camera parameters
distance = 0.6  # Fixed distance from the object

min_lat = 30; max_lat = 90; step_lat = 10
min_lon = 0; max_lon = 360; step_lon = 10
euler_or = [90, 0, 0]
object = "./small_clamp/model.urdf" #"./power_drill/model.urdf"


# Function to compute the camera position
def compute_camera_position(longitude, latitude, distance):
    # Convert longitude and latitude from degrees to radians
    longitude = np.radians(longitude)
    latitude = np.radians(latitude)
    
    # Compute camera position using spherical coordinates
    x = distance * np.sin(latitude) * np.cos(longitude)
    y = distance * np.sin(latitude) * np.sin(longitude)
    z = distance * np.cos(latitude)
    
    return [x, y, z]
'''
# Function to compute the camera's quaternion orientation from yaw and pitch
def compute_camera_orientation(longitude, latitude):
    # Convert yaw and pitch to radians
    longitude = np.radians(longitude)
    latitude = np.radians(latitude)
    
    # Quaternion rotation from pitch and yaw
    q = p.getQuaternionFromEuler([latitude, 0, longitude])  # (roll, pitch, yaw)
    
    return q
'''
# Function to capture RGBD image and compute foreground depth
def capture_empty_rgbd_image(longitude, latitude):

    # Compute camera position
    camera_position = compute_camera_position(longitude, latitude, distance)

    print(camera_position)

    # Define camera parameters
    view_matrix = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=[0, 0, 1],  # The camera targets the center of the object
        distance=distance,
        yaw=longitude,  # Longitude (rotation around the Z-axis)
        pitch=latitude,  # Latitude (rotation around the X-axis)
        roll=0,
        upAxisIndex=2
    )

    projection_matrix = p.computeProjectionMatrixFOV(
        fov=60,  # Field of view
        aspect=1.0,  # Aspect ratio (width/height)
        nearVal=0.1,  # Near plane
        farVal=100.0  # Far plane
    )

    # Capture an RGBD image (including depth)
    width, height, rgb_img, depth_img, seg_img = p.getCameraImage(
        width=640,  # Width of the image
        height=480,  # Height of the image
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
        lightDirection=[1, 1, 1],  # Direction of light source (optional)
    )

    # Convert the RGB image to a numpy array
    rgb_img = np.array(rgb_img)

    # Normalize and process the depth image (depth is in meters)
    depth_img = np.array(depth_img)
    depth_img = depth_img / 1000.0  # Convert depth to meters (since PyBullet depth is in mm)

    print(np.min(depth_img))
    print(np.max(depth_img))

    # Return the results and the images
    return rgb_img, depth_img

# Function to capture RGBD image and compute foreground depth
def capture_full_rgbd_image(longitude, latitude, empty_results):
    # Remove all previous debug items (lines, spheres, etc.)
    p.removeAllUserDebugItems()

    # Compute camera position
    camera_position = compute_camera_position(longitude, latitude, distance)

    print(camera_position)

    # Define camera parameters
    view_matrix = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=[0, 0, 1],  # The camera targets the center of the object
        distance=distance,
        yaw=longitude,  # Longitude (rotation around the Z-axis)
        pitch=latitude,  # Latitude (rotation around the X-axis)
        roll=0,
        upAxisIndex=2
    )
    '''
    # Add a debug line to visualize the camera's position
    p.addUserDebugLine(
        lineFromXYZ=[camera_position[0]+3,camera_position[1]+0,camera_position[2]], #camera_position,    # Camera position (start of line)
        lineToXYZ=[0, 0, 1],            # Target point (center of the object)
        lineColorRGB=[1, 0, 0],  # Red line
        lineWidth=2
    )
    '''
    projection_matrix = p.computeProjectionMatrixFOV(
        fov=60,  # Field of view
        aspect=1.0,  # Aspect ratio (width/height)
        nearVal=0.1,  # Near plane
        farVal=100.0  # Far plane
    )

    # Capture an RGBD image (including depth)
    width, height, rgb_img, depth_img, seg_img = p.getCameraImage(
        width=640,  # Width of the image
        height=480,  # Height of the image
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
        lightDirection=[1, 1, 1],  # Direction of light source (optional)
    )

    # Convert the RGB image to a numpy array
    rgb_img = np.array(rgb_img)

    # Normalize and process the depth image (depth is in meters)
    depth_img = np.array(depth_img)
    depth_img = depth_img / 1000.0  # Convert depth to meters (since PyBullet depth is in mm)

    print(np.min(depth_img))
    print(np.max(depth_img))

    # Step 1: Subtract the background from the depth image
    foreground_mask = depth_img < empty_depth_img
    foreground_depth = depth_img[foreground_mask]
    #print(f"Foreground Mask: {np.sum(foreground_mask)} pixels")

    # Step 2: Compute sum and mean of the pixel values
    rgb_sum = np.sum(rgb_img)
    rgb_mean = np.mean(rgb_img)
    
    if foreground_depth.size == 0:
        print(f"No foreground depth values for longitude={longitude}, latitude={latitude}")
    else:
        foreground_depth = foreground_depth[foreground_depth>0]
        depth_sum = np.sum(foreground_depth)
        depth_mean = np.mean(foreground_depth)
        depth_var = np.var(foreground_depth)

    # Return the results and the images
    return rgb_img, depth_img, foreground_depth, rgb_sum, rgb_mean, depth_sum, depth_mean, depth_var

def run_empty_simulation():
    # Connect to PyBullet (use GUI or direct simulation)
    p.connect(p.GUI)  # For GUI visualization
    # p.connect(p.DIRECT)  # Use this for headless simulation (without GUI)

    # Set the path to PyBullet's default URDFs
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Load a simple object (cube in this case)
    planeId = p.loadURDF("plane.urdf")
    #basePosition=[0, 0, 1]; baseOrientation=p.getQuaternionFromEuler(euler_or); cube_id = p.loadURDF(object, basePosition, baseOrientation, globalScaling=3)

    # Collecting results for 360-degree rotation
    #longitude_steps = range(0, 360, 10)  # 0 to 360 degrees, step of 10 degrees (longitude)
    #latitude_steps = range(30, 90, 10)  # 0 to 90 degrees, step of 10 degrees (latitude)
    longitude_steps = range(min_lon, max_lon, step_lon)  # 0 to 360 degrees, step of 10 degrees (longitude)
    latitude_steps = range(min_lat, max_lat, step_lat)  # 0 to 90 degrees, step of 10 degrees (latitude)

    # Store results
    empty_results = []

    for longitude in longitude_steps:
        for latitude in latitude_steps:
            #print(f"Capturing image at longitude={longitude}°, latitude={latitude}°")

            # Capture RGBD image and compute statistics
            empty_rgb_img, empty_depth_img = capture_empty_rgbd_image(longitude, -1*latitude)
            '''
            # Optionally: Display the images during each rotation step
            plt.figure(figsize=(10, 5))

            # Subplot 1: RGB Image
            plt.subplot(1, 2, 1)
            plt.imshow(empty_rgb_img)
            plt.title(f"RGB Image (longitude={longitude}°, latitude={latitude}°)")
            plt.axis('off')

            # Subplot 2: Depth Image
            plt.subplot(1, 2, 2)
            plt.imshow(empty_depth_img, cmap='jet')
            plt.title("Depth Image")
            plt.colorbar(label="Depth (meters)")
            plt.axis('off')
            '''
            
            '''
            # Subplot 3: Foreground Depth Image
            plt.subplot(1, 3, 3)
            plt.imshow(foreground_depth, cmap='jet')
            plt.title("Foreground Depth Image")
            plt.colorbar(label="Foreground Depth (meters)")
            plt.axis('off')
            '''
            
            '''
            plt.tight_layout()
            plt.show()

            time.sleep(1 / 240.)  # Small delay for visualization
            '''

        print('longitude')
        print(longitude)
        #print(results)
        
    # Disconnect when done
    p.disconnect()

    return empty_rgb_img, empty_depth_img

def run_full_simulation(empty_depth_img):
    # Connect to PyBullet (use GUI or direct simulation)
    p.connect(p.GUI)  # For GUI visualization
    # p.connect(p.DIRECT)  # Use this for headless simulation (without GUI)

    # Set the path to PyBullet's default URDFs
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Load a simple object (cube in this case)
    planeId = p.loadURDF("plane.urdf")
    basePosition=[0, 0, 1]
    baseOrientation=p.getQuaternionFromEuler(euler_or)
    cube_id = p.loadURDF(object, basePosition, baseOrientation, globalScaling=3)

    #print(basePosition); print(baseOrientation)

    # Retrieve the position and orientation of the cube
    position, orientation = p.getBasePositionAndOrientation(cube_id)

    # Convert the position and orientation into a single vector (using NumPy array)
    object_position_vector = np.array(position)
    object_orientation_vector = np.array(orientation)

    np.save('object_position.npy', object_position_vector)
    np.save('object_orientation.npy', object_orientation_vector)


    # Collecting results for 360-degree rotation
    #longitude_steps = range(0, 360, 10)  # 0 to 360 degrees, step of 10 degrees (longitude)
    #latitude_steps = range(30, 90, 10)  # 0 to 90 degrees, step of 10 degrees (latitude)
    longitude_steps = range(min_lon, max_lon, step_lon) 
    latitude_steps = range(min_lat, max_lat, step_lat)

    # Store results
    results = []

    for longitude in longitude_steps:
        for latitude in latitude_steps:
            #print(f"Capturing image at longitude={longitude}°, latitude={latitude}°")

            #longitude = 0
            #latitude = -40

            # Capture RGBD image and compute statistics
            rgb_img, depth_img, foreground_depth, rgb_sum, rgb_mean, depth_sum, depth_mean, depth_var = capture_full_rgbd_image(longitude, -1*latitude, empty_depth_img)

            # Store the results
            results.append({
                'longitude': longitude,
                'latitude': latitude,
                'rgb_sum': rgb_sum,
                'rgb_mean': rgb_mean,
                'depth_sum': depth_sum,
                'depth_mean': depth_mean,
                'depth_var': depth_var
            })

        print('longitude')
        print(longitude)
        #print(results)

    '''
            # Optionally: Display the images during each rotation step
            plt.figure(figsize=(10, 5))

            # Subplot 1: RGB Image
            plt.subplot(1, 3, 1)
            plt.imshow(rgb_img)
            plt.title(f"RGB Image (longitude={longitude}°, latitude={latitude}°)")
            plt.axis('off')

            # Subplot 2: Depth Image
            plt.subplot(1, 3, 2)
            plt.imshow(depth_img, cmap='jet')
            plt.title("Depth Image")
            plt.colorbar(label="Depth (meters)")
            plt.axis('off')

            # Subplot 3: Foreground Depth Image
            plt.subplot(1, 3, 3)
            plt.imshow(foreground_depth, cmap='jet')
            plt.title("Foreground Depth Image")
            plt.colorbar(label="Foreground Depth (meters)")
            plt.axis('off')

            plt.tight_layout()
            plt.show()

            time.sleep(1 / 240.)  # Small delay for visualization
    '''
    # Disconnect when done
    p.disconnect()

    # Extract the sum and mean values
    x1 = int((max_lat-min_lat)/step_lat)
    x2 = int((max_lon-min_lon)/step_lon)
    sum_data = np.zeros((x1, x2))  
    mean_data = np.zeros((x1, x2))  
    var_data = np.zeros((x1, x2)) 
    
    #sum_data = np.zeros((9, 36))  # 9 latitudes, 36 longitudes
    #mean_data = np.zeros((9, 36))  # 9 latitudes, 36 longitudes
    #var_data = np.zeros((9, 36))  # 9 latitudes, 36 longitudes

    for result in results:
        longitude_idx = int(result['longitude'] // step_lon)  # longitude index (0 to 360° -> 36 steps)
        if (min_lat > 0):
            inc = min_lat/step_lat
        else:
            inc = 0
        latitude_idx = int((result['latitude'] // step_lat) - inc) # latitude index (0 to 90° -> 9 steps)
        
        if result['latitude'] == 90:
            latitude_idx = int(x1-1)  # Explicitly set the index to 8 for latitude = 90°
        
        #print(longitude_idx);
        #print(latitude_idx)
    #else:
    #     latitude_idx = latitude // 10  # For other latitudees, use the floor division approach
        
        sum_data[latitude_idx, longitude_idx] = result['depth_sum']  # Store depth sum
        mean_data[latitude_idx, longitude_idx] = result['depth_mean']  # Store depth mean
        var_data[latitude_idx, longitude_idx] = result['depth_var']

    # Longitude and latitude for the spherical projection
    lon = np.linspace(-np.pi, np.pi, x2)  # Longitude: 0° to 360° (in radians)
    lat = np.linspace(0, np.pi / 2., x1)   # Latitude: 0° to 90° (in radians)
    #lon = np.linspace(-np.pi, np.pi, 6)  # Longitude: 0° to 360° (in radians)
    #lat = np.linspace(0, np.pi / 2., 3)   # Latitude: 0° to 90° (in radians)

    # Create a mesh grid for the spherical projection
    Lon, Lat = np.meshgrid(lon, lat)

    sum_data_normalized = (sum_data-np.min(sum_data))/(np.max(sum_data)-np.min(sum_data))
    mean_data_normalized = (mean_data-np.min(mean_data))/(np.max(mean_data)-np.min(mean_data))
    var_data_normalized = (var_data-np.min(var_data))/(np.max(var_data)-np.min(var_data))

    # Save sum_data and mean_data to a .npy file
    np.save('longitude_latitude_depth_sum.npy', sum_data)
    np.save('longitude_latitude_depth_mean.npy', mean_data)


    print("Sum"); print(sum_data_normalized)
    print("Mean"); print(mean_data_normalized)
    print("Variance"); print(var_data_normalized)

    #print(Lon) print(Lat)
    
    # Plotting the results
    fig = plt.figure(figsize=(14, 7))

    # Plot for the sum of pixel values (depth sum)
    ax1 = fig.add_subplot(131, projection='mollweide')
    im1 = ax1.pcolormesh(Lon, Lat, sum_data_normalized, cmap=plt.cm.jet)
    #ax1.set_title('Normalized Total Sum of Depth \nPixels for Each Camera View', fontsize=18)
    ax1.set_xticks([])  # Hide the longitude ticks
    ax1.set_yticks([])
    fig.colorbar(im1, ax=ax1, fraction=0.03, pad=0.04) #label="Sum Value")

    # Plot for the mean of pixel values (depth mean)
    ax2 = fig.add_subplot(132, projection='mollweide')
    im2 = ax2.pcolormesh(Lon, Lat, mean_data_normalized, cmap=plt.cm.jet)
    #ax2.set_title('Normalized Average Depth \nValue for Each Camera View', fontsize=18)
    ax2.set_xticks([])  # Hide the longitude ticks
    ax2.set_yticks([])
    fig.colorbar(im2, ax=ax2, fraction=0.03, pad=0.04)#label="Mean Value")

    # Plot for the mean of pixel values (depth mean)
    ax3 = fig.add_subplot(133, projection='mollweide')
    im3 = ax3.pcolormesh(Lon, Lat, var_data_normalized, cmap=plt.cm.jet)
    #ax3.set_title('Normalized Standard Deviation of \nDepth Value for Each Camera View', fontsize=18)
    ax3.set_xticks([])  # Hide the longitude ticks
    ax3.set_yticks([])
    fig.colorbar(im3, ax=ax3, fraction=0.03, pad=0.04) #, label="Variance")

    # Show the plots
    plt.tight_layout()
    plt.show()
    

empty_rgb_img, empty_depth_img = run_empty_simulation()

run_full_simulation(empty_depth_img)

