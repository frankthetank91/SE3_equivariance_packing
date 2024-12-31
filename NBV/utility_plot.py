import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def plot_camera(vecApex,vecDepth,vecSum,rankDepth,rankSum,pos):
    # Parameters
    R = 0.25
    pyramid_height = 0.05
    pyramid_base_side = 0.05

    #-------------------------------------------
    # Generate Hemisphere (z >= 0)
    #-------------------------------------------
    # Use spherical coordinates
    phi = np.linspace(0, np.pi/2, 50)  # 0 to pi/2 for the upper hemisphere
    theta = np.linspace(0, 2*np.pi, 50)

    phi, theta = np.meshgrid(phi, theta)

    center_x = 0.21
    center_y = 0.0
    center_z = 0.0

    x = R * np.sin(phi) * np.cos(theta)
    y = R * np.sin(phi) * np.sin(theta)
    z = R * np.cos(phi)

    x += center_x
    y += center_y
    z += center_z

    fig = plt.figure(figsize=(8,8))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(x, y, z, color='lightblue', alpha=0.1, rstride=1, cstride=1, linewidth=0)

    for i in range(8):
    #-------------------------------------------
    # Define Camera Apex on the Hemisphere
    #-------------------------------------------
    # Let's pick a point on the hemisphere, for example:
    # apex at some angle: phi_cam and theta_cam
        x_apex = vecApex[i][0]
        y_apex = vecApex[i][1]
        z_apex = vecApex[i][2]
        apex = np.array([x_apex, y_apex, z_apex])
        print(x_apex,y_apex,z_apex)

        #-------------------------------------------
        # Define Pyramid representing the Camera
        #-------------------------------------------
        # The pyramid apex is at 'apex'.
        # The pyramid height and base is defined.
        # The base center is along the vector from apex to origin.
        target_point = np.array([0.21, 0, 0])
        direction = target_point-apex  # Vector from apex to origin
        direction_norm = direction / np.linalg.norm(direction)

        # Apex is at apex
        # Base center will be apex + direction_norm * pyramid_height
        base_center = apex + direction_norm * pyramid_height

        # We need to find a coordinate system to define the square base.
        # The base should be perpendicular to the apex-origin direction.
        # Let's pick an arbitrary vector not collinear with direction to construct an orthonormal basis.
        up = np.array([0,0,1])
        if np.allclose(direction_norm, up, atol=1e-8) or np.allclose(direction_norm, -up, atol=1e-8):
            # direction is aligned or anti-aligned with z-axis, pick another vector
            up = np.array([1,0,0])

        # Construct orthonormal vectors for the base plane
        # First vector perpendicular to direction_norm:
        v1 = np.cross(direction_norm, up)
        v1 = v1 / np.linalg.norm(v1)
        # Second vector perpendicular to direction_norm and v1:
        v2 = np.cross(direction_norm, v1)
        v2 = v2 / np.linalg.norm(v2)

        # The base is a square centered at base_center with side length pyramid_base_side.
        half_side = pyramid_base_side / 2.0
        # Base corners relative to base_center
        base_offsets = [
            v1*half_side + v2*half_side,
            v1*half_side - v2*half_side,
            -v1*half_side - v2*half_side,
            -v1*half_side + v2*half_side
        ]

        base_points = [base_center + offset for offset in base_offsets]

        # Pyramid vertices
        vertices = [apex] + base_points

        # Faces of the pyramid (each face a triangle)
        faces = [
            [apex, base_points[0], base_points[1]],
            [apex, base_points[1], base_points[2]],
            [apex, base_points[2], base_points[3]],
            [apex, base_points[3], base_points[0]],
            # Base face (optional to show): [base_points[0], base_points[1], base_points[2], base_points[3]]
        ]

        #-------------------------------------------
        # Plotting
        #-------------------------------------------
        '''
        fig = plt.figure(figsize=(8,8))
        ax = fig.add_subplot(111, projection='3d')
        ax.plot_surface(x, y, z, color='lightblue', alpha=0.1, rstride=1, cstride=1, linewidth=0)
        '''

        # Plot the camera pyramid
        # Draw edges of the pyramid for clarity
        for face in faces:
            # face is a list of vertices
            # close the polygon
            face_points = np.array(face + [face[0]])
            ax.plot(face_points[:,0], face_points[:,1], face_points[:,2], color='red')

        # Optionally draw the base separately if you wish
        base_polygon = np.array(base_points + [base_points[0]])
        ax.plot(base_polygon[:,0], base_polygon[:,1], base_polygon[:,2], color='green')

        # Mark the apex point
        colorVec = np.array(['red','green','blue','orange','brown','purple','pink','cyan'])
        #ax.scatter(apex[0], apex[1], apex[2], color=colorVec[i], s=50, label=f'{pos[i]}')

        #ax.scatter(apex[0], apex[1], apex[2], color=colorVec[i], s=50, label=f'rank: {rankDepth[i]}, value: {vecDepth[i]}')
        #ax.text(apex[0], apex[1], apex[2], f"  {rankDepth[i]}", color='blue')

        ax.scatter(apex[0], apex[1], apex[2], color=colorVec[i], s=50, label=f'rank: {rankSum[i]}, value: {vecSum[i]}')
        ax.text(apex[0], apex[1], apex[2], f"  {rankSum[i]}", color='blue')

        ax.view_init(elev=40, azim=180)

    # Box dimensions
    length = 0.22  # along x-axis
    width = 0.3    # along y-axis
    height = 0.1   # along z-axis

    # Center the box at (0, 0, 0)
    # Adjusting vertices by subtracting half of the box dimensions
    half_length = length / 2
    half_width = width / 2
    half_height = height / 2

    vertices = np.array([
        [-half_length, -half_width, -half_height],  # (x1, y1, z1)
        [half_length, -half_width, -half_height],  # (x2, y1, z1)
        [half_length, half_width, -half_height],  # (x2, y2, z1)
        [-half_length, half_width, -half_height],  # (x1, y2, z1)
        [-half_length, -half_width, half_height],  # (x1, y1, z2)
        [half_length, -half_width, half_height],  # (x2, y1, z2)
        [half_length, half_width, half_height],  # (x2, y2, z2)
        [-half_length, half_width, half_height]  # (x1, y2, z2)
    ])

    target_center = np.array([0.21, 0, 0])
    vertices += target_center

    # List of faces defined by the vertices (order of vertices)
    faces = [
        [vertices[0], vertices[1], vertices[2], vertices[3]],  # bottom
        [vertices[4], vertices[5], vertices[6], vertices[7]],  # top
        [vertices[0], vertices[1], vertices[5], vertices[4]],  # front
        [vertices[1], vertices[2], vertices[6], vertices[5]],  # right
        [vertices[2], vertices[3], vertices[7], vertices[6]],  # back
        [vertices[3], vertices[0], vertices[4], vertices[7]]   # left
    ]

    ax.add_collection3d(Poly3DCollection(faces, facecolors='brown', linewidths=1, edgecolors='r', alpha=.5))

    # Enhance plot appearance
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    #ax.set_title('Camera Positions and Normalized Depth')
    ax.set_title('Camera Positions and Normalized Sum')
    # ax.set_box_aspect([1,1,1])  # equal aspect ratio

    # Set limits to nicely view the hemisphere and camera
    ax.set_xlim([-0.1, 0.5])
    ax.set_ylim([-R-0.1, R+0.1])
    ax.set_zlim([0, R+0.1])
    # set each axis is in equal scale
    ax.set_aspect('equal')

    #plt.legend(title='Position')
    #plt.legend(title='Normalized Depth Count')
    plt.legend(title='Normalized Sum')
    plt.tight_layout()
    #plt.savefig('camera_depth_plot.png', dpi=300, bbox_inches='tight')
    plt.savefig('camera_sum_plot.png', dpi=300, bbox_inches='tight')
    plt.show()

# Positions
#Center
x_apex = 0.20876280963420868; y_apex = 7.422822818625718e-05; z_apex = 0.2486495077610016
apexC = np.array([x_apex, y_apex, z_apex])

#Center Back
x_apex = 0.11068261414766312; y_apex = -0.00041275974945165217; z_apex = 0.24987506866455078
apexCB = np.array([x_apex, y_apex, z_apex])

#Left Side
x_apex = 0.1953950822353363; y_apex = 0.08930594474077225; z_apex = 0.23143133521080017
apexLS = np.array([x_apex, y_apex, z_apex])

#Right Side
x_apex = 0.20232878625392914; y_apex = -0.09407271444797516; z_apex = 0.24158577620983124
apexRS = np.array([x_apex, y_apex, z_apex])

#Left Side Middle
x_apex = 0.20805159211158752; y_apex = 0.04929427430033684; z_apex = 0.24765601754188538
apexLSM = np.array([x_apex, y_apex, z_apex])

#Right Side Middle
x_apex = 0.21068497002124786; y_apex = -0.05026287958025932; z_apex = 0.2507428526878357
apexRSM = np.array([x_apex, y_apex, z_apex])

# Bottom Left Corner
x_apex = 0.10856112092733383; y_apex = 0.14277978241443634; z_apex = 0.23962701857089996
apexBLC = np.array([x_apex, y_apex, z_apex])

# Bottom Right Corner
x_apex = 0.11000581085681915; y_apex = -0.1514127552509308; z_apex = 0.25131651759147644
apexBRC = np.array([x_apex, y_apex, z_apex])

'''
# Run 1
#Center
depth_dataC = 64698
sum_dataC = 1266.6036376953125/depth_dataC

#Center Back
depth_dataCB = 52582
sum_dataCB = 833.2899169921875/depth_dataCB

#Left Side
depth_dataLS = 61515
sum_dataLS = 1154.372802734375/depth_dataLS

#Right Side
depth_dataRS = 59511
sum_dataRS = 1047.5054931640625/depth_dataRS

#Left Side Middle
depth_dataLSM = 61553
sum_dataLSM = 1066.380126953125/depth_dataLSM

#Right Side Middle
depth_dataRSM = 62074
sum_dataRSM = 1038.6591796875/depth_dataRSM

# Bottom Left Corner
depth_dataBLC = 44994
sum_dataBLC = 669.2198486328125/depth_dataBLC

# Bottom Right Corner
depth_dataBRC = 36329
sum_dataBRC = 483.1808776855469/depth_dataBRC


# Run 2
#Center
depth_dataC = 23778
sum_dataC = 323.6385803222656/depth_dataC

#Center Back
depth_dataCB = 11579
sum_dataCB = 182.19192504882812/depth_dataCB

#Left Side
depth_dataLS = 12623
sum_dataLS = 236.51766967773438/depth_dataLS

#Right Side
depth_dataRS = 12753
sum_dataRS = 212.19964599609375/depth_dataRS

#Left Side Middle
depth_dataLSM = 12184
sum_dataLSM = 229.82748413085938/depth_dataLSM

#Right Side Middle
depth_dataRSM = 11110
sum_dataRSM = 213.39993286132812/depth_dataRSM

# Bottom Left Corner
depth_dataBLC = 11840
sum_dataBLC = 135.24696350097656/depth_dataBLC

# Bottom Right Corner
depth_dataBRC = 10323
sum_dataBRC = 115.0038833618164/depth_dataBRC


# Run 3
#Center
depth_dataC = 13258
sum_dataC = 221.0468292236328/depth_dataC

#Center Back
depth_dataCB = 9286
sum_dataCB = 115.2859878540039/depth_dataCB

#Left Side
depth_dataLS = 12511
sum_dataLS = 309.73370361328125/depth_dataLS

#Right Side
depth_dataRS = 11477
sum_dataRS = 133.8076171875/depth_dataRS

#Left Side Middle
depth_dataLSM = 11776
sum_dataLSM = 229.6862335205078/depth_dataLSM

#Right Side Middle
depth_dataRSM = 13036
sum_dataRSM = 182.31362915039062/depth_dataRSM

# Bottom Left Corner
depth_dataBLC = 12341
sum_dataBLC = 159.56446838378906/depth_dataBLC

# Bottom Right Corner
depth_dataBRC = 7716
sum_dataBRC = 54.357093811035156/depth_dataBRC


# Run 4
#Center
depth_dataC = 15400
sum_dataC = 294.2939453125/depth_dataC

#Center Back
depth_dataCB = 14515
sum_dataCB = 140.79962158203125/depth_dataCB

#Left Side
depth_dataLS = 10011
sum_dataLS = 137.32521057128906/depth_dataLS

#Right Side
depth_dataRS = 15854
sum_dataRS = 299.4512939453125/depth_dataRS

#Left Side Middle
depth_dataLSM = 12197
sum_dataLSM = 184.0587158203125/depth_dataLSM

#Right Side Middle
depth_dataRSM = 14536
sum_dataRSM = 243.7178192138672/depth_dataRSM

# Bottom Left Corner
depth_dataBLC = 7301
sum_dataBLC = 56.84340286254883/depth_dataBLC

# Bottom Right Corner
depth_dataBRC = 12377
sum_dataBRC = 72.494140625/depth_dataBRC


# Run 5
#Center
depth_dataC = 16087
sum_dataC = 237.0703582763672/depth_dataC

#Center Back
depth_dataCB = 12835
sum_dataCB = 256.6081848144531/depth_dataCB

#Left Side
depth_dataLS = 15913
sum_dataLS = 332.3143615722656/depth_dataLS

#Right Side
depth_dataRS = 8590
sum_dataRS = 119.49388885498047/depth_dataRS

#Left Side Middle
depth_dataLSM = 15063
sum_dataLSM = 250.99209594726562/depth_dataLSM

#Right Side Middle
depth_dataRSM = 10901
sum_dataRSM = 171.1293487548828/depth_dataRSM

# Bottom Left Corner
depth_dataBLC = 15558
sum_dataBLC = 307.980712890625/depth_dataBLC

# Bottom Right Corner
depth_dataBRC = 15468
sum_dataBRC = 119.8470687866211/depth_dataBRC
'''

# Run 6
#Center
depth_dataC = 17047
sum_dataC = 301.62347412109375/depth_dataC

#Center Back
depth_dataCB = 9446
sum_dataCB = 158.41204833984375/depth_dataCB

#Left Side
depth_dataLS = 13090
sum_dataLS = 155.3997039794922/depth_dataLS

#Right Side
depth_dataRS = 13222
sum_dataRS = 294.2434387207031/depth_dataRS

#Left Side Middle
depth_dataLSM = 14335
sum_dataLSM = 195.1842041015625/depth_dataLSM

#Right Side Middle
depth_dataRSM = 10860
sum_dataRSM = 221.6981658935547/depth_dataRSM

# Bottom Left Corner
depth_dataBLC = 6734
sum_dataBLC = 64.41568756103516/depth_dataBLC

# Bottom Right Corner
depth_dataBRC = 21477
sum_dataBRC = 302.760986328125/depth_dataBRC



# Define arrays
#rankDepth = np.array([1,2,5,6,3,4,8,7])
#rankSum = np.array([8,3,7,4,6,5,2,1])
pos = np.array(['Center','Center Back','Left Side','Right Side','Middle Left Side','Middle Right Side','Bottom Left Corner','Bottom Right Corner'])
vecApex = np.array([apexC,apexCB,apexLS,apexRS,apexLSM,apexRSM,apexBLC,apexBRC])
vecDepth = np.array([depth_dataC/307200,depth_dataCB/307200,depth_dataLS/307200,depth_dataRS/307200,depth_dataLSM/307200,depth_dataRSM/307200,depth_dataBLC/307200,depth_dataBRC/307200])
vecSum = np.array([sum_dataC,sum_dataCB,sum_dataLS,sum_dataRS,sum_dataLSM,sum_dataRSM,sum_dataBLC,sum_dataBRC])

# Find rank
sorted_indices = np.argsort(vecDepth)[::-1]  
rankDepth = np.empty_like(sorted_indices)
rankDepth[sorted_indices] = np.arange(1, len(vecDepth) + 1)

sorted_indices = np.argsort(vecSum)  
rankSum = np.empty_like(sorted_indices)
rankSum[sorted_indices] = np.arange(1, len(vecSum) + 1)

plot_camera(vecApex,vecDepth,vecSum,rankDepth,rankSum,pos)
