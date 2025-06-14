Simulate objects falling in Pybullet, save position and orientation of objects inside of the box. 
Use positions and orientations to create a scene in blender of the box and objects.
Generate 10 random camera positions and one camera view over the origin (center of the box), save in Text File.
Run main code to render RGB, Depth, and Segmentation Masks in Blender.


1. Run "python box_objects.py" in terminal
This code runs the Pybullet simulation where the objects fall into the box.
The code chooses N random objects from a specified folder.
The file "pybullet_poses.npz" should be created after running the code.

2. Run "python camera_positions.py" in terminal
This code generates 10 random camera positions and one camera position above the center of the box. Orientation towards (0,0,0.15).
The files "camera_positions.txt", "dome_lat_lon.txt", and "ring_radius_height.txt" should be created after running the code. The files "dome_lat_lon.txt" and "ring_radius_height.txt" should be empty.

OR

Run "python dome_camera_positions.py" in terminal
This code generates camera positions based on a set radius and minimum/maximum/step values for latitude and longitude with one camera position above the center of the box. Orientation towards (0,0,0.15).
The files "camera_positions.txt", "dome_lat_lon.txt", and "ring_radius_height.txt" should be created after running the code. The file "ring_radius_height.txt" should be empty.

OR 

Run "python ring_camera_positions.py" in terminal
This code generates N camera positions around a circle with a specified radius and height and one camera position above the center of the box. Orientation towards (0,0,0.15).
The files "camera_positions.txt", "dome_lat_lon.txt", and "ring_radius_height.txt" should be created after running the code. The file "dome_lat_lon.txt" should be empty.

2. Run "blenderproc run create_scene.py" in terminal
This code loads the object files into Blender and saves the scene.
The files "my_scene.blend" and "scene.pt" should be created after running the code.

4. Run "blenderproc run my_main.py camera_positions.txt my_scene.blend output" in terminal
This code renders the RGB, Depth, and Segmentation Masks and saves .hdf5 files.
The folder "output" should be created with 10 .hdf5 files inside.

5. Run "blenderproc vis hdf5 output/0.hdf5" in terminal
Run the above command to view the contents of the .hdf5 files.
