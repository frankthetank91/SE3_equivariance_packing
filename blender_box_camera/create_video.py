import cv2
import os
from natsort import natsorted  # ensures proper frame order

input_folder = "renders"
output_path = "output_video.mp4"
fps = 30

# Load image files
images = [f for f in os.listdir(input_folder) if f.endswith(".png")]
images = natsorted(images)  # sort like frame_0001, frame_0002, etc.

# Read first frame to get size
first_frame = cv2.imread(os.path.join(input_folder, images[0]))
height, width, _ = first_frame.shape

# Create video writer
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Compatible with Mac
video_writer = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

# Write each frame
for img_name in images:
    frame = cv2.imread(os.path.join(input_folder, img_name))
    video_writer.write(frame)

video_writer.release()
print(f"✅ Video saved to: {output_path}")
