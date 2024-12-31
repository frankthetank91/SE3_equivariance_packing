

import zmq
import struct
import numpy as np
import cv2

# Configuration Parameters
STREAM_HOST = "localhost"  # Change if running on a different machine
STREAM_PORT = 5555
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

def main():
    # ZeroMQ Setup
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(f"tcp://{STREAM_HOST}:{STREAM_PORT}")
    socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all topics
    print(f"Connected to RGBD stream on port {STREAM_PORT}...")

    try:
        while True:
            # Blocking receive
            msg = socket.recv()  # This will block until a message is received

            # Check if message has at least header size
            header_size = struct.calcsize('IIIIII')  # 8 bytes (two unsigned integers)
            if len(msg) < header_size:
                print("Received message is too short.")
                continue

            # Unpack header
            rgb_size, depth_size,rgbd_rgb_size,rgbd_depth_size,camera_size,camera_intrinsic_size = struct.unpack('IIIIII', msg[:header_size])

            expected_size = header_size + rgb_size + depth_size + rgbd_rgb_size + rgbd_depth_size+camera_size + camera_intrinsic_size
            if len(msg) != expected_size:
                print(f"Incorrect message size. Expected {expected_size}, got {len(msg)}.")
                continue

            # Extract RGB and Depth Bytes
            rgb_bytes = msg[header_size:header_size + rgb_size]
            depth_bytes = msg[header_size + rgb_size:header_size + rgb_size + depth_size]
            rgbd_rgb_bytes = msg[header_size + rgb_size + depth_size:header_size + rgb_size + depth_size + rgbd_rgb_size]
            rgbd_depth_bytes = msg[header_size + rgb_size + depth_size + rgbd_rgb_size:header_size + rgb_size + depth_size + rgbd_rgb_size + rgbd_depth_size]
            
            camera_bytes = msg[header_size + rgb_size + depth_size + rgbd_rgb_size + rgbd_depth_size:header_size + rgb_size + depth_size + rgbd_rgb_size + rgbd_depth_size + camera_size]
            camera_intrinsic_bytes = msg[header_size + rgb_size + depth_size + rgbd_rgb_size + rgbd_depth_size + camera_size:header_size + rgb_size + depth_size + rgbd_rgb_size + rgbd_depth_size + camera_size + camera_intrinsic_size]
            
            
            # Convert Bytes to Arrays
            rgb = np.frombuffer(rgb_bytes, dtype=np.uint8).reshape((CAMERA_HEIGHT, CAMERA_WIDTH, 3))
            # apply colored image from the depth buffer
            depth = np.frombuffer(depth_bytes, dtype=np.uint8).reshape((CAMERA_HEIGHT, CAMERA_WIDTH))
            
          
            
            # get camera bytes (near,far)
            camera_near, camera_far = struct.unpack('ff', camera_bytes)
            # get camera intrinsic values
            fx, fy, cx, cy = struct.unpack('ffff', camera_intrinsic_bytes)
            
            depth = np.frombuffer(depth_bytes, dtype=np.uint8).reshape((CAMERA_HEIGHT, CAMERA_WIDTH)) # rather than 0-255, convert this to colormap
            # apply near, far values to the depth image to get real depth (meric depth)
            depth = depth.astype(np.float32) * (camera_far - camera_near) / 255.0 + camera_near


            # Convert Bytes to Arrays
            rgbd_rgb = np.frombuffer(rgbd_rgb_bytes, dtype=np.uint8).reshape((CAMERA_HEIGHT, CAMERA_WIDTH, 3))
            # apply colored image from the depth buffer
            rgbd_depth = np.frombuffer(rgbd_depth_bytes, dtype=np.uint16).reshape((CAMERA_HEIGHT, CAMERA_WIDTH))
            
            rgbd_depth = rgbd_depth.astype(np.float32) * (100. - camera_near) / 65535.0 + camera_near
            # colorjet
            rgbd_depth = cv2.applyColorMap(cv2.convertScaleAbs(rgbd_depth, alpha=0.03), cv2.COLORMAP_JET)
            
            # apply near, far values to the depth image to get real depth (meric depth)
            # rgbd_depth = rgbd_depth.astype(np.float32) * (1.0 - camera_near) / 255.0 + camera_near

            # change rgb to bgr
            rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            rgbd_rgb = cv2.cvtColor(rgbd_rgb, cv2.COLOR_RGB2BGR)


            # Display Images
            cv2.imshow("RGB Image", rgb)
            cv2.imshow("Depth Image", depth)
            cv2.imshow("RGBD RGB Image", rgbd_rgb)
            cv2.imshow("RGBD Depth Image", rgbd_depth)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Receiver stopped.")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        cv2.destroyAllWindows()
        socket.close()
        context.term()

if __name__ == "__main__":
    main()
