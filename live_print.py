# live_print.py

import pyrealsense2 as rs
import numpy as np
import cv2
from insightface.app import FaceAnalysis
import math
import matplotlib.pyplot as plt
from recognize_persons import face_orientation_degrees


def main():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

    # Start streaming
    pipeline.start(config)

    # Create an align object to align depth frames to color frames
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Initialize insightface for face analysis
    app = FaceAnalysis()
    app.prepare(ctx_id=0, det_size=(1280, 1280))

    # Set up live plotting for a top-down view (using x and z axes)
    plt.ion()  # interactive mode on
    fig, ax = plt.subplots(figsize=(6,6))
    
    try:
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # Detect faces using insightface
            faces = app.get(color_image)
            if faces:
                # Use the first detected face
                face = faces[0]
                # Determine the center of the face bounding box
                x1, y1, x2, y2 = map(int, face.bbox)
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)

                # Get the depth value at the center pixel (in millimeters)
                depth_value = depth_image[center_y, center_x]
                # Use the active profile to get camera intrinsics
                profile = pipeline.get_active_profile()
                intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
                # Deproject the pixel to a 3D point (in mm)
                point3d = rs.rs2_deproject_pixel_to_point(intrinsics, [center_x, center_y], depth_value)
                # Convert from millimeters to centimeters
                x3d = point3d[0] * 0.1  # x coordinate in cm
                y3d = point3d[1] * 0.1  # y coordinate (vertical) in cm
                z3d = point3d[2] * 0.1  # z coordinate (forward) in cm

                # Build landmarks for face orientation estimation.
                # Here we assume face.kps contains five keypoints.
                landmarks = [
                    face.kps[0][0], face.kps[0][1],  # left eye
                    face.kps[1][0], face.kps[1][1],  # right eye
                    face.kps[2][0], face.kps[2][1],  # nose
                    face.kps[3][0], face.kps[3][1],  # left mouth corner
                    face.kps[4][0], face.kps[4][1],  # right mouth corner
                    (x1 + x2) / 2, y2               # approximated chin (bottom-center of bbox)
                ]
                yaw, pitch, roll = face_orientation_degrees(color_image, landmarks)
                # We'll treat the yaw as the face orientation angle (alpha)
                alpha = yaw-90

                print(f"x: {x3d:.2f} cm, y: {y3d:.2f} cm, z: {z3d:.2f} cm, alpha: {alpha:.2f}°")
                
                # Update the live top-down plot.
                # For a top-down view, we consider the camera at (0,0).
                # We'll plot the face using x3d (horizontal) and z3d (forward).
                ax.cla()  # clear the axes
                # Plot the camera position at the origin.
                ax.plot(0, 0, 'bs', markersize=10, label='Camera (Origin)')
                # Plot the face position
                ax.plot(x3d, z3d, 'ro', markersize=10, label='Face Position')
                # Draw an arrow at the face position indicating the face orientation.
                # Convert alpha to radians.
                alpha_rad = math.radians(alpha)
                arrow_length = 20  # length of the orientation arrow in cm
                ax.arrow(x3d, z3d, arrow_length * math.cos(alpha_rad), arrow_length * math.sin(alpha_rad),
                         head_width=5, head_length=5, fc='r', ec='r', label='Face Orientation')
                # Set axis labels and title
                ax.set_xlabel("X (cm)")
                ax.set_ylabel("Z (cm) [Forward]")
                ax.set_title("Top-Down View: Face Position & Orientation")
                # Set some limits (adjust as needed for your scene)
                ax.set_xlim(-100, 100)
                ax.set_ylim(0, 200)
                ax.legend()
                plt.draw()
                plt.pause(0.001)
            
            # Optionally, display the color image (if your OpenCV build supports it)
            cv2.imshow("Live Video", color_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        plt.ioff()
        plt.show()

if __name__ == "__main__":
    main()
