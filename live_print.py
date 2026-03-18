import cv2
import pyrealsense2 as rs
import numpy as np
import math
from insightface.app import FaceAnalysis

# Function to estimate face orientation (yaw, pitch, roll in degrees) using solvePnP
def face_orientation_degrees(frame, landmarks):
    size = frame.shape  # image size
    # 2D image points from landmarks
    image_points = np.array([
        (landmarks[4],  landmarks[5]),   # Nose tip
        (landmarks[10], landmarks[11]),  # Chin
        (landmarks[0],  landmarks[1]),   # Left eye left corner
        (landmarks[2],  landmarks[3]),   # Right eye right corner
        (landmarks[6],  landmarks[7]),   # Left mouth corner
        (landmarks[8],  landmarks[9])    # Right mouth corner
    ], dtype="double")
    # 3D model points corresponding to the landmarks (generic face model)
    model_points = np.array([
        (0.0, 0.0, 0.0),       # Nose tip
        (0.0, -330.0, -65.0),  # Chin
        (-165.0, 170.0, -135.0),   # Left eye left corner
        (165.0, 170.0, -135.0),    # Right eye right corner
        (-150.0, -150.0, -125.0),  # Left mouth corner
        (150.0, -150.0, -125.0)    # Right mouth corner
    ])
    # Camera internals (approximate focal length from image size assuming 60° FOV)
    center = (size[1] / 2, size[0] / 2)
    focal_length = center[0] / math.tan(60/2 * math.pi/180)
    camera_matrix = np.array([
        [focal_length, 0, center[0]],
        [0, focal_length, center[1]],
        [0, 0, 1]
    ], dtype="double")
    dist_coeffs = np.zeros((4,1))  # Assuming no lens distortion

    # Solve for head pose
    success, rotation_vector, translation_vector = cv2.solvePnP(
        model_points, image_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE
    )
    if not success:
        return 0.0, 0.0, 0.0  # default if solvePnP fails

    # Get rotation matrix and Euler angles from rotation vector
    rot_matrix, _ = cv2.Rodrigues(rotation_vector)
    proj_matrix = np.hstack((rot_matrix, translation_vector))
    euler_angles = cv2.decomposeProjectionMatrix(proj_matrix)[6]  # [pitch, yaw, roll]
    pitch = float(euler_angles[0])
    yaw   = float(euler_angles[1])
    roll  = float(euler_angles[2])
    # Convert to degrees in range [-180, 180]
    pitch = math.degrees(math.asin(math.sin(math.radians(pitch))))
    yaw   = math.degrees(math.asin(math.sin(math.radians(yaw))))
    roll  = -math.degrees(math.asin(math.sin(math.radians(roll))))
    return yaw, pitch, roll

# Initialize RealSense pipeline and alignment
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
profile = pipeline.start(config)
align = rs.align(rs.stream.color)
# Get color camera intrinsics for 3D deprojection
color_intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

# Initialize face detection model (InsightFace)
app = FaceAnalysis()
app.prepare(ctx_id=0, det_size=(640, 640))

print("Starting live face orientation tracking...")
try:
    while True:
        # Wait for synchronized frames (depth aligned to color)
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        # Convert frames to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # Detect faces in the color frame
        faces = app.get(color_image)
        # Prepare a top-down view image for visualization
        map_img = np.full((500, 500, 3), 255, dtype=np.uint8)
        origin_x = map_img.shape[1] // 2  # center of map width
        origin_y = map_img.shape[0] // 2  # center of map height (robot/camera position)
        cv2.circle(map_img, (origin_x, origin_y), 5, (0, 255, 0), -1)  # robot at origin (green dot)

        for face in faces:
            # Compute face center coordinates (pixels)
            x1, y1, x2, y2 = face.bbox.astype(int)
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)
            # Deproject center pixel to 3D (camera coordinates, in same units as depth, typically millimeters)
            depth_value = depth_image[center_y, center_x]
            X, Y, Z = rs.rs2_deproject_pixel_to_point(color_intrinsics, [center_x, center_y], float(depth_value))
            # Convert units from mm to cm for easier interpretation (if depth is in mm)
            X *= 0.1; Y *= 0.1; Z *= 0.1

            # Estimate face orientation angles (camera-relative yaw, pitch, roll)
            # Prepare landmarks array for solvePnP
            landmarks = [
                face.kps[0][0], face.kps[0][1],  # left eye
                face.kps[1][0], face.kps[1][1],  # right eye
                face.kps[2][0], face.kps[2][1],  # nose
                face.kps[3][0], face.kps[3][1],  # left mouth corner
                face.kps[4][0], face.kps[4][1],  # right mouth corner
                (x1 + x2) / 2.0, y2             # chin (approximate midpoint of jawline at bottom of bbox)
            ]
            yaw_deg, pitch_deg, roll_deg = face_orientation_degrees(color_image, landmarks)

            # **Compute world-relative face orientation (alpha)**
            # Bearing from camera to face (angle of face position relative to camera forward)
            bearing_deg = math.degrees(math.atan2(X, Z))  # X: horizontal offset, Z: depth forward
            # World-relative orientation alpha (0 = facing same direction as robot, ±90 = facing sideways)
            world_alpha = bearing_deg - yaw_deg
            # Normalize alpha to [-180, 180] range
            if world_alpha > 180:
                world_alpha -= 360
            elif world_alpha <= -180:
                world_alpha += 360

            # Print the name (if available) and world-relative orientation and position
            name = face.userid if hasattr(face, "userid") and face.userid is not None else "Person"
            print(name, world_alpha, X, Y, Z)

            # Draw the person's position and orientation on the top-down map
            # Convert world coordinates (X forward/right, Z forward) to map pixel coordinates
            # Here, X is left-right (positive = right), Z is forward (positive = forward)
            px = int(origin_x + X)    # 1 cm per pixel for simplicity
            py = int(origin_y - Z)    # subtract Z for forward (upwards on map)
            cv2.circle(map_img, (px, py), 5, (255, 0, 0), -1)  # mark person position (blue dot)
            # Calculate arrow end point for face orientation
            # 0° (world_alpha=0) means facing forward (up on map), 90° means facing right, -90° facing left, 180°/-180° facing back
            angle_rad = math.radians(90 - world_alpha)  # convert to radians, relative to +X-axis for drawing
            arrow_length = 50  # arrow length in pixels
            end_x = int(px + arrow_length * math.cos(angle_rad))
            end_y = int(py - arrow_length * math.sin(angle_rad))
            cv2.arrowedLine(map_img, (px, py), (end_x, end_y), (0, 0, 255), 2, tipLength=0.2)  # red arrow for face orientation

        # Display the top-down view with orientation arrow
        cv2.imshow("Top-Down View", map_img)
        # Press 'q' to exit loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    # Cleanup
    pipeline.stop()
    cv2.destroyAllWindows()
