import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2 as cv
import time
import sys
import json
import math

def rotate_points2(points, alpha_degrees):
    # convert degrees to radian
    alpha_rad = np.radians(alpha_degrees)
    
    # the rotate matrix
    rotation_matrix = np.array([
        [1, 0, 0],
        [0, np.cos(alpha_rad), -np.sin(alpha_rad)],
        [0, np.sin(alpha_rad), np.cos(alpha_rad)]
    ])
    
    # rotate the points
    rotated_points = np.dot(points, rotation_matrix.T)
    
    return rotated_points

import numpy as np

def rotate_points(points, alpha_degrees, beta_degrees):
    # convert degrees to radians
    alpha_rad = np.radians(alpha_degrees)
    beta_rad = np.radians(beta_degrees)
    
    # rotation matrix around X axis (alpha)
    rotation_matrix_x = np.array([
        [1, 0, 0],
        [0, np.cos(alpha_rad), -np.sin(alpha_rad)],
        [0, np.sin(alpha_rad), np.cos(alpha_rad)]
    ])
    
    # rotation matrix around Z axis (beta)
    rotation_matrix_z = np.array([
        [np.cos(beta_rad), -np.sin(beta_rad), 0],
        [np.sin(beta_rad), np.cos(beta_rad), 0],
        [0, 0, 1]
    ])
    
    # combined rotation matrix (Z * X) - first apply X, then Z
    combined_rotation_matrix = np.dot(rotation_matrix_z, rotation_matrix_x)
    
    # rotate the points
    rotated_points = np.dot(points, combined_rotation_matrix.T)
    
    return rotated_points


def save_intrinsics(file_path):
    # Create a pipeline
    pipeline = rs.pipeline()
    config = rs.config()

    # Enable the RGB stream
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

    # Start streaming
    profile = pipeline.start(config)
    
    try:
        # Get the intrinsics of the color stream
        intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

        # Convert intrinsics to a dictionary
        intrinsics_dict = {
            'width': intrinsics.width,
            'height': intrinsics.height,
            'ppx': intrinsics.ppx,
            'ppy': intrinsics.ppy,
            'fx': intrinsics.fx,
            'fy': intrinsics.fy,
            'model': intrinsics.model.name,  # Convert the distortion model to its name (string)
            'coeffs': intrinsics.coeffs
        }

        # Save intrinsics to a file
        with open(file_path, 'w') as file:
            json.dump(intrinsics_dict, file, indent=4)
        
        print(f"Intrinsics saved to {file_path}")

    finally:
        # Stop streaming
        pipeline.stop()





def load_intrinsics(file_path):
    with open(file_path, 'r') as file:
        intrinsics_dict = json.load(file)

    # Convert dictionary back to intrinsics
    intrinsics = rs.intrinsics()
    intrinsics.width = intrinsics_dict['width']
    intrinsics.height = intrinsics_dict['height']
    intrinsics.ppx = intrinsics_dict['ppx']
    intrinsics.ppy = intrinsics_dict['ppy']
    intrinsics.fx = intrinsics_dict['fx']
    intrinsics.fy = intrinsics_dict['fy']

    # Convert the model name back to the appropriate distortion type
    distortion_model_name = intrinsics_dict['model']
    if distortion_model_name == "none":
        intrinsics.model = rs.distortion.none
    elif distortion_model_name == "modified_brown_conrady":
        intrinsics.model = rs.distortion.modified_brown_conrady
    elif distortion_model_name == "inverse_brown_conrady":
        intrinsics.model = rs.distortion.inverse_brown_conrady
    elif distortion_model_name == "brown_conrady":
        intrinsics.model = rs.distortion.brown_conrady
    elif distortion_model_name == "ftheta":
        intrinsics.model = rs.distortion.ftheta
    elif distortion_model_name == "kan_bradley":
        intrinsics.model = rs.distortion.kan_bradley
    else:
        raise ValueError(f"Unknown distortion model: {distortion_model_name}")
    
    intrinsics.coeffs = intrinsics_dict['coeffs']

    return intrinsics






def create_images(no_pictures):
    print("Capturing Aligned RGB and Depth Images")

    # Create a pipeline
    pipeline = rs.pipeline()
    config = rs.config()


    # # Enable the RGB stream at 1280x720
    # config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # # Enable the Depth stream at 640x480
    # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    
    # Enable the RGB stream at 1280x720
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

    # Enable the Depth stream at 1280x720
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

    # Start streaming
    profile = pipeline.start(config)

    # Create an align object
    # rs.stream.color indicates that we want to align depth to color
    align_to = rs.stream.color
    align = rs.align(align_to)

    try:
        # Capture 3 RGB and aligned Depth images
        for i in range(no_pictures):
            frames = pipeline.wait_for_frames()

            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

            # Get the aligned frames
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # Save the images
            rgb_filename = f"aligned_rgb_image_{i + 1}.png"
            depth_filename = f"aligned_depth_image_{i + 1}.png"
            cv.imwrite(rgb_filename, color_image)
            cv.imwrite(depth_filename, depth_image)

            print(f"Saved {rgb_filename} and {depth_filename}")

        # Wait for 1 second
        time.sleep(1)

    finally:
        # Stop streaming
        pipeline.stop()
    calculate_median_image(no_pictures)


def calculate_median_image(no_pictures):
    image_paths=['captured images/aligned_depth_image_'+str(i)+'.png' for i in range(1,no_pictures+1)]
    # Load the images
    images = [cv.imread(image_path, cv.IMREAD_UNCHANGED) for image_path in image_paths]

    # Stack the images along a new dimension and calculate the median along that dimension
    stacked_images = np.stack(images, axis=-1)
    median_image = np.median(stacked_images, axis=-1)

    # Convert the result to the same type as the input images
    median_image = median_image.astype(np.uint16)
    output_path="aligned_depth_image_m.png"
    # Save the median image
    cv.imwrite(output_path, median_image)
    print(f"Median image saved as: {output_path}")







# Your existing functions...
def get_XYZ_from_pictures_with_file(pixel_x, pixel_y):

    intrinsics_file = 'intrinsics_file'
    # save_intrinsics(intrinsics_file)
    # Load the RGB and Depth images
    # rgb_image = cv.imread(path_to_rgb)
    depth_image = cv.imread('captured images/aligned_depth_image_m.png', cv.IMREAD_UNCHANGED)
    
    # Load intrinsics from file
    intrinsics = load_intrinsics(intrinsics_file)

    # Get the corresponding depth value
    depth_value = depth_image[pixel_y, pixel_x]  # Depth is in millimeters
    
    # Use the deprojection function
    x, y, z = rs.rs2_deproject_pixel_to_point(intrinsics, [pixel_x, pixel_y], depth_value)
    
    # Convert to centimeters
    x *= 0.1
    y *= 0.1
    z *= 0.1

    y  = -0.9665*y + 38.32
    XYZ = rotate_points([x,z,y], 30, 0.74)
    
    return XYZ[0], XYZ[1], XYZ[2]






if __name__ == '__main__':

    time.sleep(10)
    create_images(4)
