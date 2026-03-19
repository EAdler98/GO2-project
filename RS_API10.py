import pyrealsense2 as rs
import numpy as np
import cv2 as cv
import time

# Function to capture RGB images
def capture_rgb_images(num_pictures):
    print("Capturing RGB Images")

    # Create a pipeline for RGB
    pipeline_rgb = rs.pipeline()
    config_rgb = rs.config()

    # Enable the RGB stream at 1280x720 resolution with 15 FPS
    config_rgb.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 15)

    # Start RGB streaming
    profile_rgb = pipeline_rgb.start(config_rgb)

    rgb_images = []

    try:
        for i in range(num_pictures):
            frames_rgb = pipeline_rgb.wait_for_frames()

            # Get the color frame
            color_frame = frames_rgb.get_color_frame()

            if not color_frame:
                continue

            # Convert RGB image to numpy array
            color_image = np.asanyarray(color_frame.get_data())
            rgb_images.append(color_image)

            # Save the RGB image
            rgb_filename = f"rgb_image_{i + 1}.png"
            cv.imwrite(rgb_filename, color_image)

            print(f"Saved {rgb_filename}")

    finally:
        # Stop RGB streaming
        pipeline_rgb.stop()

    return rgb_images

# Function to capture and align Depth images
def capture_depth_images(num_pictures):
    print("Capturing Aligned Depth Images")

    # Create a pipeline for Depth and align to RGB
    pipeline = rs.pipeline()
    config = rs.config()

    # Enable both RGB and Depth streams
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 15)
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 6)

    # Start the pipeline
    profile = pipeline.start(config)

    # Create an align object to align Depth to RGB
    align_to = rs.stream.color
    align = rs.align(align_to)
    
    depth_images = []
    rgb_images = []
    time.sleep(1)
    try:
        for i in range(num_pictures):
            time.sleep(1)
            frames = pipeline.wait_for_frames()

            # Align the depth frame to the color frame
            aligned_frames = align.process(frames)

            # Get aligned frames
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            rgb_images.append(color_image)
            depth_images.append(depth_image)

            # Save the aligned images
            rgb_filename = f"captured images/aligned_rgb_image_{i + 1}.png"
            depth_filename = f"captured images/aligned_depth_image_{i + 1}.png"
            cv.imwrite(rgb_filename, color_image)
            cv.imwrite(depth_filename, depth_image)

            print(f"Saved {rgb_filename} and {depth_filename}")

    finally:
        # Stop streaming
        pipeline.stop()

    return depth_images, rgb_images

# Function to calculate median depth image
def calculate_median_image(num_pictures):
    image_paths = ['captured images/aligned_depth_image_' + str(i) + '.png' for i in range(2, num_pictures + 1)]
    
    # Load the images
    images = [cv.imread(image_path, cv.IMREAD_UNCHANGED) for image_path in image_paths]

    # Stack the images along a new dimension and calculate the median along that dimension
    stacked_images = np.stack(images, axis=-1)
    median_image = np.median(stacked_images, axis=-1)

    # Convert the result to the same type as the input images
    median_image = median_image.astype(np.uint16)
    output_path = "captured images/aligned_depth_image_m.png"
    
    # Save the median image
    cv.imwrite(output_path, median_image)
    print(f"Median image saved as: {output_path}")

# Function to create a clearer depth image using median depth image
def create_clearer_depth_image():
    # Load the median depth image
    median_image_path = "captured images/aligned_depth_image_m.png"
    depth_image = cv.imread(median_image_path, cv.IMREAD_UNCHANGED)

    if depth_image is None:
        print("Error: Median depth image not found.")
        return

    # Normalize the depth image to increase contrast
    normalized_depth = cv.normalize(depth_image, None, 0, 255, cv.NORM_MINMAX, cv.CV_8U)

    # Apply a simple sharpening kernel to make the edges clearer
    kernel = np.array([[0, -1, 0], 
                       [-1, 5, -1],
                       [0, -1, 0]])
    sharpened_depth = cv.filter2D(normalized_depth, -1, kernel)

    # Optionally apply a colormap for better visualization (COLORMAP_RAINBOW or others)
    colormap_depth = cv.applyColorMap(sharpened_depth, cv.COLORMAP_JET)

    # Save the clearer depth image
    output_path = "captured images/clearer_depth_image_simple.png"
    cv.imwrite(output_path, colormap_depth)
    print(f"Clearer depth image saved as: {output_path}")

def main():
    # Capture RGB and aligned Depth images using the correct RealSense alignment
    depth_images, rgb_images = capture_depth_images(4)

    # Calculate and save the median depth image
    calculate_median_image(4)

    # Create and save a clearer depth image
    create_clearer_depth_image()

# Run the main function
if __name__ == "__main__":
    time.sleep(10)
    main()
