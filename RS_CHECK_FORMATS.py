import pyrealsense2 as rs

# Create a pipeline
pipeline = rs.pipeline()
config = rs.config()

# Get the device once it's connected
pipeline_profile = config.resolve(pipeline)
device = pipeline_profile.get_device()

# List supported streams and resolutions
for sensor in device.sensors:
    print(f"Sensor: {sensor.get_info(rs.camera_info.name)}")
    for stream_profile in sensor.profiles:
        if stream_profile.is_video_stream_profile():
            video_profile = stream_profile.as_video_stream_profile()
            print(f"Stream: {video_profile.stream_name()} "
                  f"Resolution: {video_profile.width()}x{video_profile.height()} "
                  f"Format: {video_profile.format()} "
                  f"FPS: {video_profile.fps()}")
