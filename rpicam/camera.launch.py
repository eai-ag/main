from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # Note: gscam automatically adds the appsink, so we define everything before it

    # Example segment for cropping
    # ! jpegdec ! video/x-raw,width=1280,height=720 ! videocrop left=270 right=290 ! jpegenc


    # # Full HD stream: 5MB/s
    # gst_pipeline = "v4l2src ! image/jpeg,width=1920,height=1080,framerate=31/1 ! videorate ! image/jpeg,framerate=30/1 ! tee name=t t. ! queue ! multipartmux ! tcpserversink host=0.0.0.0 port=5000 t. ! queue ! jpegparse"
    
    # Low resolution stream (using libcamerasrc, which works also for CSI cameras)
    gst_pipeline = "libcamerasrc ! image/jpeg,width=1280,height=720,framerate=60/1 ! videorate ! image/jpeg,framerate=30/1 ! tee name=t t. ! queue ! multipartmux ! tcpserversink host=0.0.0.0 port=5000 t. ! queue ! jpegparse"


    gscam_node = Node(
        package='gscam',
        executable='gscam_node',
        name='gscam_driver',
        namespace='helix',
        parameters=[{
            'gscam_config': gst_pipeline,
            'camera_name': 'camera',
            'frame_id': 'camera',
            'camera_info_url': '',
            'use_gst_timestamps': False, # ???
            'image_encoding': 'jpeg',
            'sync_sink': False, # ???
        }],
        output='screen'
    )

    return LaunchDescription([
        gscam_node,
    ])
