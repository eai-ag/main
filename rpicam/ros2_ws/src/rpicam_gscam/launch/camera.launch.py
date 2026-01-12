from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # GStreamer pipeline with tee for dual output
    # Matches docker-compose pipeline with videocrop settings
    # Note: gscam automatically adds the appsink, so we define everything before it
    gst_pipeline = (
        'libcamerasrc ! '
        'video/x-raw,width=1280,height=720,framerate=30/1 ! '
        'videocrop left=270 right=290 ! '
        'videoconvert ! '
        'tee name=t '
        't. ! queue ! jpegenc ! multipartmux ! tcpserversink host=0.0.0.0 port=5000'
        't. ! queue ! videoconvert ! video/x-raw,format=RGB '
    )


    gst_pipeline = "v4l2src ! video/x-raw,width=1280,height=720 ! videocrop left=270 right=290 ! tee name=t t. ! queue ! jpegenc ! multipartmux ! tcpserversink host=0.0.0.0 port=5000 t. ! queue ! videoconvert ! video/x-raw,format=RGB"
    

    gscam_node = Node(
        package='gscam',
        executable='gscam_node',
        name='gscam_driver',
        parameters=[{
            'gscam_config': gst_pipeline,
            'camera_name': 'camera',
            'frame_id': 'camera',
            'camera_info_url': '',
            'use_gst_timestamps': False,
            'image_encoding': 'rgb8',
            'sync_sink': True,
        }],
        remappings=[
            ('camera/image_raw', 'image_raw'),
            ('camera/camera_info', 'camera_info'),
        ],
        output='screen'
    )

    return LaunchDescription([
        gscam_node,
    ])
