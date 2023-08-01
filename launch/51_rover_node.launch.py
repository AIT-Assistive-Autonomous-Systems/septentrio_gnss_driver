import os
import launch
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from ament_index_python.packages import get_package_share_directory

os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time}: [{name}] [{severity}]\t{message}'
# Verbose log:
#os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time}: [{name}] [{severity}]\t{message} ({function_name}() at {file_name}:{line_number})'

# Start as component:

def generate_launch_description():

    tf_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = "0 0 0 0 0 0 base_link imu".split(' ')
    )

    tf_gnss = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = "0 0 0 0 0 0 imu gnss".split(' ')
    )

    tf_vsm = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = "0 0 0 0 0 0 imu vsm".split(' ')
    )

    tf_aux1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = "0 0 0 0 0 0 imu aux1".split(' ')
    )

    default_file_name = '51.yaml'
    name_arg_file_name = "file_name"
    arg_file_name = DeclareLaunchArgument(name_arg_file_name,
                                          default_value=TextSubstitution(text=str(default_file_name)))
    name_arg_file_path = 'path_to_config'
    arg_file_path = DeclareLaunchArgument(name_arg_file_path,
                                          default_value=[get_package_share_directory('septentrio_gnss_driver'), '/config/', LaunchConfiguration(name_arg_file_name)])

    node = Node(
            package='septentrio_gnss_driver',
            executable='septentrio_gnss_driver_node',
            name='septentrio_gnss_driver',
            remappings=[
                ('/attcoveuler', '/septentrio51/attcoveuler'),
                ('/atteuler', '/septentrio51/atteuler'),
                ('/basevectorcart', '/septentrio51/basevectorcart'),
                ('/basevectorgeod', '/septentrio51/basevectorgeod'),
                ('/gpgga', '/septentrio51/gpgga'),
                ('/gpgsa', '/septentrio51/gpgsa'),
                ('/gprmc', '/septentrio51/gprmc'),
                ('/gpsfix', '/septentrio51/gpsfix'),
                ('/gpst', '/septentrio51/gpst'),
                ('/measepoch', '/septentrio51/measepoch'),
                ('/navsatfix', '/septentrio51/navsatfix'),
                ('/poscovcartesian', '/septentrio51/poscovcartesian'),
                ('/poscovgeodetic', '/septentrio51/poscovgeodetic'),
                ('/pose', '/septentrio51/pose'),
                ('/pvtcartesian', '/septentrio51/pvtcartesian'),
                ('/pvtgeodetic', '/septentrio51/pvtgeodetic'),
                ('/twist_gnss', '/septentrio51/twist_gnss'),
                ('/velcovgeodetic', '/septentrio51/velcovgeodetic')
            ],
            # namespace='septentrio_51',
            emulate_tty=True,
            sigterm_timeout = '10',
            parameters=[LaunchConfiguration(name_arg_file_path)])

    return launch.LaunchDescription([arg_file_name, arg_file_path, node, tf_imu, tf_gnss, tf_vsm, tf_aux1])
