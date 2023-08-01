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

    default_file_name = '55.yaml'
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
                ('/attcoveuler', '/septentrio55/attcoveuler'),
                ('/atteuler', '/septentrio55/atteuler'),
                ('/basevectorcart', '/septentrio55/basevectorcart'),
                ('/basevectorgeod', '/septentrio55/basevectorgeod'),
                ('/gpgga', '/septentrio55/gpgga'),
                ('/gpgsa', '/septentrio55/gpgsa'),
                ('/gprmc', '/septentrio55/gprmc'),
                ('/gpsfix', '/septentrio55/gpsfix'),
                ('/gpst', '/septentrio55/gpst'),
                ('/measepoch', '/septentrio55/measepoch'),
                ('/navsatfix', '/septentrio55/navsatfix'),
                ('/poscovcartesian', '/septentrio55/poscovcartesian'),
                ('/poscovgeodetic', '/septentrio55/poscovgeodetic'),
                ('/pose', '/septentrio55/pose'),
                ('/pvtcartesian', '/septentrio55/pvtcartesian'),
                ('/pvtgeodetic', '/septentrio55/pvtgeodetic'),
                ('/twist_gnss', '/septentrio55/twist_gnss'),
                ('/velcovgeodetic', '/septentrio55/velcovgeodetic')
            ],
            # namespace='septentrio_51',
            emulate_tty=True,
            sigterm_timeout = '10',
            parameters=[LaunchConfiguration(name_arg_file_path)])

    return launch.LaunchDescription([arg_file_name, arg_file_path, node, tf_imu, tf_gnss, tf_vsm, tf_aux1])
