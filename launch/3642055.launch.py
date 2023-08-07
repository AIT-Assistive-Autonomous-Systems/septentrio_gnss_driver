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

    default_file_name = '3642055.yaml'
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
                ('/attcoveuler', '/septentrio3642055/attcoveuler'),
                ('/atteuler', '/septentrio3642055/atteuler'),
                ('/basevectorcart', '/septentrio3642055/basevectorcart'),
                ('/basevectorgeod', '/septentrio3642055/basevectorgeod'),
                ('/gpgga', '/septentrio3642055/gpgga'),
                ('/gpgsa', '/septentrio3642055/gpgsa'),
                ('/gprmc', '/septentrio3642055/gprmc'),
                ('/gpsfix', '/septentrio3642055/gpsfix'),
                ('/gpst', '/septentrio3642055/gpst'),
                ('/measepoch', '/septentrio3642055/measepoch'),
                ('/navsatfix', '/septentrio3642055/navsatfix'),
                ('/poscovcartesian', '/septentrio3642055/poscovcartesian'),
                ('/poscovgeodetic', '/septentrio3642055/poscovgeodetic'),
                ('/pose', '/septentrio3642055/pose'),
                ('/pvtcartesian', '/septentrio3642055/pvtcartesian'),
                ('/pvtgeodetic', '/septentrio3642055/pvtgeodetic'),
                ('/twist_gnss', '/septentrio3642055/twist_gnss'),
                ('/velcovgeodetic', '/septentrio3642055/velcovgeodetic')
            ],
            emulate_tty=True,
            sigterm_timeout = '10',
            parameters=[LaunchConfiguration(name_arg_file_path)])

    return launch.LaunchDescription([arg_file_name, arg_file_path, node, tf_imu, tf_gnss, tf_vsm, tf_aux1])
