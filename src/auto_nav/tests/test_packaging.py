import ast
import importlib
import sys
import types
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _load_console_scripts():
    setup_py = PACKAGE_ROOT / 'setup.py'
    tree = ast.parse(setup_py.read_text(encoding='utf-8'))

    for node in tree.body:
        if not isinstance(node, ast.Expr) or not isinstance(node.value, ast.Call):
            continue
        call = node.value
        if getattr(call.func, 'id', None) != 'setup':
            continue
        for keyword in call.keywords:
            if keyword.arg != 'entry_points':
                continue
            return ast.literal_eval(keyword.value)['console_scripts']

    raise AssertionError('setup.py is missing console_scripts entry points')


def _install_ros_import_stubs():
    def add_module(name):
        module = types.ModuleType(name)
        sys.modules[name] = module
        return module

    rclpy = add_module('rclpy')
    rclpy.init = lambda *args, **kwargs: None
    rclpy.spin = lambda *args, **kwargs: None
    rclpy.shutdown = lambda *args, **kwargs: None

    rclpy_action = add_module('rclpy.action')
    rclpy_action.ActionClient = type('ActionClient', (), {})
    rclpy.action = rclpy_action

    rclpy_duration = add_module('rclpy.duration')
    rclpy_duration.Duration = type('Duration', (), {})
    rclpy.duration = rclpy_duration

    rclpy_node = add_module('rclpy.node')
    rclpy_node.Node = type('Node', (), {})
    rclpy.node = rclpy_node

    rclpy_time = add_module('rclpy.time')
    rclpy_time.Time = type('Time', (), {})
    rclpy.time = rclpy_time

    action_msgs = add_module('action_msgs')
    action_msgs_msg = add_module('action_msgs.msg')
    action_msgs_msg.GoalStatus = type(
        'GoalStatus',
        (),
        {
            'STATUS_SUCCEEDED': 4,
            'STATUS_CANCELED': 5,
        },
    )
    action_msgs.msg = action_msgs_msg

    geometry_msgs = add_module('geometry_msgs')
    geometry_msgs_msg = add_module('geometry_msgs.msg')
    geometry_msgs_msg.Point = type('Point', (), {})
    geometry_msgs_msg.PoseStamped = type('PoseStamped', (), {})
    geometry_msgs_msg.Twist = type('Twist', (), {})
    geometry_msgs.msg = geometry_msgs_msg

    nav2_msgs = add_module('nav2_msgs')
    nav2_msgs_action = add_module('nav2_msgs.action')
    nav2_msgs_action.NavigateToPose = type(
        'NavigateToPose',
        (),
        {'Goal': type('Goal', (), {})},
    )
    nav2_msgs.action = nav2_msgs_action
    nav2_msgs_srv = add_module('nav2_msgs.srv')
    nav2_msgs_srv.SaveMap = type(
        'SaveMap',
        (),
        {'Request': type('Request', (), {})},
    )
    nav2_msgs.srv = nav2_msgs_srv

    nav_msgs = add_module('nav_msgs')
    nav_msgs_msg = add_module('nav_msgs.msg')
    nav_msgs_msg.Odometry = type('Odometry', (), {})
    nav_msgs_msg.OccupancyGrid = type('OccupancyGrid', (), {})
    nav_msgs.msg = nav_msgs_msg

    sensor_msgs = add_module('sensor_msgs')
    sensor_msgs_msg = add_module('sensor_msgs.msg')
    sensor_msgs_msg.LaserScan = type('LaserScan', (), {})
    sensor_msgs.msg = sensor_msgs_msg

    std_msgs = add_module('std_msgs')
    std_msgs_msg = add_module('std_msgs.msg')
    std_msgs_msg.String = type('String', (), {})
    std_msgs.msg = std_msgs_msg

    tf2_ros = add_module('tf2_ros')
    tf2_ros.Buffer = type('Buffer', (), {})
    tf2_ros.TransformListener = type('TransformListener', (), {})
    tf2_ros.TransformException = Exception

    visualization_msgs = add_module('visualization_msgs')
    visualization_msgs_msg = add_module('visualization_msgs.msg')
    visualization_msgs_msg.Marker = type('Marker', (), {})
    visualization_msgs_msg.MarkerArray = type('MarkerArray', (), {})
    visualization_msgs.msg = visualization_msgs_msg


def test_console_scripts_keep_ros_names_and_resolve_to_navigation_package():
    console_scripts = _load_console_scripts()

    assert (
        'auto_nav_command_node = navigation.command_node:main' in console_scripts
    )
    assert (
        'auto_nav_navigation_node = navigation.navigation_node:main'
        in console_scripts
    )

    _install_ros_import_stubs()
    command_module = importlib.import_module('navigation.command_node')
    navigation_module = importlib.import_module('navigation.navigation_node')

    assert callable(command_module.main)
    assert callable(navigation_module.main)


def test_navigation_launch_still_uses_existing_ros_executable_names():
    launch_file = PACKAGE_ROOT / 'launch' / 'navigation.launch.py'
    contents = launch_file.read_text(encoding='utf-8')

    assert "executable='auto_nav_command_node'" in contents
    assert "executable='auto_nav_navigation_node'" in contents


def test_exploration_launch_is_packaged_with_map_saver_and_exploration_mode():
    launch_file = PACKAGE_ROOT / 'launch' / 'exploration.launch.py'
    contents = launch_file.read_text(encoding='utf-8')

    assert "executable='auto_nav_command_node'" in contents or 'executable="auto_nav_command_node"' in contents
    assert "executable='auto_nav_navigation_node'" in contents or 'executable="auto_nav_navigation_node"' in contents
    assert '"mode": "exploration"' in contents or "'mode': 'exploration'" in contents
    assert 'map_saver_server' in contents


def test_exploration_rviz_config_is_packaged_with_frontier_marker_topic():
    rviz_file = PACKAGE_ROOT / 'config' / 'exploration.rviz'
    contents = rviz_file.read_text(encoding='utf-8')

    assert '/auto_nav/frontier_markers' in contents
    assert '/map' in contents
    assert '/ldlidar_node/scan' in contents
