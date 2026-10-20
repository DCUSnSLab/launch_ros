# Copyright 2026 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for dump-params utilities and dry-run collection."""

import array

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import EmitEvent
from launch.actions import GroupAction
from launch.events import Shutdown

from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LifecycleNode
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import SetParameter
from launch_ros.actions import SetParametersFromFile
from launch_ros.descriptions import ComposableNode
from launch_ros.descriptions import Parameter
from launch_ros.utilities.dump_params import attach_dump_params_collector
from launch_ros.utilities.dump_params import DumpParamsCollector
from launch_ros.utilities.dump_params import DumpParamsError
from launch_ros.utilities.dump_params import resolve_composable_node_fqn
from launch_ros.utilities.dump_params import resolve_node_from_ros_args

import pytest
import yaml


def test_resolve_node_name_first_rule_wins():
    # rcl uses the first matching __node/__ns rule, so ros_arguments placed before the
    # rules Node generates win, and remappings placed after them do not.
    assert resolve_node_from_ros_args([
        '--ros-args', '-r', '__node:=first', '-r', '__node:=amcl', '-r', '__ns:=/nav',
    ])[0] == '/nav/first'
    assert resolve_node_from_ros_args([
        '--ros-args', '-r', '__ns:=/nav', '-r', 'amcl:__node:=other', '-r', '__node:=amcl',
    ])[0] == '/nav/amcl'
    assert resolve_node_from_ros_args(['--ros-args', '-r', '__node:=amcl'])[0] == '/amcl'
    assert resolve_node_from_ros_args(['-r', '__node:=not_ros_args']) == (None, {})
    assert resolve_composable_node_fqn(['__node:=renamed'], 'comp', '/ns') == '/ns/renamed'
    assert resolve_composable_node_fqn([], 'comp', '') == '/comp'
    assert resolve_composable_node_fqn([], '', '/ns') is None


def test_resolve_parameters_by_node_key_order(tmp_path):
    # rcl groups overrides by node key in order of first appearance; a key that
    # appears first keeps its position even when later sources update it.
    params_file = tmp_path / 'params.yaml'
    params_file.write_text(
        '/**:\n'
        '  ros__parameters:\n'
        '    a: wild\n'
        '/ns/node:\n'
        '  ros__parameters:\n'
        '    a: exact\n'
        '    b: exact\n'
        'ns:\n'
        '  node:\n'
        '    ros__parameters:\n'
        '      nested: {inner: 1}\n'
        '/**:\n'
        '  ros__parameters:\n'
        '    b: wild_again\n'
        '    c: wild_again\n'
        '/ns/*:\n'
        '  ros__parameters:\n'
        '    c: star\n'
        '/other:\n'
        '  ros__parameters:\n'
        '    d: other\n'
    )
    fqn, params = resolve_node_from_ros_args([
        'positional', '--ros-args', '-r', '__node:=node', '-r', '__ns:=/ns',
        '--params-file', str(params_file), '-p', 'a:=from_rule', '-p', 'node:e:=prefixed',
        '-p', 'other:f:=1', '--', '-p', 'g:=not_ros_args',
    ])
    assert fqn == '/ns/node'
    # The 'node:' prefix is matched as '/node', not as the node's base name.
    assert params == {'a': 'exact', 'b': 'exact', 'c': 'star', 'nested.inner': 1}


def test_resolve_parameters_rcl_scalar_types(tmp_path):
    params_file = tmp_path / 'params.yaml'
    params_file.write_text(
        '/**:\n'
        '  ros__parameters:\n'
        '    sci: 1e3\n'
        '    hex: 0x10\n'
        '    oct: 010\n'
        '    leading_zero: 08\n'
        '    yes_short: y\n'
        '    off: off\n'
        '    inf: .inf\n'
        "    quoted: '5'\n"
        '    tagged: !!str 7\n'
        '    text: abc\n'
        '    ints: [1, -2]\n'
        "    strs: ['1', b]\n"
    )
    _, params = resolve_node_from_ros_args([
        '--ros-args', '-r', '__node:=n', '--params-file', str(params_file),
        '-p', 'rule:=0x1F', '-p', 'rule_list:=[1.5, 2.0]',
    ])
    assert params == {
        'sci': 1000.0, 'hex': 16, 'oct': 8, 'leading_zero': 8.0, 'yes_short': True,
        'off': False, 'inf': float('inf'), 'quoted': '5', 'tagged': '7', 'text': 'abc',
        'ints': [1, -2], 'strs': ['1', 'b'], 'rule': 31, 'rule_list': [1.5, 2.0],
    }
    for name in ('sci', 'leading_zero'):
        assert type(params[name]) is float


def test_dumps_quotes_strings_rcl_would_retype(tmp_path):
    values = {'a': 'y', 'b': '1e3', 'c': '010', 'd': 'text', 'e': 1, 'f': ['on', 'x']}
    collector = DumpParamsCollector()
    collector.add_node('/n', values)
    dumped = collector.dumps()
    assert 'd: text' in dumped
    params_file = tmp_path / 'dump.yaml'
    params_file.write_text(dumped)
    assert resolve_node_from_ros_args([
        '--ros-args', '-r', '__node:=n', '--params-file', str(params_file)]) == ('/n', values)


def test_collector_coalesce_and_diverge():
    collector = DumpParamsCollector()
    collector.add_node('/a', {'x': 1})
    collector.add_node('/a', {'x': 1})  # identical → coalesce
    with pytest.raises(DumpParamsError):
        collector.add_node('/a', {'x': 2})


def test_dump_params_node_set_parameter_and_namespace():
    collector = DumpParamsCollector()
    ls = LaunchService(noninteractive=True)
    attach_dump_params_collector(ls.context, collector)

    ld = LaunchDescription([
        GroupAction([
            PushRosNamespace('nav'),
            SetParameter(name='use_sim_time', value=True),
            Node(
                package='demo_nodes_cpp',
                executable='talker',
                name='amcl',
                parameters=[{
                    'max_particles': 2000,
                    'robot_model_type': 'differential',
                }],
            ),
        ]),
        EmitEvent(event=Shutdown(reason='test')),
    ])
    ls.include_launch_description(ld)
    assert ls.run() == 0

    dumped = collector.to_yaml_dict()
    assert '/nav/amcl' in dumped
    params = dumped['/nav/amcl']['ros__parameters']
    assert params['use_sim_time'] is True
    assert params['max_particles'] == 2000
    assert params['robot_model_type'] == 'differential'


def _dump_params(actions):
    collector = DumpParamsCollector()
    ls = LaunchService(noninteractive=True)
    attach_dump_params_collector(ls.context, collector)
    ls.include_launch_description(LaunchDescription([
        *actions,
        EmitEvent(event=Shutdown(reason='test')),
    ]))
    assert ls.run() == 0
    return yaml.safe_load(collector.dumps())


def test_dump_params_same_name_later_source_wins():
    # Runtime argv order (not launch-file order): ros_arguments, then
    # SetParameter, then parameters=[...]. Later entry wins, matching rcl.
    dumped = _dump_params([
        SetParameter(name='from_global_and_ros', value='from_global'),
        SetParameter(name='from_all_three', value='from_global'),
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker',
            namespace='/',
            parameters=[{
                'from_all_three': 'from_parameters',
                'from_parameters_and_ros': 'from_parameters',
                'only_parameters': 1,
            }],
            ros_arguments=[
                '-p', 'from_global_and_ros:=from_ros',
                '-p', 'from_all_three:=from_ros',
                '-p', 'from_parameters_and_ros:=from_ros',
                '-p', 'only_ros:=2',
            ],
        ),
    ])
    params = dumped['/talker']['ros__parameters']
    assert params['from_global_and_ros'] == 'from_global'
    assert params['from_all_three'] == 'from_parameters'
    assert params['from_parameters_and_ros'] == 'from_parameters'
    assert params['only_parameters'] == 1
    assert params['only_ros'] == 2


def test_dump_params_parameter_descriptions():
    dumped = _dump_params([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker',
            namespace='/',
            parameters=[
                Parameter('x', 1),
                Parameter('labels', ['a', 'b']),
            ],
            ros_arguments=['-p', 'x:=99'],
        ),
    ])
    params = dumped['/talker']['ros__parameters']
    assert params['x'] == 1
    assert params['labels'] == ['a', 'b']


def test_dump_params_composable_node_arrays():
    dumped = _dump_params([
        LoadComposableNodes(
            target_container='my_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='demo_nodes_cpp',
                    plugin='demo_nodes_cpp::Talker',
                    name='talker',
                    namespace='demo',
                    parameters=[{
                        'ids': [1, 2, 3],
                        'gains': [0.5, 1.5],
                        'enabled': True,
                    }],
                ),
            ],
        ),
    ])
    params = dumped['/demo/talker']['ros__parameters']
    assert params['ids'] == [1, 2, 3]
    assert params['gains'] == [0.5, 1.5]
    assert params['enabled'] is True


def test_collector_dumps_array_values():
    collector = DumpParamsCollector()
    collector.add_node('/n', {
        'ids': array.array('q', [1, 2, 3]),
        'gains': array.array('d', [0.5, 1.5]),
    })
    dumped = yaml.safe_load(collector.dumps())
    params = dumped['/n']['ros__parameters']
    assert params['ids'] == [1, 2, 3]
    assert params['gains'] == [0.5, 1.5]


def test_dump_params_runtime_precedence(tmp_path):
    global_file = tmp_path / 'global.yaml'
    global_file.write_text('/**:\n  ros__parameters:\n    from_file: 1\n    shared: file\n')
    dumped = _dump_params([
        SetParameter(name='shared', value='global'),
        SetParameter(name='number_text', value='7'),
        SetParametersFromFile(str(global_file)),
        # The global -p rule creates the wildcard key before the node's own file, so the
        # node's dict wins over its later Parameter description.
        Node(
            package='demo_nodes_cpp', executable='talker', name='desc', namespace='/',
            parameters=[{'shared': 'dict'}, Parameter('shared', 'description')],
        ),
        # Without a namespace the dict is written under the wildcard key instead.
        Node(
            package='demo_nodes_cpp', executable='talker', name='desc_no_ns',
            parameters=[{'shared': 'dict'}, Parameter('shared', 'description')],
        ),
        Node(
            package='demo_nodes_cpp', executable='talker', name='no_ns',
            arguments=['--ros-args', '-p', 'from_arguments:=3'],
        ),
        # remappings come after the name rule, ros_arguments before it.
        Node(
            package='demo_nodes_cpp', executable='talker', name='kept',
            remappings=[('__node', 'ignored')], parameters=[{'r': 1}],
        ),
        Node(
            package='demo_nodes_cpp', executable='talker', name='replaced', namespace='/',
            ros_arguments=['-r', '__node:=renamed'], parameters=[{'r': 2}],
        ),
        Node(package='not_installed_pkg', executable='none', name='uninstalled'),
    ])
    common = {'shared': 'file', 'number_text': 7, 'from_file': 1}
    assert dumped['/desc']['ros__parameters'] == dict(common, shared='dict')
    assert dumped['/desc_no_ns']['ros__parameters'] == dict(common, shared='description')
    assert dumped['/no_ns']['ros__parameters'] == dict(common, from_arguments=3)
    assert dumped['/kept']['ros__parameters'] == dict(common, r=1)
    assert '/renamed' in dumped and '/replaced' not in dumped
    # A dict keyed by the launch name does not match the renamed node.
    assert dumped['/renamed']['ros__parameters'] == common
    assert dumped['/uninstalled']['ros__parameters'] == common


def test_dump_params_unnamed_node_and_lifecycle():
    collector = DumpParamsCollector()
    ls = LaunchService(noninteractive=True)
    attach_dump_params_collector(ls.context, collector)
    ls.include_launch_description(LaunchDescription([
        Node(package='demo_nodes_cpp', executable='talker', parameters=[{'a': 1}]),
        LifecycleNode(
            package='lifecycle', executable='lifecycle_talker', name='lc', namespace='',
            parameters=[{'a': 2}], autostart=True,
        ),
        EmitEvent(event=Shutdown(reason='test')),
    ]))
    assert ls.run() == 0
    assert collector.to_yaml_dict() == {'/lc': {'ros__parameters': {'a': 2}}}
    assert len(collector.warnings) == 1
    assert 'unspecified name' in collector.warnings[0]


def test_dump_params_container_and_composable_nodes():
    dumped = _dump_params([
        SetParameter(name='global', value=1),
        ComposableNodeContainer(
            package='rclcpp_components', executable='component_container',
            name='container', namespace='/c', parameters=[{'own': True}],
            composable_node_descriptions=[
                ComposableNode(
                    package='demo_nodes_cpp', plugin='demo_nodes_cpp::Talker',
                    name='comp', parameters=[{'global': 2, 'ids': [1, 2]}],
                    remappings=[('__node', 'renamed')],
                ),
                ComposableNode(
                    package='demo_nodes_cpp', plugin='demo_nodes_cpp::Talker',
                    parameters=[{'skipped': 1}],
                ),
            ],
        ),
    ])
    assert dumped == {
        '/c/container': {'ros__parameters': {'global': 1, 'own': True}},
        '/renamed': {'ros__parameters': {'global': 2, 'ids': [1, 2]}},
    }
