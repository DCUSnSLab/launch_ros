# Copyright 2026 Metro Robots
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

"""Tests for launch_ros.descriptions.CombinedParameterFiles."""

import os

from launch import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.descriptions import CombinedParameterFiles
from launch_ros.utilities.dump_params import resolve_node_from_ros_args
import pytest
from test_parameter_file import get_parameter_file, MockContext
import yaml


def get_params(lc, desc, node='/my_ns/my_node'):
    namespace, _, name = node.rpartition('/')
    _, params = resolve_node_from_ros_args([
        '--ros-args', '-r', '__node:=' + name, '-r', '__ns:=' + (namespace or '/'),
        '--params-file', str(desc.evaluate(lc))])
    return params


def test_combined_description():
    param_file_dict1 = {
        '/my_ns/my_node': {
            'ros__parameters': {
                'indie_param1': 1,
                'repeated_param': 1,
            }
        }
    }
    param_file_dict2 = {
        '/my_ns/my_node': {
            'ros__parameters': {
                'indie_param2': 2,
                'repeated_param': 2,
            }
        }
    }

    lc = MockContext()
    with get_parameter_file(yaml.dump(param_file_dict1)) as file_name1, \
         get_parameter_file(yaml.dump(param_file_dict2)) as file_name2:

        # Load parameters with dict1 first
        desc_a = CombinedParameterFiles([file_name1, file_name2])
        params_a = get_params(lc, desc_a)
        assert isinstance(params_a, dict)
        assert params_a.get('indie_param1') == 1
        assert params_a.get('indie_param2') == 2
        assert params_a.get('indie_param3') is None
        assert params_a.get('repeated_param') == 2

        # Load parameters with dict2 first
        desc_b = CombinedParameterFiles([file_name2, file_name1])
        params_b = get_params(lc, desc_b)
        assert isinstance(params_b, dict)
        assert params_b.get('indie_param1') == 1
        assert params_b.get('indie_param2') == 2
        assert params_b.get('indie_param3') is None
        assert params_b.get('repeated_param') == 1


def test_combined_matches_files_passed_in_order(tmp_path):
    first = tmp_path / 'first.yaml'
    first.write_text(
        '/ns:\n'
        '  node:\n'
        '    ros__parameters:\n'
        '      a: exact\n'
        '/**:\n'
        '  ros__parameters:\n'
        '    a: wild\n'
        '    b: wild\n'
        "    quoted: '5'\n"
        '    word: y\n'
        '    nested: {inner: 1}\n'
    )
    second = tmp_path / 'second.yaml'
    second.write_text(
        '/ns:\n'
        '  other:\n'
        '    ros__parameters:\n'
        '      a: other\n'
        '/ns/node:\n'
        '  ros__parameters:\n'
        '    b: exact\n'
        '    nested.inner: 2\n'
        '/**:\n'
        '  ros__parameters:\n'
        '    a: wild_again\n'
    )
    separate = resolve_node_from_ros_args([
        '--ros-args', '-r', '__node:=node', '-r', '__ns:=/ns',
        '--params-file', str(first), '--params-file', str(second)])[1]
    assert separate == {
        'a': 'wild_again', 'b': 'wild', 'quoted': '5', 'word': True, 'nested.inner': 1}
    desc = CombinedParameterFiles([str(first), str(second)])
    assert get_params(MockContext(), desc, '/ns/node') == separate
    assert get_params(MockContext(), desc, '/ns/other') == dict(separate, a='other')


def test_combined_files_are_written_once_per_content(tmp_path):
    for directory in ('one', 'two'):
        (tmp_path / directory).mkdir()
        (tmp_path / directory / 'params.yaml').write_text(
            '/**:\n  ros__parameters:\n    dir: {}\n'.format(directory))
    extra = tmp_path / 'extra.yaml'
    extra.write_text('/**:\n  ros__parameters:\n    extra: 1\n')
    desc = CombinedParameterFiles(
        [PathJoinSubstitution([LaunchConfiguration('dir'), 'params.yaml']), str(extra)])

    lc = LaunchContext()
    lc.launch_configurations['dir'] = str(tmp_path / 'one')
    path_one = desc.evaluate(lc)
    assert desc.evaluate(lc) == path_one
    # Another scope with other substitution values gets its own file.
    lc.launch_configurations['dir'] = str(tmp_path / 'two')
    path_two = desc.evaluate(lc)
    assert path_two != path_one
    assert yaml.safe_load(path_two.read_text())['/**']['ros__parameters'] == {
        'dir': 'two', 'extra': 1}
    assert yaml.safe_load(path_one.read_text())['/**']['ros__parameters']['dir'] == 'one'

    desc.cleanup()
    assert not os.path.exists(path_one) and not os.path.exists(path_two)


def test_combined_single_file_is_not_rewritten(tmp_path):
    only = tmp_path / 'only.yaml'
    only.write_text('/**:\n  ros__parameters:\n    a: 1\n')
    assert CombinedParameterFiles([only]).evaluate(MockContext()) == only


def test_combined_files_substitutions_in_contents(tmp_path):
    source = tmp_path / 'source.yaml'
    source.write_text('/**:\n  ros__parameters:\n    value: $(var value)\n')
    lc = LaunchContext()
    lc.launch_configurations['value'] = '3'
    desc = CombinedParameterFiles([source], allow_substs=True)
    assert get_params(lc, desc) == {'value': 3}


def test_combined_files_requires_a_list():
    with pytest.raises(TypeError):
        CombinedParameterFiles('params.yaml')
