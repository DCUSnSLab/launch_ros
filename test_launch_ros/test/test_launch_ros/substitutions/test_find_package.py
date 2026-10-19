# Copyright 2019 Open Source Robotics Foundation, Inc.
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

"""Test for the FindPackage substitutions."""

from pathlib import Path
from unittest import mock

from launch import LaunchContext

from launch_ros.substitutions import FindPackagePrefix
from launch_ros.substitutions import FindPackageShare
import launch_ros.substitutions.find_package as find_package

import pytest


def test_find_package_prefix():
    sub = FindPackagePrefix('launch_ros')
    context = LaunchContext()
    package_prefix = Path(sub.perform(context))
    package_xml_file = package_prefix / Path('share/launch_ros/package.xml')
    assert package_xml_file.is_file()


def test_find_package_share():
    sub = FindPackageShare('launch_ros')
    context = LaunchContext()
    package_prefix = Path(sub.perform(context))
    package_xml_file = package_prefix / Path('package.xml')
    assert package_xml_file.is_file()


def test_find_package_paths():
    sub = FindPackageShare('launch_ros') / 'package.xml'
    context = LaunchContext()
    result_path = sub.perform(context)
    assert Path(result_path).is_file()


@pytest.mark.parametrize('substitution,finder', [
    (FindPackagePrefix, 'get_package_prefix'),
    (FindPackageShare, 'get_package_share_directory'),
])
def test_repeated_package_lookup(substitution, finder, monkeypatch):
    """Reuse a package lookup while retaining overlay and cwd changes."""
    context = LaunchContext()
    with mock.patch.object(find_package, finder, return_value='/first') as lookup:
        assert substitution('test_package').perform(context) == '/first'
        assert substitution('test_package').perform(context) == '/first'
        assert lookup.call_count == 1
        monkeypatch.setenv('AMENT_PREFIX_PATH', '/other')
        lookup.return_value = '/second'
        assert substitution('test_package').perform(context) == '/second'
        assert lookup.call_count == 2
        monkeypatch.chdir('/')
        assert substitution('test_package').perform(context) == '/second'
        assert lookup.call_count == 3


def test_package_lookup_failure_is_not_cached():
    """Retry failed lookups when a package becomes available."""
    context = LaunchContext()
    with mock.patch.object(find_package, 'get_package_prefix') as lookup:
        lookup.side_effect = LookupError('missing')
        with pytest.raises(LookupError):
            FindPackagePrefix('missing_package').perform(context)
        lookup.side_effect = None
        lookup.return_value = '/installed'
        assert FindPackagePrefix('missing_package').perform(context) == '/installed'
        assert lookup.call_count == 2
