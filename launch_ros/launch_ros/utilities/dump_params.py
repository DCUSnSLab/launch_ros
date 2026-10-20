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

"""Utilities for ``ros2 launch --dump-params`` dry-run parameter collection."""

import array
import re
from typing import Any
from typing import Dict
from typing import Iterable
from typing import List
from typing import Mapping
from typing import Optional
from typing import Tuple

from launch.launch_context import LaunchContext

import yaml

from .to_parameters_list import is_node_name_matched

DUMP_PARAMS_CONTEXT_KEY = 'dump_params_collector'


class DumpParamsError(RuntimeError):
    """Raised when parameter dumping cannot produce a reliable result."""


class DumpParamsCollector:
    """Collects resolved per-node parameter dicts during a dry-run launch."""

    def __init__(self) -> None:
        self._nodes: Dict[str, Dict[str, Any]] = {}
        self._warnings: List[str] = []

    @property
    def warnings(self) -> List[str]:
        return list(self._warnings)

    def warn(self, message: str) -> None:
        self._warnings.append(message)

    def add_node(self, fqn: str, params: Mapping[str, Any]) -> None:
        """
        Record parameters for a fully qualified node name.

        Identical param sets for the same FQN are coalesced. Divergent sets raise.
        """
        normalized_fqn = _normalize_fqn(fqn)
        param_dict = {
            name: _normalize_yaml_value(value) for name, value in params.items()
        }
        if normalized_fqn in self._nodes:
            if self._nodes[normalized_fqn] == param_dict:
                return
            raise DumpParamsError(
                "duplicate node '{}' with divergent parameter sets".format(normalized_fqn)
            )
        self._nodes[normalized_fqn] = param_dict

    def to_yaml_dict(self) -> Dict[str, Any]:
        """Return the standard params-file mapping."""
        return {
            fqn: {'ros__parameters': params}
            for fqn, params in self._nodes.items()
        }

    def dumps(self) -> str:
        """Serialize collected parameters as a params-file YAML document."""
        return yaml.dump(
            self.to_yaml_dict(),
            Dumper=_ParamsFileDumper,
            default_flow_style=False,
            sort_keys=False,
            allow_unicode=True,
        )


def is_dump_params_mode(context: LaunchContext) -> bool:
    """Return True when the launch context is collecting dump-params output."""
    return get_dump_params_collector(context) is not None


def get_dump_params_collector(context: LaunchContext) -> Optional[DumpParamsCollector]:
    """Return the collector attached to the context, if any."""
    return getattr(context.locals, DUMP_PARAMS_CONTEXT_KEY, None)


def attach_dump_params_collector(
    context: LaunchContext,
    collector: Optional[DumpParamsCollector] = None,
) -> DumpParamsCollector:
    """Attach a collector to the launch context and return it."""
    if collector is None:
        collector = DumpParamsCollector()
    context.extend_globals({DUMP_PARAMS_CONTEXT_KEY: collector})
    return collector


_ROS_ARGS_WITH_VALUE = {
    '-r', '--remap', '-p', '--param', '--params-file', '-e', '--enclave',
    '--log-level', '--log-config-file', '--log-file-name',
}


def resolve_node_from_ros_args(argv: Iterable[str]) -> Tuple[Optional[str], Dict[str, Any]]:
    """
    Resolve the node FQN and parameter overrides rcl derives from a command line.

    Follows rcl and rclcpp/rclpy: the first unprefixed ``__node``/``__ns`` rule wins;
    ``-p`` rules and ``--params-file`` entries are grouped by node name key in order of
    first appearance, later values replace earlier ones within a key, and the matching
    keys are applied in that key order.

    :returns: the FQN, or None if no node name rule is present, and the parameters.
    """
    node_name = None
    namespace = None
    keyed: Dict[str, Dict[str, Any]] = {}
    in_ros_args = False
    tokens = iter(argv)
    for token in tokens:
        if token == '--ros-args':
            in_ros_args = True
            continue
        if not in_ros_args:
            continue
        if token == '--':
            in_ros_args = False
            continue
        if token not in _ROS_ARGS_WITH_VALUE:
            continue
        value = next(tokens, None)
        if value is None:
            raise DumpParamsError("missing value after '{}'".format(token))
        if token in ('-r', '--remap'):
            src, _, dst = value.partition(':=')
            if src in ('__node', '__name') and node_name is None:
                node_name = dst
            elif src == '__ns' and namespace is None:
                namespace = dst
        elif token in ('-p', '--param'):
            key, name, param_value = _parse_param_rule(value)
            keyed.setdefault(key, {})[name] = param_value
        elif token == '--params-file':
            with open(value, 'r') as f:
                for key, name, param_value in _params_from_yaml(f.read(), value):
                    keyed.setdefault(key, {})[name] = param_value
    if node_name is None:
        return None, {}
    fqn = '/' + '/'.join(
        part for part in (namespace or '').split('/') + [node_name] if part)
    result: Dict[str, Any] = {}
    for key, params in keyed.items():
        if is_node_name_matched(_normalize_fqn(key), fqn):
            result.update(params)
    return fqn, result


def resolve_composable_node_fqn(
    remap_rules: Iterable[str],
    node_name: str,
    node_namespace: str,
) -> Optional[str]:
    """Resolve the FQN of a composable node the way the component manager names it."""
    rules = ['--ros-args']
    for rule in remap_rules:
        rules += ['-r', rule]
    if node_name:
        rules += ['-r', '__node:=' + node_name]
    if node_namespace:
        rules += ['-r', '__ns:=' + node_namespace]
    fqn, _ = resolve_node_from_ros_args(rules)
    return fqn


_PLAIN_SCALAR_TAG = 'tag:ros.org,2026:rcl_plain'


class _RclYamlLoader(yaml.SafeLoader):
    """Composes YAML keeping untagged plain scalars apart, so rcl typing can apply."""

    def resolve(self, kind, value, implicit):
        if kind is yaml.ScalarNode and implicit[0]:
            return _PLAIN_SCALAR_TAG
        return super().resolve(kind, value, implicit)


_RCL_TRUE = {'Y', 'y', 'yes', 'Yes', 'YES', 'true', 'True', 'TRUE', 'on', 'On', 'ON'}
_RCL_FALSE = {'N', 'n', 'no', 'No', 'NO', 'false', 'False', 'FALSE', 'off', 'Off', 'OFF'}
_RCL_INT = re.compile(r'[+-]?(0[xX][0-9a-fA-F]+|0[0-7]*|[1-9][0-9]*)')
_RCL_FLOAT = re.compile(r'[+-]?((\d+\.?\d*|\.\d+)([eE][+-]?\d+)?|inf(inity)?|nan)', re.IGNORECASE)
_RCL_SPECIAL_FLOAT = {
    '.nan': float('nan'), '.NaN': float('nan'), '.NAN': float('nan'),
    '.inf': float('inf'), '.Inf': float('inf'), '.INF': float('inf'),
    '+.inf': float('inf'), '+.Inf': float('inf'), '+.INF': float('inf'),
    '-.inf': float('-inf'), '-.Inf': float('-inf'), '-.INF': float('-inf'),
}


def _rcl_plain_scalar(value: str) -> Any:
    """Type an untagged, unquoted scalar like rcl_yaml_param_parser does."""
    if value in _RCL_TRUE:
        return True
    if value in _RCL_FALSE:
        return False
    if _RCL_INT.fullmatch(value):
        sign = -1 if value.startswith('-') else 1
        digits = value.lstrip('+-')
        if digits[:2] in ('0x', '0X'):
            number = int(digits, 16)
        elif len(digits) > 1 and digits[0] == '0':
            number = int(digits, 8)
        else:
            number = int(digits)
        number *= sign
        if -2**63 <= number < 2**63:
            return number
    if value in _RCL_SPECIAL_FLOAT:
        return _RCL_SPECIAL_FLOAT[value]
    if _RCL_FLOAT.fullmatch(value):
        number = float(value)
        if number not in (float('inf'), float('-inf')) or 'inf' in value.lower():
            return number
    return value


def _rcl_value(node: yaml.Node) -> Any:
    if isinstance(node, yaml.ScalarNode):
        if node.tag == _PLAIN_SCALAR_TAG:
            return _rcl_plain_scalar(node.value)
        return node.value
    if isinstance(node, yaml.SequenceNode):
        return [_rcl_value(item) for item in node.value]
    raise DumpParamsError('unsupported parameter value at {}'.format(node.start_mark))


def _parse_param_rule(rule: str) -> Tuple[str, str, Any]:
    """Split a ``[node:]name:=value`` rule into its node key, name and typed value."""
    target, sep, raw_value = rule.partition(':=')
    if not sep:
        raise DumpParamsError("invalid parameter rule '{}', expected name:=value".format(rule))
    key, colon, name = target.rpartition(':')
    node = yaml.compose(raw_value, Loader=_RclYamlLoader)
    if node is None:
        raise DumpParamsError("invalid parameter rule '{}', missing value".format(rule))
    return (key if colon else '/**'), name, _rcl_value(node)


def _params_from_yaml(text: str, source: str) -> List[Tuple[str, str, Any]]:
    """Return (node key, parameter name, value) entries of a params file in document order."""
    return [
        (key, name, _rcl_value(value))
        for key, name, value in parameter_file_entries(text, source, _RclYamlLoader)
    ]


def parameter_file_entries(
    text: str,
    source: str,
    loader=yaml.SafeLoader,
) -> List[Tuple[str, str, yaml.Node]]:
    """
    Return the entries of a params file in document order, as rcl reads them.

    Each entry is the node name key (nested namespaces joined with '/'), the parameter
    name (nested names joined with '.') and the composed YAML value node.
    """
    root = yaml.compose(text, Loader=loader)
    entries: List[Tuple[str, str, yaml.Node]] = []
    if root is None:
        return entries

    def add_params(key: str, prefix: str, mapping: yaml.MappingNode) -> None:
        for name_node, value_node in mapping.value:
            name = prefix + name_node.value
            if isinstance(value_node, yaml.MappingNode):
                add_params(key, name + '.', value_node)
            else:
                entries.append((key, name, value_node))

    def visit(parts: List[str], node: yaml.Node) -> None:
        if not isinstance(node, yaml.MappingNode):
            raise RuntimeError(
                "invalid parameter file '{}' at {}".format(source, node.start_mark))
        for key_node, value_node in node.value:
            if key_node.value != 'ros__parameters':
                visit(parts + [key_node.value], value_node)
            elif isinstance(value_node, yaml.MappingNode):
                add_params('/'.join(parts), '', value_node)
            else:
                raise RuntimeError(
                    "invalid parameter file '{}' at {}".format(source, value_node.start_mark))

    visit([], root)
    return entries


class _ParamsFileDumper(yaml.SafeDumper):
    """Quotes strings that rcl would read as another type, such as 'y' or '1e3'."""

    def represent_str(self, data):
        if isinstance(_rcl_plain_scalar(data), str):
            return super().represent_str(data)
        return self.represent_scalar('tag:yaml.org,2002:str', data, style="'")


_ParamsFileDumper.add_representer(str, _ParamsFileDumper.represent_str)


def _normalize_fqn(fqn: str) -> str:
    if not fqn:
        return '/'
    if not fqn.startswith('/'):
        return '/' + fqn
    return fqn


def _normalize_yaml_value(value: Any) -> Any:
    """Return a value yaml.safe_dump can represent."""
    if isinstance(value, (list, tuple, array.array)):
        return [_normalize_yaml_value(item) for item in value]
    return value
