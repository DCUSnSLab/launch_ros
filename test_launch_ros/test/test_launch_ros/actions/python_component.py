# Copyright 2026 nevlife
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

"""An rclpy node that a component container loads as ``python_component:Listener``."""

from rclpy.node import Node
from std_msgs.msg import String


class Listener(Node):

    def __init__(self, **kwargs):
        super().__init__('py_listener', **kwargs)
        self.create_subscription(String, 'chatter', lambda msg: None, 10)
