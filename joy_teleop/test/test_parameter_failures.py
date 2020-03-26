# -*- coding: utf-8 -*-
#
# Copyright (c) 2020 Open Source Robotics Foundation
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of PAL Robotics SL. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import contextlib
import unittest

import launch
import launch_ros
import launch_testing
import pytest


@pytest.mark.rostest
def generate_test_description():
    return launch.LaunchDescription([launch_testing.actions.ReadyToTest()])
    # return launch.LaunchDescription([
    #     # Always restart daemon to isolate tests.
    #     launch.actions.ExecuteProcess(
    #         cmd=['ros2', 'daemon', 'stop'],
    #         name='daemon-stop',
    #         on_exit=[
    #             launch.actions.ExecuteProcess(
    #                 cmd=['ros2', 'daemon', 'start'],
    #                 name='daemon-start',
    #             )
    #         ]
    #     ),
    # ])


class TestJoyTeleopParameterFailures(unittest.TestCase):

    @classmethod
    def setUpClass(cls, launch_service, proc_info, proc_output):
        @contextlib.contextmanager
        def launch_joy_teleop(self, parameters):
            joy_teleop_node = launch_ros.actions.Node(
                package='joy_teleop',
                node_executable='joy_teleop',
                output='both',
                parameters=[parameters])

            with launch_testing.tools.launch_process(
                    launch_service, joy_teleop_node, proc_info, proc_output) as joy_teleop:
                yield joy_teleop

        cls.launch_joy_teleop = launch_joy_teleop

    def test_no_buttons_or_axes(self):
        parameters = {}
        parameters['simple_message.type'] = 'topic'
        parameters['simple_message.interface_type'] = 'std_msgs/msg/String'
        parameters['simple_message.topic_name'] = '/simple_message_type'
        parameters['simple_message.message_value.data.value'] = 'button2'

        with self.launch_joy_teleop(parameters) as joy_teleop_process:
            assert joy_teleop_process.wait_for_shutdown(timeout=10)

        assert joy_teleop_process.exit_code == 1
        # assert joy_teleop_process.output == ''
