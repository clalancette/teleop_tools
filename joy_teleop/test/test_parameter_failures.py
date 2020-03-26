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

import unittest

import launch
import launch_ros
import launch_testing
import pytest


@pytest.mark.rostest
def generate_test_description():
    parameters = {}
    parameters['simple_message.type'] = 'topic'
    parameters['simple_message.interface_type'] = 'std_msgs/msg/String'
    parameters['simple_message.topic_name'] = '/simple_message_type'
    parameters['simple_message.message_value.data.value'] = 'button2'

    joy_teleop_node = launch_ros.actions.Node(
        package='joy_teleop',
        node_executable='joy_teleop',
        output='both',
        parameters=[parameters])

    ld = launch.LaunchDescription()
    ld.add_action(joy_teleop_node)
    ld.add_action(launch_testing.actions.ReadyToTest())

    return ld, locals()


# TODO(clalancette): Can be removed once we switch to Foxy
def assertInStderr(proc_output,
                   expected_output,
                   process,
                   cmd_args=None,
                   *,
                   output_filter=None,
                   strict_proc_matching=True):

    resolved_procs = launch_testing.util.resolveProcesses(
        info_obj=proc_output,
        process=process,
        cmd_args=cmd_args,
        strict_proc_matching=strict_proc_matching
    )
    if output_filter is not None:
        if not callable(output_filter):
            raise ValueError('output_filter is not callable')
    output_match = launch_testing.tools.text.build_text_match(expected_output)

    for proc in resolved_procs:
        full_output = ''.join(
            output.text.decode() for output in proc_output[proc] if output.from_stderr
        )
        if output_filter is not None:
            full_output = output_filter(full_output)
        if output_match(full_output) is not None:
            break
    else:
        names = ', '.join(sorted(p.process_details['name'] for p in resolved_procs))
        assert False, "Did not find '{}' in output for any of the matching process: {}".format(
            expected_output, names
        )


@launch_testing.post_shutdown_test()
class TestJoyTeleopNoButtonParameter(unittest.TestCase):

    def test_joy_teleop_no_button_parameter_exit_code(self, proc_info, joy_teleop_node):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[1])
        assertInStderr(self.proc_output, 'No buttons or axes configured for command', 'joy_teleop')
