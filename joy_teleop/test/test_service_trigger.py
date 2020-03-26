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
import rclpy
import sensor_msgs.msg
import std_srvs.srv


@pytest.mark.rostest
def generate_test_description():
    parameters = {}
    parameters['twist.type'] = 'service'
    parameters['twist.interface_type'] = 'std_srvs/srv/Trigger'
    parameters['twist.service_name'] = '/trigger'
    parameters['twist.buttons'] = [4]
    joy_teleop_node = launch_ros.actions.Node(
        package='joy_teleop',
        node_executable='joy_teleop',
        output='both',
        parameters=[parameters])

    return (
        launch.LaunchDescription([
            joy_teleop_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'joy_teleop_node': joy_teleop_node,
        }
    )


class TestJoyTeleopService(unittest.TestCase):

    def setUp(self):
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node('test_service', context=self.context)
        self.executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)

    def test_twist(self, launch_service, joy_teleop_node, proc_output):
        joy_publisher = self.node.create_publisher(
            msg_type=sensor_msgs.msg.Joy,
            topic='joy',
            qos_profile=1,
        )

        saw_trigger = False
        future = rclpy.task.Future()

        def trigger(request, response):
            nonlocal saw_trigger
            nonlocal future
            response.success = True
            response.message = 'service_response'
            saw_trigger = True
            future.set_result(True)

            return response

        srv = self.node.create_service(std_srvs.srv.Trigger, '/trigger', trigger)

        joy_msg = sensor_msgs.msg.Joy()
        joy_msg.header.stamp.sec = 0
        joy_msg.header.stamp.nanosec = 0
        joy_msg.header.frame_id = ''
        joy_msg.axes = []
        # Above we set the button to be used as '4', so here we set the '4' button active.
        joy_msg.buttons = [0, 0, 0, 0, 1]

        def publish_message():
            joy_publisher.publish(joy_msg)
            joy_msg.buttons[4] = int(not joy_msg.buttons[4])
        publish_timer = self.node.create_timer(0.1, publish_message)

        # TODO(clalancette): it would be nice to print a different error if we timeout,
        # but spin_until_future_complete() doesn't give us that information.
        self.executor.spin_until_future_complete(future, timeout_sec=10)

        # Check
        self.assertTrue(saw_trigger)

        # Cleanup
        self.node.destroy_timer(publish_timer)
        self.node.destroy_service(srv)
        joy_publisher.destroy()
