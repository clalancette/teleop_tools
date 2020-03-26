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
import std_msgs.msg


@pytest.mark.rostest
def generate_test_description():
    parameters = {}
    parameters['simple_message.type'] = 'topic'
    parameters['simple_message.interface_type'] = 'std_msgs/msg/String'
    parameters['simple_message.topic_name'] = '/simple_message_type'
    parameters['simple_message.deadman_buttons'] = [2]
    parameters['simple_message.message_value.data.value'] = 'button2'
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


class TestJoyTeleopDebounce(unittest.TestCase):

    def setUp(self):
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node('test_debounce', context=self.context)
        self.executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)

    def test_debounce(self, launch_service, joy_teleop_node, proc_output):
        # This test works by having a subscription to the output topic.  Every
        # time the subscription is received, we increment a counter.  There is
        # also a timer callback that executes every 0.1 seconds, publishing the
        # the 'joy' message with button 2 on.  We then let the system run for
        # 5 seconds.  Due to debouncing, we should only ever receive one
        # subscription callback in that time.

        joy_publisher = self.node.create_publisher(
            msg_type=sensor_msgs.msg.Joy,
            topic='joy',
            qos_profile=1,
        )

        num_received_messages = 0

        def receive_simple_message(msg):
            nonlocal num_received_messages
            num_received_messages += 1

        simple_message_subscriber = self.node.create_subscription(
            std_msgs.msg.String,
            '/simple_message_type',
            receive_simple_message,
            1,
        )

        joy_msg = sensor_msgs.msg.Joy()
        joy_msg.header.stamp.sec = 0
        joy_msg.header.stamp.nanosec = 0
        joy_msg.header.frame_id = ''
        joy_msg.axes = []
        # Above we set the button to be used as '2', so here we set the '2' button active.
        joy_msg.buttons = [0, 0, 1]

        def publish_message():
            nonlocal num_received_messages
            joy_publisher.publish(joy_msg)
        publish_timer = self.node.create_timer(0.1, publish_message)

        # TODO(clalancette): it would be nice to print a different error if we timeout,
        # but spin_until_future_complete() doesn't give us that information.
        self.executor.spin_until_future_complete(rclpy.task.Future(), timeout_sec=5)

        # Check
        self.assertTrue(num_received_messages == 1)

        # Cleanup
        self.node.destroy_timer(publish_timer)
        self.node.destroy_subscription(simple_message_subscriber)
        joy_publisher.destroy()
