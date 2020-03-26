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

import geometry_msgs.msg
from joy_teleop_testing_common import generate_joy_test_description, TestJoyTeleop
import pytest
import rclpy


@pytest.mark.rostest
def generate_test_description():
    parameters = {}
    parameters['twist.type'] = 'topic'
    parameters['twist.interface_type'] = 'geometry_msgs/msg/Twist'
    parameters['twist.topic_name'] = '/cmd_vel'
    parameters['twist.deadman_buttons'] = [2]
    parameters['twist.axis_mappings.linear-x.axis'] = 1
    parameters['twist.axis_mappings.linear-x.scale'] = 0.8
    parameters['twist.axis_mappings.linear-x.offset'] = 0.0
    parameters['twist.axis_mappings.angular-z.axis'] = 3
    parameters['twist.axis_mappings.angular-z.scale'] = 0.5
    parameters['twist.axis_mappings.angular-z.offset'] = 0.0

    return generate_joy_test_description(parameters)


class TestJoyTeleopTopicAxisMappings(TestJoyTeleop):

    def publish_message(self):
        self.joy_publisher.publish(self.joy_msg)
        self.joy_msg.buttons[2] = int(not self.joy_msg.buttons[2])

    def test_twist(self):
        twist = None
        future = rclpy.task.Future()

        def receive_twist(msg):
            nonlocal twist
            nonlocal future
            twist = msg
            future.set_result(True)

        qos = rclpy.qos.QoSProfile(history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
                                   depth=1,
                                   reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
                                   durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)

        twist_subscriber = self.node.create_subscription(
            geometry_msgs.msg.Twist,
            '/cmd_vel',
            receive_twist,
            qos,
        )

        try:
            self.joy_msg.axes = [0.0, 1.0, 0.0, 1.0]
            # Above we set the button to be used as '2', so here we set the '2' button active.
            self.joy_msg.buttons = [0, 0, 1]

            self.executor.spin_until_future_complete(future, timeout_sec=10)

            # Check
            self.assertTrue(future.done() and future.result(),
                            'Timed out waiting for twist topic to complete')
            self.assertEqual(twist.linear.x, 0.8)
            self.assertEqual(twist.angular.z, 0.5)
        finally:
            # Cleanup
            self.node.destroy_subscription(twist_subscriber)
