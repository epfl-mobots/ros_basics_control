#!/usr/bin/env python3

from tdmclient import ClientAsync

import rospy
from ros_basics_msgs.msg import SimpleVelocities
from ros_basics_msgs.msg import ProximitySensors

import numpy as np

from geometry_msgs.msg import Twist


class ThymioInterface:
    def __init__(self):
        self._client = ClientAsync()
        self._node = self._client.aw(self._client.wait_for_node())
        self._client.aw(self._node.lock_node())

        rospy.init_node('thymio_intf_node', anonymous=True)
        self._psensor_pub = rospy.Publisher(
            'proximity_sensors', ProximitySensors, queue_size=10)
        rospy.Subscriber("set_velocities", SimpleVelocities,
                         self._vel_callback)
        rospy.on_shutdown(self._shutdown)

        self._lb_new = -500.
        self._ub_new = 500.
        self._lb_old = -0.14
        self._ub_old = 0.14
        self._wheel_rad = 0.022
        self._wheel_dist_2 = 0.0475
        # self._regu = 45  # TODO: check this
        self._rate = 10

    def spin(self):
        loop_rate = rospy.Rate(self._rate)
        while not rospy.is_shutdown():
            self._client.aw(self._node.wait_for_variables({"prox.horizontal"}))
            prox = list(self._node.v.prox.horizontal)
            self._node.flush()
            self._publish_psensors(prox)
            loop_rate.sleep()

    def _publish_psensors(self, values):
        ps = ProximitySensors()
        ps.header.stamp = rospy.Time.now()
        ps.values = values
        self._psensor_pub.publish(ps)

    def _vel_callback(self, data):
        l_speed = (2 * data.v + self._wheel_dist_2 * -data.w) / 2
        r_speed = (2 * data.v - self._wheel_dist_2 * -data.w) / 2
        # l_speed /= self._regu
        # r_speed /= self._regu
        l_speed = self._rescale(l_speed)
        r_speed = self._rescale(r_speed)
        self._move(l_speed, r_speed)

    def _move(self, l_speed, r_speed):
        # print("(l, r): ({}, {})".format(l_speed, r_speed))
        l_speed = l_speed if l_speed >= 0 else 2 ** 16 + l_speed
        r_speed = r_speed if r_speed >= 0 else 2 ** 16 + r_speed
        cmd = {
            "motor.left.target": [l_speed],
            "motor.right.target": [r_speed],
        }
        self._node.send_set_variables(cmd)
        self._node.flush()

    def _rescale(self, val):
        if val > self._ub_old:
            val = self._ub_old
        elif val < self._lb_old:
            val = self._lb_old
        nrange = self._ub_new - self._lb_new
        orange = self._ub_old - self._lb_old
        return int((((val - self._lb_old) * nrange) / orange) + self._lb_new)

    def _shutdown(self):
        self._move(0, 0)


if __name__ == '__main__':
    ti = ThymioInterface()
    ti.spin()
