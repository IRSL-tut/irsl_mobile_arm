#!/usr/bin/env python3

import rospy
import tf
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

_g_br_  = None
_g_pub_ = None

_g_x = 0.0
_g_y = 0.0
_g_th = 0.0

_prev_vx = 0.0
_prev_vy = 0.0
_prev_vth = 0.0

_prev_tm = None

def publishOdom(tm, child_frame='base_link'):
    om_msg = Odometry()

    ## om_msg.header = twist_msg.header ##
    om_msg.header.stamp = tm
    om_msg.header.frame_id = 'odom'
    om_msg.child_frame_id  = child_frame

    om_msg.pose.covariance[0]  = 0.01
    om_msg.pose.covariance[7]  = 0.01
    om_msg.pose.covariance[14] = 10.0 #
    om_msg.pose.covariance[21] = 10.0 #
    om_msg.pose.covariance[28] = 10.0 #
    om_msg.pose.covariance[35] = 0.1
    om_msg.pose.pose.position.x = _g_x
    om_msg.pose.pose.position.y = _g_y
    om_msg.pose.pose.position.z = 0.0
    om_msg.pose.pose.orientation.x = 0.0
    om_msg.pose.pose.orientation.y = 0.0
    om_msg.pose.pose.orientation.z = math.sin(_g_th*0.5)
    om_msg.pose.pose.orientation.w = math.cos(_g_th*0.5)

    om_msg.twist.covariance[0]  = 0.004
    om_msg.twist.covariance[7]  = 0.004
    om_msg.twist.covariance[14] = 10.0 #
    om_msg.twist.covariance[21] = 10.0 #
    om_msg.twist.covariance[28] = 10.0 #
    om_msg.twist.covariance[35] = 0.04
    om_msg.twist.twist.linear.x = _prev_vx # twist_msg.twist.linear.x
    om_msg.twist.twist.linear.y = _prev_vy # twist_msg.twist.linear.y
    ## om_msg.twist.twist.linear.z = 0.0
    ##om_msg.twist.twist.angular.x = 0.0
    ##om_msg.twist.twist.angular.y = 0.0
    om_msg.twist.twist.angular.z = _prev_vth # twist_msg.twist.angular.z

    _g_pub_.publish(om_msg)

def publishTf(tm, child_frame='base_link'):
    _g_br_.sendTransform((_g_x, _g_y, 0.0),
                         (0.0, 0.0, math.sin(_g_th*0.5), math.cos(_g_th*0.5)),
                         tm,      ## msg.header.stamp
                         child_frame,
                         'odom')

def callback(*args):
    now = rospy.Time.now()
    publishOdom(now)
    publishTf(now)

def callback_twist(msg):
    global _g_x, _g_y, _g_th, _prev_vx, _prev_vy, _prev_vth, _prev_tm
    now = rospy.Time.now()
    if _prev_tm is None:
        _prev_vx  = msg.linear.x
        _prev_vy  = msg.linear.y
        _prev_vth = msg.angular.z
        _prev_tm  = now
        return
    else:
        tm = (now - _prev_tm).to_sec()
        #
        cs = math.cos(_g_th)
        ss = math.sin(_g_th)
        #_g_x  += tm * ( cs * msg.linear.x - ss * msg.linear.y )
        #_g_y  += tm * ( ss * msg.linear.x + cs * msg.linear.y )
        #_g_th += tm * msg.angular.z
        _g_x  += tm * ( cs * _prev_vx - ss * _prev_vy )
        _g_y  += tm * ( ss * _prev_vx + cs * _prev_vy )
        _g_th += tm * _prev_vth

        _prev_vx  = msg.linear.x
        _prev_vy  = msg.linear.y
        _prev_vth = msg.angular.z
        _prev_tm  = now

if __name__ == '__main__':
    rospy.init_node('dummy_odom_publisher')

    _g_br_  = tf.TransformBroadcaster()
    _g_pub_ = rospy.Publisher('odom', Odometry)

    sub = rospy.Subscriber('cmd_vel', Twist, callback_twist)
    tm  = rospy.Timer(rospy.Duration(0.04), callback) ## 25Hz(0.04 sec)

    rospy.spin()
