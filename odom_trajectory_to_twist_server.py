#!/usr/bin/env python3

import rospy
import actionlib
import control_msgs.msg
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, Vector3
import nav_msgs.msg

import math
import scipy.interpolate
import numpy as np

from cnoid.IRSLCoords import coordinates

def interpolate1D(tt, yy):
    vv = [0.0]
    for idx in range(1, len(tt)-1):
        v_prev = (yy[idx] - yy[idx-1])/(tt[idx] - tt[idx-1])
        v_next = (yy[idx+1] - yy[idx])/(tt[idx+1] - tt[idx])
        if v_prev * v_next <= 0.0:
            v = 0.0
        else:
            v = (v_prev + v_next) * 0.5
        vv.append(v)
    vv.append(0.0)
    #
    ff = []
    for st, ed, y_st, y_ed, v_st, v_ed in zip(tt[0:-1], tt[1:], yy[0:-1], yy[1:], vv[0:-1], vv[1:]):
        t_ = np.array([st, ed])
        y_ = np.array([y_st, y_ed])
        bc_type=[ [(1, v_st), (2, 0.0)], [(1, v_ed), (2, 0.0)] ]
        ff.append( scipy.interpolate.make_interp_spline(t_, y_, k = 5, bc_type=bc_type) )
        #bc_type=[ [(1, v_st) ], [(1, v_ed) ] ]
        #ff.append( scipy.interpolate.make_interp_spline(t_, y_, k = 3, bc_type=bc_type) )
    return ff

def interpolate(tt, xx, yy, th, rate):
    ffx  = interpolate1D(tt, xx)
    ffy  = interpolate1D(tt, yy)
    ffth = interpolate1D(tt, th)
    #
    npt = [ np.linspace(st, ed, num=(math.floor( (ed - st)*rate ) + 1))[:-1] for st, ed in zip(tt[0:-1], tt[1:]) ]
    #
    tnew = []
    xnew = []
    ynew = []
    thnew = []
    for nn, fx_, fy_, fz_ in zip(npt, ffx, ffy, ffth):
        tnew  += nn.tolist()
        xnew  += fx_(nn).tolist()
        ynew  += fy_(nn).tolist()
        thnew += fz_(nn).tolist()
    #
    return tnew, xnew, ynew, thnew

#### test
#import matplotlib.pyplot as plt
#import random
#random.seed()
#tt = [0, 0.5, 1.0, 1.5, 2, 4, 8, 10]
##yy = [0, 1.0, 1.0, 1.0, 0]
#yy = [0.0]
#for t in tt[1:]:
#    yy.append(4*random.random() - 2)
#new, xnew = func(tt, yy)
#plt.plot(np.array(tt), np.array(yy), 'o', np.array(tnew), np.array(xnew), '-'); plt.show()

class OdomTrajectoryAction(object):
    def __init__(self, name, rate=100):
        self.set_parameters();
        self._action_name = name
        self._twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self._odom_sub  = rospy.Subscriber('odom', nav_msgs.msg.Odometry, self.odom_cb, queue_size=1)
        self._as = actionlib.SimpleActionServer(self._action_name + '/follow_joint_trajectory',
                                                control_msgs.msg.FollowJointTrajectoryAction,
                                                execute_cb=self.execute_cb, auto_start = False)
        self._current_odom = None
        self._as.start()

    def set_parameters(self):
        self._rate = 100
        if rospy.has_param('~rate'):
            rate = rospy.get_param('~rate')

    def gen_feedback(self):
        pass

    def odom_cb(self, msg):
        self._current_odom = msg

    def execute_cb(self, goal):
        ### check time mode or immediately
        # base_tm = goal.trajectory.header.stamp
        # rospy.getTime()
        #goal.trajectory.header.frame_id
        absolute_ = True
        if self._current_odom is None:
            absolute_ = False
            rospy.logwarn('do not receive odom')
        else:
            odom_ = self._current_odom
            cds = coordinates([odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z],
                              [odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y, odom_.pose.pose.orientation.z, odom_.pose.pose.orientation.w])
            odom_x = cds.pos[0]
            odom_y = cds.pos[1]
            odom_t = cds.RPY[2]

        jnames_ = goal.trajectory.joint_names
        points_ = goal.trajectory.points

        if not 'odom_x' in jnames_ or not 'odom_y' in jnames_ or not 'odom_t' in jnames_:
            _result = control_msgs.msg.FollowJointTrajectoryResult()
            _result.error_code   = -2
            _result.error_string = 'invalid joint_names / {}'.format(jnames_)
            self._as.set_succeeded(_result)
            rospy.logwarn('invalid joint_names / {}'.format(jnames_))
            return

        x_index = jnames_.index('odom_x')
        y_index = jnames_.index('odom_y')
        t_index = jnames_.index('odom_t')

        if absolute_:
            tt = [0.0]
            xx = [odom_x]
            yy = [odom_y]
            zz = [odom_t]
        else:
            tt = [0.0]
            xx = [0.0]
            yy = [0.0]
            zz = [0.0]
        for p in points_:
            from_start = p.time_from_start.to_sec()
            tt.append(from_start)
            xx.append(p.positions[x_index])
            yy.append(p.positions[y_index])
            zz.append(p.positions[t_index])

        newtt, newxx, newyy, newzz = interpolate(tt, xx, yy, zz, self._rate)

        r = rospy.Rate(self._rate)
        dt = 1.0/self._rate
        success = True
        prev_xx = xx[0]
        prev_yy = yy[0]
        prev_zz = zz[0]
        cntr = 0
        while len(newxx) > 0:
            if self._as.is_preempt_requested():
                rospy.logwarn('{}: Preempted'.format(self._action_name))
                self._as.set_preempted()
                success = False
                self._twist_pub.publish(Twist())
                break
            #
            xx_ = newxx.pop(0)
            yy_ = newyy.pop(0)
            zz_ = newzz.pop(0)
            wvx = (xx_ - prev_xx)*self._rate ## world
            wvy = (yy_ - prev_yy)*self._rate ## world
            sz = math.sin(prev_zz)
            cz = math.cos(prev_zz)
            vx =   wvx * cz + wvy  * sz
            vy = - wvx * sz + wvy  * cz
            vz = (zz_ - prev_zz)*self._rate
            #
            msg = Twist(linear=Vector3(x=vx, y=vy), angular=Vector3(z=vz))
            self._twist_pub.publish(msg)
            #
            feedback_ = control_msgs.msg.FollowJointTrajectoryFeedback(header=Header(stamp=rospy.Time.now()))
            feedback_.joint_names = jnames_
            feedback_.desired.positions  = [xx_, yy_, zz_]
            feedback_.desired.velocities = [vx, vy, vz ]
            feedback_.desired.time_from_start = rospy.Duration(cntr * dt)
            self._as.publish_feedback(feedback_)
            #
            prev_xx = xx_
            prev_yy = yy_
            prev_zz = zz_
            cntr += 1
            r.sleep()
        ##
        self._twist_pub.publish(Twist())
        if success:
            ###
            _result = control_msgs.msg.FollowJointTrajectoryResult()
            _result.error_code   = 0
            _result.error_string = ''
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(_result)

if __name__ == '__main__':
    rospy.init_node('omni_base_controller')

    server = OdomTrajectoryAction(rospy.get_name())
    rospy.spin()

### GOAL
#trajectory_msgs/JointTrajectory trajectory
#  std_msgs/Header header
#    uint32 seq
#    time stamp
#    string frame_id
#  string[] joint_names
#  trajectory_msgs/JointTrajectoryPoint[] points
#    float64[] positions
#    float64[] velocities
#    float64[] accelerations
#    float64[] effort
#    duration time_from_start
#control_msgs/JointTolerance[] path_tolerance
#  string name
#  float64 position
#  float64 velocity
#  float64 acceleration
#control_msgs/JointTolerance[] goal_tolerance
#  string name
#  float64 position
#  float64 velocity
#  float64 acceleration
#duration goal_time_tolerance

### Feedback
#std_msgs/Header header
#  uint32 seq
#  time stamp
#  string frame_id
#string[] joint_names
#trajectory_msgs/JointTrajectoryPoint desired
#  float64[] positions
#  float64[] velocities
#  float64[] accelerations
#  float64[] effort
#  duration time_from_start
#trajectory_msgs/JointTrajectoryPoint actual
#  float64[] positions
#  float64[] velocities
#  float64[] accelerations
#  float64[] effort
#  duration time_from_start
#trajectory_msgs/JointTrajectoryPoint error
#  float64[] positions
#  float64[] velocities
#  float64[] accelerations
#  float64[] effort
#  duration time_from_start

### Result
#int32 SUCCESSFUL=0
#int32 INVALID_GOAL=-1
#int32 INVALID_JOINTS=-2
#int32 OLD_HEADER_TIMESTAMP=-3
#int32 PATH_TOLERANCE_VIOLATED=-4
#int32 GOAL_TOLERANCE_VIOLATED=-5
#int32 error_code
#string error_string

### trajectory_msgs/JointTrajectoryPoint
#float64[] positions
#float64[] velocities
#float64[] accelerations
#float64[] effort
#duration time_from_start

###nav_msgs/Odometry
#std_msgs/Header header
#  uint32 seq
#  time stamp
#  string frame_id
#string child_frame_id
#geometry_msgs/PoseWithCovariance pose
#  geometry_msgs/Pose pose
#    geometry_msgs/Point position
#      float64 x
#      float64 y
#      float64 z
#    geometry_msgs/Quaternion orientation
#      float64 x
#      float64 y
#      float64 z
#      float64 w
#  float64[36] covariance
#geometry_msgs/TwistWithCovariance twist
#  geometry_msgs/Twist twist
#    geometry_msgs/Vector3 linear
#      float64 x
#      float64 y
#      float64 z
#    geometry_msgs/Vector3 angular
#      float64 x
#      float64 y
#      float64 z
#  float64[36] covariance
