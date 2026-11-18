#!/usr/bin/env python3
"""Passive eight-agent display and optional one-shot goal; no planner feedback."""
from collections import deque
import colorsys
import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

rospy.init_node('fixed_wall_observer')
positions = {}
orientations = {}
trails = [deque(maxlen=600) for _ in range(8)]
pub = rospy.Publisher('/fixed_wall_benchmark/markers', MarkerArray, queue_size=1)
goal_pub = rospy.Publisher('/goal', PoseStamped, queue_size=1)
ready_at = None
sent = False

def odom(msg, i):
    positions[i] = msg.pose.pose.position
    orientations[i] = msg.pose.pose.orientation


def planners_ready():
    """A recorder may subscribe too; require the actual eight planners."""
    if goal_pub.get_num_connections() < 8:
        return False
    try:
        code, _, state = rospy.get_master().getSystemState()
        subscribers = next((nodes for topic, nodes in state[1] if topic == '/goal'), [])
        return code == 1 and all('/drone_%d_ego_planner_node' % i in subscribers
                                 for i in range(8))
    except Exception as error:
        rospy.logwarn_throttle(5, 'Waiting for planner subscriptions: %s', error)
        return False


subs = [rospy.Subscriber('/drone_%d_visual_slam/odom' % i, Odometry, odom,
                         callback_args=i, queue_size=1) for i in range(8)]
rate = rospy.Rate(5)
while not rospy.is_shutdown():
    markers = []
    for i, p in list(positions.items()):
        trails[i].append(Point(p.x, p.y, p.z))
        rgb = colorsys.hsv_to_rgb(i / 8.0, 0.8, 0.8)
        for kind in ['robot', 'trail']:
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = rospy.Time.now()
            m.ns, m.id = kind, i
            m.action = Marker.ADD
            m.pose.orientation.w = 1
            m.color.r, m.color.g, m.color.b = rgb
            m.color.a = 1
            if kind == 'robot':
                m.type = Marker.MESH_RESOURCE
                m.mesh_resource = 'package://odom_visualization/meshes/fake_drone.dae'
                m.mesh_use_embedded_materials = False
                m.pose.position = p
                m.pose.orientation = orientations[i]
                m.scale.x = m.scale.y = m.scale.z = 0.35
                m.color.r = m.color.g = m.color.b = 0.0
            else:
                m.type = Marker.LINE_STRIP
                m.scale.x = 0.025
                m.points = list(trails[i])
            markers.append(m)
    pub.publish(MarkerArray(markers=markers))
    if len(positions) == 8 and ready_at is None:
        ready_at = rospy.Time.now()
    if (rospy.get_param('~auto_start', False) and not sent and ready_at is not None
            and (rospy.Time.now() - ready_at).to_sec() > 3
            and planners_ready()):
        goal = PoseStamped()
        goal.header.frame_id = 'world'
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = rospy.get_param('~goal_x', 10.0)
        goal.pose.position.z = rospy.get_param('~height', 1.5)
        goal.pose.orientation.w = 1
        goal_pub.publish(goal)
        sent = True
        rospy.loginfo('Eight-agent fixed-wall goal sent once')
    rate.sleep()
