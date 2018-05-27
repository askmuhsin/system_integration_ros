#!/usr/bin/env python

from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import numpy as np
import rospy
import math
import sys

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.
As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.
Once you have created dbw_node, you will update this node to use the status of traffic lights too.
Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
'''

LOOKAHEAD_WPS = 50

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        rospy.logwarn("*** NODE Initialization - waypoint_updater ***")
        ## Subscribe to nodes :
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/obstacle_waypoints', PoseStamped, self.obstacle_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)


        ## Publish waypoints
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.lane = None
        self.current_position = None
        self.next_stopline_waypoint = -1
        self.current_velocity_in_mps = None
        self.current_state = 2
        self.state_change = True
        self.number_of_waypoints = None
        self.final_waypoints = None

        # self.acceleration_limit_in_mps = rospy.get_param('~accel_limit', 1.)
        self.deceleration_limit_max_in_mps = -rospy.get_param('~decel_limit', -5.)
        self.deceleration_limit_min_in_mps = min(1.0, -rospy.get_param('~decel_limit', -5.) / 2.)
        self.max_velocity_in_mps = rospy.get_param('/waypoint_loader/velocity') / 3.6
        self.acceleration_limit_in_mps = 2.0

        # rospy.logwarn("*** max_velocity_in_mps {} ***".format(self.max_velocity_in_mps))
        rospy.logwarn("*** acceleration_limit_in_mps {} ***".format(self.acceleration_limit_in_mps))

        self.loop()     ## <<ROUTE 2>>

## _____________________________________________________________________________
    def loop(self):
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            self.publish_waypoints()        ## <<ROUTE 3>>
            rate.sleep()
## _____________________________________________________________________________
    def publish_waypoints(self):
        if self.lane is None or self.current_position is None or self.current_velocity_in_mps is None:
            return

        waypoint_idx = self.get_closest_waypoint_idx()      ## <<ROUTE 4>>
        lane = Lane()
        lane.header.stamp = rospy.Time.now()

        ## Machine state
        if self.current_state == 1:     ## 1 --> Move forward (STATE)
            if self.next_stopline_waypoint != -1:
                start_position = self.current_position
                end_position = self.lane.waypoints[self.next_stopline_waypoint].pose.pose.position
                brake_distance = self.distance_bw_pos(start_position, end_position) - 0.5       ## <<ROUTE 6>>

                min_brake_distance = 0.5*self.current_velocity_in_mps**2 / self.deceleration_limit_max_in_mps
                max_brake_distance = 0.5*self.current_velocity_in_mps**2 / self.deceleration_limit_min_in_mps
                if max_brake_distance >= brake_distance >= min_brake_distance:
                    self.current_state = 2      ## 2 --> Slow down
                    self.state_change = True

        elif self.current_state == 2:   ## 2 --> Slow down (STATE)
            if self.next_stopline_waypoint == -1:
                self.current_state = 1      ## 1 --> Move forward (STATE)
                self.state_change = True

        else:
            rospy.logerr("*** publish_waypoints : UNKNOWN state ***")

        ## Machine state logic
        if self.state_change:
            if self.current_state == 1:     ## 1 --> Move forward (STATE)
                self.accelerate(lane, waypoint_idx)     ## <<ROUTE 6.1>>
            elif self.current_state == 2:   ## 2 --> Slow down (STATE)
                self.decelerate(lane, waypoint_idx)     ## <<ROUTE 6.2>>

        elif not self.state_change:
            if self.current_state == 1:     ## 1 --> Move forward (STATE)
                self.cont_state(lane, waypoint_idx, self.max_velocity_in_mps)     ## <<ROUTE 6.3>>
            elif self.current_state == 2:   ## 2 --> Slow down (STATE)
                self.cont_state(lane, waypoint_idx, 0)     ## <<ROUTE 6.4>>
        else:
            rospy.logerr("*** publish_waypoints : UNKNOWN state ***")

        self.state_changed = False
        self.final_waypoints = lane.waypoints
        self.final_waypoints_pub.publish(lane)

## _____________________________________________________________________________
    def get_closest_waypoint_idx(self):
        minimal_distance = sys.maxint
        waypoints = self.lane.waypoints
        waypoint_idx = -1

        for n in range(self.number_of_waypoints):
            distance = self.distance_bw_pos(self.current_position, waypoints[n].pose.pose.position)
            if distance < minimal_distance:
                minimal_distance = distance
                waypoint_idx = n

        if self.is_waypoint_behind(waypoint_idx):       ## <<ROUTE 5.2>>
            waypoint_idx = (waypoint_idx + 1) % self.number_of_waypoints

        return waypoint_idx

## _____________________________________________________________________________
    def distance_bw_pos(self, pos1, pos2):
        return ((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2 + (pos1.z - pos2.z)**2)**0.5

## _____________________________________________________________________________
    def is_waypoint_behind(self, waypoint_idx):
        dx = self.current_position.x - self.lane.waypoints[waypoint_idx].pose.pose.position.x
        dy = self.current_position.y - self.lane.waypoints[waypoint_idx].pose.pose.position.y
        nx = self.lane.waypoints[(waypoint_idx + 1) % self.number_of_waypoints].pose.pose.position.x - \
             self.lane.waypoints[waypoint_idx].pose.pose.position.x
        ny = self.lane.waypoints[(waypoint_idx + 1) % self.number_of_waypoints].pose.pose.position.y - \
             self.lane.waypoints[waypoint_idx].pose.pose.position.y
        dp = dx * nx + dy * ny
        return dp > 0.0

## _____________________________________________________________________________
    def accelerate(self, lane, waypoint_idx):
        # rospy.logwarn("*** STATE accelerate ***")

        acceleration = self.acceleration_limit_in_mps
        current_velocity = self.current_velocity_in_mps
        target_velocity = self.current_velocity_in_mps

        for n in range(LOOKAHEAD_WPS):
            if target_velocity > self.max_velocity_in_mps:
                break
            start_pos = self.current_position
            end_pos = self.lane.waypoints[(waypoint_idx + n) % self.number_of_waypoints].pose.pose.position
            distance = self.distance_bw_pos(start_pos, end_pos)     ## <<ROUTE 6.5>>

            target_velocity = (current_velocity**2.0 + 2.0*acceleration*distance)**0.5
            if target_velocity > self.max_velocity_in_mps:
                target_velocity = self.max_velocity_in_mps

            current_waypoint = self.lane.waypoints[(waypoint_idx + n) % self.number_of_waypoints]
            current_waypoint.twist.twist.linear.x = target_velocity
            lane.waypoints.append(current_waypoint)

## _____________________________________________________________________________
    def decelerate(self, lane, waypoint_idx):
        # rospy.logwarn("*** STATE decelerate ***")

        current_velocity = self.current_velocity_in_mps
        target_velocity = self.current_velocity_in_mps

        start_pos = self.current_position
        end_pos = self.lane.waypoints[self.next_stopline_waypoint].pose.pose.position
        distance = self.distance_bw_pos(start_pos, end_pos) - 0.5     ## <<ROUTE 6.6>>

        acceleration = current_velocity**2.0 / (2.0*distance)

        for n in range(LOOKAHEAD_WPS):
            if target_velocity < 0.0:
                break
            start_pos = self.current_position
            end_pos = self.lane.waypoints[(waypoint_idx + n) % self.number_of_waypoints].pose.pose.position
            distance = self.distance_bw_pos(start_pos, end_pos)         ## <<ROUTE 6.7>>

            target_velocity = current_velocity**2.0 - 2.0*acceleration*distance

            if target_velocity <= 0.0:
                target_velocity = 0
            else:
                target_velocity = target_velocity**0.5

            current_waypoint = self.lane.waypoints[(waypoint_idx + n) % self.number_of_waypoints]
            current_waypoint.twist.twist.linear.x = target_velocity
            lane.waypoints.append(current_waypoint)

## _____________________________________________________________________________
    def cont_state(self, lane, waypoint_idx, current_velocity):
        # rospy.logwarn("*** STATE Continue {}(cv) ***".format(current_velocity))
        lim_ = 0
        for n in range(len(self.final_waypoints)):
            if self.final_waypoints[n].pose.pose.position == self.lane.waypoints[waypoint_idx].pose.pose.position:
                break
            lim_ = n

        for i in range(lim_, len(self.final_waypoints)):
            current_waypoint = self.lane.waypoints[(waypoint_idx + i - lim_) % self.number_of_waypoints]
            current_waypoint.twist.twist.linear.x = self.final_waypoints[i].twist.twist.linear.x
            lane.waypoints.append(current_waypoint)

        for i in range(len(lane.waypoints), LOOKAHEAD_WPS):
            current_waypoint = self.lane.waypoints[(waypoint_idx + i) % self.number_of_waypoints]
            current_waypoint.twist.twist.linear.x = current_velocity
            lane.waypoints.append(current_velocity)

## Callbacks
## _____________________________________________________________________________
    def pose_cb(self, msg):
        self.current_position = msg.pose.position

## _____________________________________________________________________________
    def waypoints_cb(self, waypoints):
        self.lane = waypoints
        self.number_of_waypoints = len(self.lane.waypoints)

## _____________________________________________________________________________
    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

## _____________________________________________________________________________
    def traffic_cb(self, msg):
        self.next_stopline_waypoint = msg.data
        if self.next_stopline_waypoint != -1 and self.number_of_waypoints is not None:
            self.next_stopline_waypoint = (self.next_stopline_waypoint - 5 + self.number_of_waypoints) % self.number_of_waypoints

## _____________________________________________________________________________
    def velocity_cb(self, msg):
        self.current_velocity_in_mps = msg.twist.linear.x


if __name__ == '__main__':
    try:
        WaypointUpdater()       ## <<ROUTE 1>>
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
