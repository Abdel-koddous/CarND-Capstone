#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
import numpy as np
import math
from std_msgs.msg import Int32

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. 
MAX_DECEL = 2 # m/s**2

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoints_tree = None
        self.base_lane = None
        self.stopline_wp_idx = -1

    	self.main_loop()

    	rospy.spin()

	
    def main_loop(self):

	    pub_rate = rospy.Rate(50) # publishing frequency at 50 Hz
	    while not rospy.is_shutdown():

	        if self.pose and self.base_waypoints:

        	    # Get closest waypoint ahead of the car from base_waypoints
        	    closest_wp_idx = self.get_closest_wp_idx()
                    #print("closest_wp_idx ==>", closest_wp_idx)
                
		    # Publish sth
                    self.publish_waypoints(closest_wp_idx)

	    	pub_rate.sleep()


    def get_closest_wp_idx(self):
        # Current coordinates of the car
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y

        #print("x = ", x, " - y = ", y)

        # Finding closest wp index to coordinates of the car using KDtree built in waypoints_cb
        closest_idx = self.waypoints_tree.query([x, y], 1)[1] # closest wp can be ahead or behind vehicle

        #print("closest_idx = ", closest_idx)

        # so Let's check if closest wp is ahead or behind the vehicle 
        closest_wp = self.waypoints_2d[closest_idx]
        prev_closest_wp = self.waypoints_2d[closest_idx - 1]

        # Used a dot product between vectors of ...
        closest_vect = np.array(closest_wp)
        prev_closest_vect = np.array([prev_closest_wp])
        car_pos_vect = np.array([x, y])


        val = np.dot(closest_vect - prev_closest_vect, car_pos_vect - closest_vect)

        if val > 0: # closest point behind vehicle
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d) # take wp following closest point
        
        return closest_idx



    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [ [ wp.pose.pose.position.x, wp.pose.pose.position.y ] for wp in waypoints.waypoints ]
            self.waypoints_tree = KDTree(self.waypoints_2d)


    def publish_waypoints(self, closest_idx):
	# Publish a random message
        final_lane = self.generate_final_lane()
	self.final_waypoints_pub.publish(final_lane)


    def generate_final_lane(self):

        lane = Lane()
        closest_idx = self.get_closest_wp_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        self.base_lane = self.base_waypoints.waypoints[closest_idx : farthest_idx]

        if (self.stopline_wp_idx == -1) or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = self.base_lane
        else:
            lane.waypoints = self.deccelerate_waypoints(self.base_lane, closest_idx)
        
        return lane
    
    def deccelerate_waypoints(self, waypoints, closest_idx):
        temp = []

        for i, wp in enumerate(waypoints):

            p = Waypoint()
            p.pose = wp.pose

            stop_idx = max(self.stopline_wp_idx - closest_idx, 0) # Car should stop now ON the stop line
            dist = self.distance(waypoints, i, stop_idx)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1:
                vel = 0.
            
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)

        return temp


    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

