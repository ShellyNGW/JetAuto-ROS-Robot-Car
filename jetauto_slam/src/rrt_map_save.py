#!/usr/bin/env python3
# encoding: utf-8
import os
import rospy
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import OccupancyGrid, Odometry

class MapSaveNode():
    def __init__(self):
        rospy.init_node('rrt_map_save', anonymous=False)
       
        self.rrt_success = False
        self.clicked_count = 0
        self.finish_count = 0
        self.time_stamped = 0
        self.finish = False

        map_frame = rospy.get_param('~map_frame','map')
        odom_topic = rospy.get_param('~odom_topic','odom')
        goal_topic = rospy.get_param('~goal','move_base_simple/goal')
        clicked_topic = rospy.get_param('~clicked_point','clicked_point')
        self.wait_finish_time = rospy.get_param('~wait_finish_time', 20)
        rateHz = rospy.get_param('~rate', 1)
        
        origin = PoseStamped()
        origin.header.seq = 0
        origin.header.frame_id = map_frame
        origin.pose.position.x = 0
        origin.pose.position.y = 0
        origin.pose.position.z = 0
        origin.pose.orientation.x = 0
        origin.pose.orientation.y = 0
        origin.pose.orientation.z = 0
        origin.pose.orientation.w = 1
        
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        rospy.Subscriber(clicked_topic, PointStamped, self.start_callback)
        goal_pub = rospy.Publisher(goal_topic, PoseStamped, queue_size=1)
        
        rate = rospy.Rate(rateHz)
        while not rospy.is_shutdown():
            if self.finish_count >= self.wait_finish_time and not self.finish:
                os.system('rosrun map_server map_saver -f $HOME/jetauto_ws/src/jetauto_slam/maps/rrt_exploration')
                rospy.ServiceProxy('/assigner/stop', Trigger)()
                rospy.loginfo('************save_map*****************')
                self.finish = True
            if self.finish_count >= self.wait_finish_time + 5 and self.finish:
                goal_pub.publish(origin)
                rospy.loginfo('**************rrt_finish**************')
                break
            if not self.rrt_success:
                rospy.loginfo('***************rrt_start**************')
                self.rrt_success = True
            rospy.loginfo('finish count: %d', self.finish_count)
            
            rate.sleep()

    def start_callback(self, msg):
        self.clicked_count += 1

    def odom_callback(self, msg):
        if self.clicked_count > 4:
            #print(msg.twist.twist.linear.x, msg.twist.twist.angular.z)
            if msg.twist.twist.linear.x < 0.005 and msg.twist.twist.angular.z < 0.015:
                current_time = rospy.get_time()
                if current_time > self.time_stamped:
                    self.finish_count += 1
                    self.time_stamped = current_time + 1
            else:
                self.finish_count = 0

if __name__ == '__main__':
    try:
        MapSaveNode()
    except rospy.ROSInterruptException:
        pass
