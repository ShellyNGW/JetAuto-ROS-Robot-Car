#!/usr/bin/env python
# encoding: utf-8
import rospy
from move_base_msgs.msg import MoveBaseActionResult
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, PoseStamped

goal_count = 0
try_again = True
goal_reached_count = 0
haved_clicked = False
add_more_point = False
markerArray = MarkerArray()

# move_base状态回调(move_base status callback)
def status_callback(msg):
    global try_again, goal_reached_count, add_more_point
    
    if haved_clicked:  # 防止通过其他方式发布点的回调进来(prevent callbacks from publishing points by other means from coming in)
        try:
            if msg.status.status == 3 or msg.status.status == 4:  # 如果是正常到达目标点(if arrive at the goal normally)
                if msg.status.status == 3:
                    goal_reached_count += 1  # 发送下一个点(send next point)
                    print('******number %s goal reached******' % goal_reached_count)
                if goal_reached_count < goal_count:  # 如果添加的点还没走完继续发送下一个点(if robot doesn't reach the added point, continue sending next point)
                    pose = PoseStamped()
                    pose.header.frame_id = map_frame
                    pose.header.stamp = rospy.Time.now()
                    pose.pose.position.x = markerArray.markers[goal_reached_count].pose.position.x
                    pose.pose.position.y = markerArray.markers[goal_reached_count].pose.position.y
                    pose.pose.orientation.w = 1
                    goal_pub.publish(pose)
                    print('******send number %s goal******' % goal_reached_count + 1)
                elif goal_reached_count == goal_count:  # 所有点到到达了(reach all the points)
                    add_more_point = True
                try_again = True
            else:  # 没有正常到达目标则再次尝试，(retry if robot doesn't reach the goal normally)
                print('******Goal cannot reached has some error :', msg.status.status, ' try again!!!!')
                if try_again:  # 尝试一次失败后不再尝试，直接发送下一个点(if the attempt ends in failure, stop and directly send next point)
                    pose = PoseStamped()
                    pose.header.frame_id = map_frame
                    pose.header.stamp = rospy.Time.now()
                    pose.pose.position.x = markerArray.markers[goal_reached_count].pose.position.x
                    pose.pose.position.y = markerArray.markers[goal_reached_count].pose.position.y
                    pose.pose.orientation.w = 1
                    goal_pub.publish(pose)
                    try_again = False
                elif goal_reached_count < goal_count:
                    goal_reached_count += 1
                    pose = PoseStamped()
                    pose.header.frame_id = map_frame
                    pose.header.stamp = rospy.Time.now()
                    pose.pose.position.x = markerArray.markers[goal_reached_count].pose.position.x
                    pose.pose.position.y = markerArray.markers[goal_reached_count].pose.position.y
                    pose.pose.orientation.w = 1
                    goal_pub.publish(pose)
        except BaseException as e:
            print(e)

def click_callback(msg):
    global add_more_point
    global goal_count, haved_clicked
    
    goal_count += 1
    print('******add number %s goal******' % goal_count)
    
    # 用数字标记来显示点(mark the point with number to display)
    marker = Marker()
    marker.header.frame_id = map_frame
    marker.type = marker.TEXT_VIEW_FACING  # 类型为text(type is text)
    marker.action = marker.ADD
    # 大小(size)
    marker.scale.x = 0.8
    marker.scale.y = 0.8
    marker.scale.z = 0.8
    # 颜色(color)
    marker.color.a = 1
    marker.color.r = 1
    marker.color.g = 0
    marker.color.b = 0
    #marker.lifetime = rospy.Duration(20)  # 显示时间，没有设置默认一直保留(display time. If not set, it will be kept by default)
    # 位置姿态
    marker.pose.position.x = msg.point.x
    marker.pose.position.y = msg.point.y
    marker.pose.position.z = msg.point.z
    marker.pose.orientation.w = 1
    # 显示内容(display content)
    marker.text = str(goal_count)
    
    # 添加到列表以保持显示(add to the list to maintain display)
    markerArray.markers.append(marker)
    marker_id = 0
    for m in markerArray.markers:
        m.id = marker_id
        marker_id += 1
    mark_pub.publish(markerArray)
    
    if goal_count == 1:  # 添加第一个点时，立刻发送到move_base(send to move_base immediately when adding the first point)
        pose = PoseStamped()
        pose.header.frame_id = map_frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = msg.point.x
        pose.pose.position.y = msg.point.y
        pose.pose.orientation.w = 1
        goal_pub.publish(pose)
        print('******send number %s goal******' % goal_count)
    elif add_more_point:  # 机器人已到达所有点，此时添加点要手动触发回调(robot arrives at all points. At this time, At this point, add a point to manually trigger the callback)
        add_more_point = False
        move = MoveBaseActionResult()
        move.status.status = 4
        move.header.stamp = rospy.Time.now()
        print('******add more point******')
        goal_status_pub.publish(move)

    haved_clicked = True

if __name__ == '__main__':
    rospy.init_node('publish_point_node')
   
    map_frame = rospy.get_param('~map_frame', 'map')
    clicked_point = rospy.get_param('~clicked_point', '/clocked_point')
    move_base_result = rospy.get_param('~move_base_result', '/move_base/result')
    
    mark_pub = rospy.Publisher('path_point', MarkerArray, queue_size=100)
    click_sub = rospy.Subscriber(clicked_point, PointStamped, click_callback)
    
    goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
    goal_status_sub = rospy.Subscriber(move_base_result, MoveBaseActionResult, status_callback)
    goal_status_pub = rospy.Publisher(move_base_result, MoveBaseActionResult, queue_size=1)
    
    rospy.spin()
