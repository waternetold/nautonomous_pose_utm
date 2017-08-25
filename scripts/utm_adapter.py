#!/usr/bin/env python
import rospy
import utm

# ROS Messages
from geometry_msgs.msg import Point, PointStamped
from nautonomous_pose_msgs.msg import PointWithCovarianceStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus

# Publisher objects
utm_publisher_fix = None
utm_publisher_point = None

map_center_point = Point(0.0, 0.0, 0.0)

center_received = False

# Params
debug_position = False

def center_callback(data):
    global center_received
    map_center_point.x = data.x
    map_center_point.y = data.y
    map_center_point.z = data.z
    center_received = True

# Convert the GPS NavSatFix to a PointWithCovarianceStamped.
def gps_callback(data):
    global utm_publisher_fix, utm_publisher_point, debug_position, center_received

    # Not publish the GPS message if there is no fix.
    if data.status.status == NavSatStatus.STATUS_NO_FIX:
        rospy.logdebug("No fix.")
        return
    
    # And do not publish if the time is invalid.
    if data.header.stamp == rospy.Time(0):
        rospy.logdebug("Time 0")
        return

    if not center_received:
        print "Warning Pose UTM: map center not received yet"
    
    utm_easting, utm_northing = utm.from_latlon(data.latitude, data.longitude)[:2]
    utm_point = Point(utm_easting - map_center_point.x, utm_northing - map_center_point.y, 0)

    # Publish the UTM point with covariance.
    utm_publisher_fix.publish(data.header, utm_point, data.position_covariance, data.position_covariance_type)
        
    # Publish debug position for RVIZ.
    if debug_position:
        utm_publisher_point.publish(data.header, utm_point)

if __name__ == '__main__':
    rospy.init_node('utm_adapter_node')

    # Publishers
    utm_publisher_fix = rospy.Publisher('utm', PointWithCovarianceStamped, queue_size=10)
    utm_publisher_point = rospy.Publisher('debug', PointStamped, queue_size=10)

    rospy.Subscriber("gps_fix_topic", NavSatFix, gps_callback)
    rospy.Subscriber('map_center_topic', Point, center_callback)

    # Params
    debug_position = rospy.get_param('debug_position', False)

    # Subscribers
    rospy.Subscriber("fix", NavSatFix, gps_callback)

    # Spin until the node is closed.
    rospy.spin()
