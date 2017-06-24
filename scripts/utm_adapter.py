#!/usr/bin/env python
# license removed for brevity
import rospy
import utm

from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import Point, PointStamped
from nautonomous_pose_msgs.msg import PointWithCovarianceStamped

utm_publisher_fix = None
utm_publisher_point = None

def gpsCallback(data):
    if data.status.status == NavSatStatus.STATUS_NO_FIX:
        print "No fix."
        return

    if data.header.stamp == rospy.Time(0):
        print "Time 0"
        return
    
    latitude = data.latitude
    longitude = data.longitude

    utm_easting, utm_northing = utm.from_latlon(latitude, longitude)[:2]
    point = Point(utm_easting, utm_northing, 0)

    utm_publisher_fix.publish(data.header, point, data.position_covariance, data.position_covariance_type)
    utm_publisher_point.publish(data.header, point)

if __name__ == '__main__':
    rospy.init_node('utm_adapter', anonymous=True)

    utm_publisher_fix = rospy.Publisher('/utm/fix', PointWithCovarianceStamped, queue_size=10)
    utm_publisher_point = rospy.Publisher('point', PointStamped, queue_size=10)

    rospy.Subscriber("/gps/fix", NavSatFix, gpsCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
