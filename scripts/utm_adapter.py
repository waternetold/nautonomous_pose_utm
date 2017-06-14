#!/usr/bin/env python
# license removed for brevity
import rospy
import utm

from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import Point, PointStamped
from nautonomous_gps_adapter.msg import PointStampedWithCovariance

utm_publisher = 0

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

    utm_publisher.publish(data.header, point, data.position_covariance, data.position_covariance_type)

if __name__ == '__main__':
     # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('utm_adapter', anonymous=True)

    utm_publisher = rospy.Publisher('/utm/fix', PointStampedWithCovariance, queue_size=10)

    rospy.Subscriber("/gps/fix", NavSatFix, gpsCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
