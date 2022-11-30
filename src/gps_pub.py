#!/usr/bin/python3
import requests
import rospy
from std_msgs.msg import Float64MultiArray

class gps():
    def __init__(self):
        self.gps_pub = rospy.Publisher("/gps", Float64MultiArray, queue_size=1)
        while not rospy.is_shutdown():
            loc = self.get_location()
            msg = Float64MultiArray()
            msg.data = [loc['latitude'], loc['longitude']]
            self.gps_pub.publish(msg)

    def get_ip(self):
        response = requests.get('https://api64.ipify.org?format=json').json()
        return response["ip"]

    def get_location(self):
        ip_address = self.get_ip()
        response = requests.get(f'https://ipapi.co/{ip_address}/json/').json()
        location_data = {
            "ip": ip_address,
            "city": response.get("city"),
            "region": response.get("region"),
            "country": response.get("country_name"),
            "latitude": response.get("latitude"),
            "longitude": response.get("longitude")
        }
        return location_data

if __name__=="__main__":
    rospy.init_node("gps_pub")
    gp = gps()
    rospy.spin()
