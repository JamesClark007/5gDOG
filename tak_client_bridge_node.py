#!/usr/bin/env python

# Author: James Clark tahl.clark@gmail.com
# Since: July 9, 2024
# Project: FOD Dog
# Purpose: bridge for OT -> waypointing

import rospy
import sys
import os
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix
from dog_door.tak_client_bridge import TakClientBridge

if __name__ == "__main__":
    try:
        rospy.init_node("tak_client_bridge_node", anonymous=True)
        rospy.loginfo("IP: %s", os.getenv("IP"))
        wpan_ip = "2001:57b:1200:c08:b933:13cd:9366:55f6"
        if wpan_ip is None:
            rospy.logerr("Missing WPAN interface or WPAN IP - TakClientBridge")
            

        # Initialize the OpenThread client with the specified WPAN IP and port 8088
        dog_bridge_obj = TakClientBridge(wpan_ip, port=8088)

        # Subscribe to the GPS data topic
        rospy.Subscriber('/fix', NavSatFix, dog_bridge_obj.update_known_gps)

        # Attempt to connect to the server
        connected = dog_bridge_obj.connect()
        while not connected:
            #rospy.logwarn("Failed to connect, retrying...")
            rospy.loginfo(connected)
            connected = dog_bridge_obj.connect()

        rospy.loginfo("Successfully connected to the server.")


    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")
        sys.exit(0)
