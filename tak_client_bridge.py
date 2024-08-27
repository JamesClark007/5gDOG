import socket
import json
import rospy
import threading
import select
from std_msgs.msg import Bool, String
from sensor_msgs.msg import NavSatFix
from dog_door.srv import AchieveGPSWaypoint
from dog_door.msg import GpsPoint

class TakClientBridge:
    def __init__(self, server_ip, port=8088):
        self.server_ip = server_ip
        self.port = port
        self.socket = None
        self.gps = None
        self.connected = False
        self.listen_timeout = 1.0
        self.reconnect_attempts = 3
        self.reconnect_delay = 5

        self.pub_latlong_target = rospy.Publisher("/robot_exec/current_wp_latlon", GpsPoint, queue_size=1)
        self.plan_status_pub = rospy.Publisher("/robot_exec/plan_status", String, queue_size=1)
        self.set_estop_pub = rospy.Publisher("/soft_estop/trigger", Bool, queue_size=1)
        self.reset_estop_pub = rospy.Publisher("/soft_estop/reset", Bool, queue_size=1)

        rospy.Subscriber("/fix", NavSatFix, self.update_known_gps)

    def update_known_gps(self, data):
        self.gps = data

    def connect(self):
        try:
            #print("rospy")#"Attempting to create socket connection to {self.server_ip} on port {self.port}")
            self.socket = socket.socket(socket.AF_INET6 if ':' in self.server_ip else socket.AF_INET, socket.SOCK_STREAM)
            #print("rospy")#"Connecting to server {self.server_ip}:{self.port}")
            self.socket.connect((self.server_ip, self.port))
            self.socket.setblocking(0)
            self.connected = True
            print("Connection successful")
            while self.connected:
                self.send_gps_data_continuously()
            #send data here
            rospy.sleep(5)
            return True
        except Exception as e:
            #print("rospy")#"Failed to connect to server: {e}")
            return False

    def listen_continuously(self):
        partial_data = ""
        while not rospy.is_shutdown() and self.connected:
            try:
                ready = select.select([self.socket], [], [], self.listen_timeout)
                if ready[0]:
                    data = self.socket.recv(1024).decode('utf-8')
                    #print("rospy")#"Received data from server: {data}")
                    if not data:
                        self.handle_disconnect()
                        break

                    partial_data += data
                    while '\n' in partial_data:
                        line, partial_data = partial_data.split('\n', 1)
                        self.process_data(line)
                else:
                    continue
            except Exception as e:
                #print("rospy")#"Error while listening to server: {e}")
                self.handle_disconnect()
                break

    def process_data(self, data):
        if data == "HEARTBEAT":
            return

        try:
            ot_packet = json.loads(data)
            #print("rospy")#"Processing received data: {ot_packet}")

            if "waypoint" in ot_packet:
                lat = ot_packet["waypoint"]["lat"]
                lon = ot_packet["waypoint"]["lon"]
                threading.Thread(target=self.achieve, args=(lat, lon)).start()

            elif "relative_waypoint" in ot_packet:
                rel_lat = ot_packet["relative_waypoint"]["rel_lat"]
                rel_lon = ot_packet["relative_waypoint"]["rel_lon"]

                if self.gps:
                    lat = self.gps.latitude + 0.0001 * rel_lat
                    lon = self.gps.longitude + 0.0001 * rel_lon
                else:
                    lat = 0
                    lon = 0

                threading.Thread(target=self.achieve, args=(lat, lon)).start()

            elif "stop" in ot_packet:
                self.set_estop_pub.publish(True)
                self.reset_estop_pub.publish(False)

        except json.JSONDecodeError:
            print("Failed to decode JSON data")

    def achieve(self, lat, lon):
        try:
            #print("rospy")#"Attempting to achieve GPS waypoint at latitude {lat}, longitude {lon}")
            rospy.wait_for_service("/foddog/achieve_gps_waypoint", timeout=5)
            waypoint_proxy = rospy.ServiceProxy("/foddog/achieve_gps_waypoint", AchieveGPSWaypoint)

            g_point = GpsPoint()
            g_point.lat = lat
            g_point.lon = lon

            self.pub_latlong_target.publish(g_point)
            self.plan_status_pub.publish('executing')

            response = waypoint_proxy(g_point, GpsPoint(), Bool(True))
            #print("rospy")#"Waypoint achieved, response: {response}")
        except rospy.ROSException as e:
            print("fail")
            #print("rospy")#"Failed to achieve waypoint: {e}")

    def send_gps_data_continuously(self):
        while not rospy.is_shutdown() and self.connected:
            if self.gps:
                try:
                    gps_data = {
                        "gps": {
                            "lat": self.gps.latitude,
                            "lon": self.gps.longitude
                        }
                    }
                    json_data = json.dumps(gps_data)
                    if json_data:
                        #print("rospy")#"Sending GPS data to server: {json_data}")
                        self.socket.sendall((json_data + '\n').encode('utf-8'))
                except Exception as e:
                    #print("rospy")#"Failed to send GPS data: {e}")
                    self.handle_disconnect()
                    break
            rospy.sleep(1)

    def handle_disconnect(self):
        print("Handling disconnection from server")
        self.connected = False
        if self.socket:
            print("Closing socket")
            self.socket.close()
        for attempt in range(self.reconnect_attempts):
            #print("rospy")#"Reconnection attempt {attempt + 1}")
            if self.connect():
                return
            rospy.sleep(self.reconnect_delay)
        print("Failed to reconnect after several attempts")

    def run(self):
        while not rospy.is_shutdown():
            if self.connect():
                listen_thread = threading.Thread(target=self.listen_continuously)
                send_thread = threading.Thread(target=self.send_gps_data_continuously)

                listen_thread.start()
                send_thread.start()

                listen_thread.join()
                send_thread.join()

            if not self.connected:
                self.handle_disconnect()

            if rospy.is_shutdown():
                break

        self.cleanup()

    def cleanup(self):
        print("Cleaning up resources")
        self.connected = False
        if self.socket:
            print("Closing socket")
            self.socket.close()

if __name__ == "__main__":
    rospy.init_node('tak_client_bridge')
    server_ip = rospy.get_param('~server_ip', '2001:57b:1200:c08:b933:13cd:9366:55f6')
    server_port = rospy.get_param('~server_port', 8088)

    bridge = TakClientBridge(server_ip, server_port)

    try:
        bridge.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        bridge.cleanup()