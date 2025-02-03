import requests
from aiortc import RTCIceCandidate, RTCPeerConnection, RTCSessionDescription
import asyncio
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty  # Per resettare Gazebo
from nav_msgs.msg import Odometry  # Per rilevare ribaltamenti

class TurtleBot3Controller(Node):
    def __init__(self):
        super().__init__('turtlebot3_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.reset_client = self.create_client(Empty, '/reset_simulation')
        self.twist = Twist()

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.get_logger().info("TurtleBot3 Controller avviato. Usa WASD per muoverti, Q per uscire.")

    def set_velocity(self, linear, angular):
        self.twist.linear.x = linear
        self.twist.angular.z = angular
        self.publisher.publish(self.twist)

    def reset_robot(self):
        if self.reset_client.wait_for_service(timeout_sec=1.0):
            req = Empty.Request()
            self.reset_client.call_async(req)
            self.get_logger().info("Reset di Gazebo eseguito!")

    def odom_callback(self, msg):
        # Controllo se il TurtleBot è ribaltato
        orientation = msg.pose.pose.orientation
        if abs(orientation.x) > 0.5 or abs(orientation.y) > 0.5:  # Controllo ribaltamento
            self.get_logger().warn("Il TurtleBot si è ribaltato! Reset in corso...")
            self.reset_robot()

ip = "192.168.1.177"
port = "6969"
SIGNALING_SERVER_URL = f'http://{ip}:{port}'
ID = "answerer01"

def rob_command(key, node):
    commands = {
        'w': (0.5, 0.0, "Avanti"),
        's': (-0.5, 0.0, "Indietro"),
        'a': (0.0, 1.0, "Ruota a sinistra"),
        'd': (0.0, -1.0, "Ruota a destra")
    }

    velocity, angular, command = commands.get(key, (0.0, 0.0, "Comando non valido: STOP"))

    node.set_velocity(velocity, angular)

    sys.stdout.write("\033[F")  # Torna su una riga
    sys.stdout.write("\033[K")  # Cancella la riga
    print("Comando ricevuto: " + command)

async def main():
    print("Starting WebRTC connection")
    peer_connection = RTCPeerConnection()
    rclpy.init()
    node = TurtleBot3Controller()

    async def send_keep_alive(channel):
        while True:
            await asyncio.sleep(3)
            if channel.readyState == "open":
                channel.send("keep-alive")

    @peer_connection.on("datachannel")
    def on_datachannel(channel):
        print("Canale aperto!")
        channel.send("Connessione stabilita con peer2!")
        asyncio.create_task(send_keep_alive(channel))

        @channel.on("message")
        async def on_message(message):
            message = message.strip().lower()  # Normalizza l'input
            rob_command(message, node)

    @peer_connection.on("connectionstatechange")
    async def on_connection_state_change():
        print("Stato della connessione:", peer_connection.connectionState)
        if peer_connection.connectionState in ["disconnected", "failed", "closed"]:
            print("Connessione persa o chiusa. Il robot si fermerà.")
            node.set_velocity(0.0, 0.0)

    try:
        resp = requests.get(SIGNALING_SERVER_URL + "/get_offer")
        if resp.status_code == 200:
            data = resp.json()
            if data["type"] == "offer":
                rd = RTCSessionDescription(sdp=data["sdp"], type=data["type"])
                await peer_connection.setRemoteDescription(rd)
                await peer_connection.setLocalDescription(await peer_connection.createAnswer())

                message = {"id": ID, "sdp": peer_connection.localDescription.sdp, "type": peer_connection.localDescription.type}
                requests.post(SIGNALING_SERVER_URL + '/answer', data=message)

                print("Canale WebRTC pronto!")
                while True:
                    await asyncio.sleep(1)

    except Exception as e:
        print(f"Errore WebRTC: {e}")
        node.set_velocity(0.0, 0.0)

asyncio.run(main())
