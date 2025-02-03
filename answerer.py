import requests
from aiortc import RTCPeerConnection, RTCSessionDescription
import asyncio
import math
import numpy as np
import threading
import sys

#----

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor


class TurtleBot3Controller(Node):
    def __init__(self):
        super().__init__('turtlebot3_controller')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.reset_client = self.create_client(Empty, '/reset_world')
        self.twist = Twist()
        self.last_position = None
        self.last_orientation = None

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.get_logger().info("TurtleBot3 Controller avviato. Usa WASD per muoverti, Q per uscire.")

    def set_velocity(self, linear, angular):
        self.twist.linear.x = linear
        self.twist.angular.z = angular
        self.publisher.publish(self.twist)

    def reset_robot(self):
        """Resetta il mondo di Gazebo (tutti gli oggetti)"""
        self.get_logger().info("Chiamata al servizio di reset del mondo...")

        if self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Servizio di reset del mondo trovato.")
            # Chiamata al servizio ResetWorld
            request = Empty.Request()  # ResetWorld non ha bisogno di parametri, quindi usiamo Empty.Request()
            self.reset_client.call_async(request)
            self.get_logger().info("Mondo di Gazebo ripristinato!")
        else:
            self.get_logger().error("Il servizio di reset del mondo non è disponibile!")

    def odom_callback(self, msg):
        """Controlla se il TurtleBot si è ribaltato e stampa la posizione (x, y, z)."""

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # Converti i quaternioni in angoli Euleriani (roll, pitch, yaw)
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        roll, pitch, yaw = quaternion_to_euler(*quaternion)


        self.last_position = (position.x, position.y, position.z)
        self.last_orientation = (roll, pitch, yaw)

        # Se roll o pitch superano ±0.5 radianti (~28°), il robot è ribaltato
        if abs(roll) > 0.5 or abs(pitch) > 0.5:
            self.get_logger().warn("🚨 TurtleBot ribaltato! Reset in corso...")
            # Chiamata sincrona per il reset
            self.reset_robot()

ip = "192.168.1.177"
port = "6969"
SIGNALING_SERVER_URL = f'http://{ip}:{port}'
ID = "answerer01"

def rob_command(key, node):
    """Interpreta il comando ricevuto e imposta la velocità del TurtleBot."""
    commands = {
        'w': (0.5, 0.0, "Avanti"),
        's': (-0.5, 0.0, "Indietro"),
        'a': (0.0, 1.0, "Ruota a sinistra"),
        'd': (0.0, -1.0, "Ruota a destra")
    }

    velocity, angular, command = commands.get(key, (0.0, 0.0, "❌ Comando non valido: STOP"))

    node.set_velocity(velocity, angular)

    sys.stdout.write("\033[F")  # Torna su una riga
    sys.stdout.write("\033[K")  # Cancella la riga

    print(f"Comando ricevuto: {command}")

    if node.last_position and node.last_orientation:
        x, y, z = node.last_position
        roll, pitch, yaw = node.last_orientation
        print(f"📍 Posizione → x: {x:.2f}, y: {y:.2f}, z: {z:.2f}")
        print(f"🔄 Orientazione → Roll: {math.degrees(roll):.2f}°, Pitch: {math.degrees(pitch):.2f}°, Yaw: {math.degrees(yaw):.2f}°")

def quaternion_to_euler(x, y, z, w):
    """Converte un quaternione in angoli RPY (roll, pitch, yaw)."""
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)  # Evita errori numerici fuori dal range [-1,1]
    pitch = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(t3, t4)

    return roll, pitch, yaw

def ros_spin_thread(node):
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()  # Mantiene in esecuzione il nodo ROS2 senza bloccare il resto del codice
    node.destroy_node()
    rclpy.shutdown()

async def main():
    print("Starting WebRTC connection")
    peer_connection = RTCPeerConnection()
    rclpy.init()
    node = TurtleBot3Controller()

    # Avvia ROS2 in un thread separato
    ros_thread = threading.Thread(target=ros_spin_thread, args=(node,))
    ros_thread.start()

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
            message = message.strip().lower()
            rob_command(message, node)

    @peer_connection.on("connectionstatechange")
    async def on_connection_state_change():
        print("Stato della connessione:", peer_connection.connectionState)
        if peer_connection.connectionState in ["disconnected", "failed", "closed"]:
            print("Connessione persa o chiusa. Il robot si fermerà.")
            node.set_velocity(0.0, 0.0)

    try:
        #rclpy.spin(node) NON VA BENE! NON RUNNA PIU WEBRTC
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
