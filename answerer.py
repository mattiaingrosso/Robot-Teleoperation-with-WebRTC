import requests
from aiortc import RTCPeerConnection, RTCSessionDescription
import asyncio
import math
import numpy as np
import threading
import time
import os
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor

# Variabili di configurazione per WebRTC
ip = "192.168.1.122"
port = "6969"
SIGNALING_SERVER_URL = f'http://{ip}:{port}'
ID = "answerer01"

class TurtleBot3Controller(Node):
    def __init__(self):
        super().__init__('turtlebot3_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.reset_client = self.create_client(Empty, '/reset_world')
        self.twist = Twist()
        self.last_command = "Attesa comandi..."

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.last_update_time = time.time()

    def set_velocity(self, linear, angular):
        """Imposta la velocità del TurtleBot."""
        self.twist.linear.x = linear
        self.twist.angular.z = angular
        self.publisher.publish(self.twist)

    def reset_robot(self):
        """Resetta il mondo di Gazebo."""
        self.get_logger().info("Reset del mondo in corso...")
        if self.reset_client.wait_for_service(timeout_sec=1.0):
            request = Empty.Request()
            self.reset_client.call_async(request)
            self.get_logger().info("Mondo Gazebo ripristinato!")
        else:
            self.get_logger().error("Il servizio di reset del mondo non è disponibile!")


    def odom_callback(self, msg):
        """Aggiorna posizione e orientazione con una stampa FLUIDA su riga."""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        roll, pitch, yaw = quaternion_to_euler(*quaternion)

        self.last_position = (position.x, position.y, position.z)
        self.last_orientation = (roll, pitch, yaw)
        sys.stdout.flush()  # Forza l'aggiornamento immediato

#---------------------------------------------------------------------------

def rob_command(key, node):
    """Gestisce i comandi e aggiorna la stampa direttamente da /odom."""
    commands = {
        'w': (0.5, 0.0, "Avanti"),
        's': (-0.5, 0.0, "Indietro"),
        'a': (0.0, 1.0, "Ruota a sinistra"),
        'd': (0.0, -1.0, "Ruota a destra"),
        '@': (0.0, 0.0, node.last_command),
        'r': (0.0, 0.0, "Reset mondo")
    }

    if key in commands:
        velocity, angular, command = commands[key]
        if key == 'r':
            node.reset_robot()
    else:
        velocity, angular, command = 0.0, 0.0, "STOP"



    node.set_velocity(velocity, angular)
    node.last_command = command  # L'output sarà aggiornato al prossimo odom_callback




def quaternion_to_euler(x, y, z, w):
    """Converte un quaternione in angoli RPY (roll, pitch, yaw)."""
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(t3, t4)

    return roll, pitch, yaw

def ros_spin_thread(node):
    """Mantiene ROS2 in esecuzione in un thread separato."""
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

async def main():

    odom = "" #dichiarazione variabile per i dat di odom

    os.system('clear')
    print("Avvio connessione WebRTC...")
    peer_connection = RTCPeerConnection()
    rclpy.init()
    node = TurtleBot3Controller()

    ros_thread = threading.Thread(target=ros_spin_thread, args=(node,))
    ros_thread.start()

    async def send_rob_data(channel):
        while True:
            await asyncio.sleep(0.01)
            if channel.readyState == "open":
                channel.send(odom)

    @peer_connection.on("datachannel")
    def on_datachannel(channel):
        print("\nCanale aperto!")
        channel.send("Connessione stabilita con peer2!")
        asyncio.create_task(send_rob_data(channel))

        @channel.on("message")
        async def on_message(message):
            message = message.strip().lower()
            rob_command(message, node)


    @peer_connection.on("connectionstatechange")
    async def on_connection_state_change():
        print("\n🔄 Stato della connessione:", peer_connection.connectionState)
        if peer_connection.connectionState in ["disconnected", "failed", "closed"]:
            print("Connessione persa o chiusa. Il robot si fermerà.")
            node.set_velocity(0.0, 0.0)


    #Connessione WebRTC
    try:
        resp = requests.get(SIGNALING_SERVER_URL + "/get_offer", timeout=5)
        if resp.status_code == 200:
            data = resp.json()
            if data["type"] == "offer":
                rd = RTCSessionDescription(sdp=data["sdp"], type=data["type"])
                await peer_connection.setRemoteDescription(rd)
                await peer_connection.setLocalDescription(await peer_connection.createAnswer())

                message = {"id": ID, "sdp": peer_connection.localDescription.sdp, "type": peer_connection.localDescription.type}
                requests.post(SIGNALING_SERVER_URL + '/answer', data=message)

                print("✅ Canale WebRTC pronto!")
                while True:
                    odom = "📍Posizione: x="+format(node.last_position[0], ".2f")+", y="+format(node.last_position[1], ".2f")+" | 🔄Yaw = "+format(math.degrees(node.last_orientation[2]), ".2f")+"° |⌨️ Comando: "+str(node.last_command)
                    print(odom)
                    await asyncio.sleep(0.01)
                    sys.stdout.write("\033[F")  # Torna su una riga
                    sys.stdout.write("\033[K")  # Cancella la riga


    except requests.RequestException as e:
        print(f"Errore di connessione: {e}")
        node.set_velocity(0.0, 0.0)
    finally:
        print("\nChiusura in corso...")
        node.set_velocity(0.0, 0.0)
        peer_connection.close()
        rclpy.shutdown()

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(main())
    except KeyboardInterrupt:
        print("\nInterruzione manuale, chiusura del programma...")
