import requests
from aiortc import RTCIceCandidate, RTCPeerConnection, RTCSessionDescription, RTCConfiguration, RTCIceServer
import asyncio
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleBot3Controller(Node):
    def __init__(self):
        super().__init__('turtlebot3_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist = Twist()
        self.get_logger().info("TurtleBot3 Controller avviato. Usa WASD per muoverti, Q per uscire.")

    def set_velocity(self, linear, angular):
        self.twist.linear.x = linear
        self.twist.angular.z = angular
        self.publisher.publish(self.twist)


#ip =  input("IP: ")
#port = input("Port: ")

ip = "192.168.1.177"
port = "6969"

SIGNALING_SERVER_URL = 'http://'+ip+':'+port
ID = "answerer01"

def rob_command(key, node):
    if key == 1:
        if key == 'w':
            node.set_velocity(0.5, 0.0)  # Avanti
            command = "Avanti"
        elif key == 's':
            node.set_velocity(-0.5, 0.0)  # Indietro
            command = "Indietro"
        elif key == 'a':
            node.set_velocity(0.0, 1.0)  # Ruota a sinistra
            command = "Ruota a sinistra"
        elif key == 'd':
            node.set_velocity(0.0, -1.0)  # Ruota a destra
            command = "Ruota a destra"
        else:
            node.set_velocity(0.0, 0.0)  # Ferma il robot
            command = "non valido"
        sys.stdout.write("\033[F")  # Torna su una riga
        sys.stdout.write("\033[K")  # Cancella la riga
        print("Comando ricevuto: "+command)


async def main():
    print("Starting")
    peer_connection = RTCPeerConnection()

    rclpy.init()
    node = TurtleBot3Controller()

    async def send_keep_alive(channel):
        while True:
            await asyncio.sleep(3)  # Invia un keep-alive ogni 3 secondi
            if channel.readyState == "open":
                channel.send("keep-alive")

    @peer_connection.on("datachannel")
    def on_datachannel(channel):
        print("Canale aperto!")
        channel.send("Connessione stabilita con peer2!")
        asyncio.create_task(send_keep_alive(channel))

        @channel.on("message")
        async def on_message(message):
            rob_command(message, node)

    @peer_connection.on("connectionstatechange")
    async def on_connection_state_change():
        print("Stato della connessione:", peer_connection.connectionState)
        if peer_connection.connectionState == "disconnected":
            print("Connessione temporaneamente persa.")
        elif peer_connection.connectionState == "failed":
            print("Connessione fallita.")
        elif peer_connection.connectionState == "closed":
            print("Connessione chiusa.")
        node.set_velocity(0.0, 0.0)


    command = " "

    resp = requests.get(SIGNALING_SERVER_URL + "/get_offer")

    print(resp.status_code)
    if resp.status_code == 200:
        data = resp.json()
        if data["type"] == "offer":
            rd = RTCSessionDescription(sdp=data["sdp"], type=data["type"])
            await peer_connection.setRemoteDescription(rd)
            await peer_connection.setLocalDescription(await peer_connection.createAnswer())

            message = {"id": ID, "sdp": peer_connection.localDescription.sdp, "type": peer_connection.localDescription.type}
            r = requests.post(SIGNALING_SERVER_URL + '/answer', data=message)
            print("Canale aperto!")
            while True:

                await asyncio.sleep(1)



asyncio.run(main())
