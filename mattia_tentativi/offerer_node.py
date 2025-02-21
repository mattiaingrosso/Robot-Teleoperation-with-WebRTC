import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncio
import requests
import sys
from aiortc import RTCPeerConnection, RTCSessionDescription

# Configurazione del Signaling Server
IP = "192.168.1.122"  # 🔥 IP aggiornato per il nuovo Offerer
PORT = "6969"
SIGNALING_SERVER_URL = f'http://{IP}:{PORT}'
ID = "offerer_ros2"

class OffererNode(Node):
    def __init__(self):
        super().__init__('offerer_node')
        self.subscription = self.create_subscription(String, '/cmd_key', self.cmd_key_callback, 10)

        # Creazione della connessione WebRTC
        self.peer_connection = RTCPeerConnection()
        self.channel = None  # 🔥 Il canale verrà creato solo dopo l'handshake
        self.connected = False  # 🔥 Flag per sapere quando la connessione è stabilita

        self.get_logger().info("✅ Offerer Node ROS 2 avviato!")

        # Stato connessione WebRTC
        @self.peer_connection.on("connectionstatechange")
        def on_connection_state_change():
            self.get_logger().info(f"📡 Stato WebRTC: {self.peer_connection.connectionState}")
            if self.peer_connection.connectionState == "connected":
                self.get_logger().info("✅ WebRTC stabilito con Answerer! Il canale è pronto!")
                self.connected = True  # 🔥 Ora possiamo inviare dati

    def cmd_key_callback(self, msg):
        """Riceve un tasto da /cmd_key e lo invia via WebRTC SOLO se pronto."""
        if self.channel and self.channel.readyState == "open" and self.connected:
            self.channel.send(msg.data)
            self.get_logger().info(f"📤 Tasto inviato via WebRTC: {msg.data}")
        else:
            self.get_logger().warn("⚠️ Canale WebRTC non pronto, impossibile inviare!")

async def main():
    rclpy.init()
    node = OffererNode()

    try:
        # 🔥 Crea il canale WebRTC PRIMA di inviare l'offerta
        node.channel = node.peer_connection.createDataChannel("cmd_channel")
        node.get_logger().info("✅ Canale WebRTC creato!")

        @node.channel.on("open")
        def on_open():
            """Invia un messaggio quando il canale si apre."""
            node.channel.send("🔗 Connessione stabilita!")
            node.get_logger().info("🔗 Canale WebRTC aperto e funzionante!")

        @node.channel.on("message")
        def on_message(message):
            """Gestisce la ricezione di dati dal canale WebRTC."""
            node.get_logger().info(f"📩 Messaggio ricevuto: {message}")

        # 🔥 Crea un'offerta SDP e la invia al Signaling Server
        await node.peer_connection.setLocalDescription(await node.peer_connection.createOffer())
        message = {"id": ID, "sdp": node.peer_connection.localDescription.sdp, "type": node.peer_connection.localDescription.type}
        r = requests.post(SIGNALING_SERVER_URL + '/offer', json=message)
        node.get_logger().info(f"📡 Offerta inviata, status code: {r.status_code}")

        # 🔥 Aspetta la risposta (answer) dall'Answerer
        while True:
            resp = requests.get(SIGNALING_SERVER_URL + "/get_answer")
            if resp.status_code == 200:
                data = resp.json()
                rd = RTCSessionDescription(sdp=data["sdp"], type=data["type"])
                await node.peer_connection.setRemoteDescription(rd)
                node.get_logger().info("✅ WebRTC stabilito con Answerer!")
                break
            elif resp.status_code == 503:
                node.get_logger().warn("❌ Answer non pronto, ritento...")
            await asyncio.sleep(1)

        rclpy.spin(node)

    except requests.RequestException as e:
        node.get_logger().error(f"❌ Errore di connessione: {e}")
    finally:
        await node.peer_connection.close()
        rclpy.shutdown()

if __name__ == "__main__":
    asyncio.run(main())
