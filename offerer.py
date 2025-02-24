import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import asyncio
import requests
from aiortc import RTCIceCandidate, RTCPeerConnection, RTCSessionDescription

class KeySubscriberNode(Node):
    def __init__(self):
        super().__init__('key_subscriber_node')
        self.subscription = self.create_subscription(
            String,         # Tipo di messaggio
            '/cmd_key',     # Nome del topic
            self.listener_callback,
            10
        )
        self.get_logger().info("✅ Nodo Subscriber avviato! In attesa di tasti pubblicati su /cmd_key...")
        self.received_key = ""  # Variabile per memorizzare l'ultimo tasto ricevuto

    def listener_callback(self, msg):
        """Callback per raccogliere i tasti ricevuti e salvarli."""
        self.get_logger().info(f"📥 Tasto ricevuto: {msg.data}")
        self.received_key = msg.data  # Aggiorna l'ultimo tasto ricevuto

    def get_received_key(self):
        """Restituisce l'ultimo tasto ricevuto."""
        return self.received_key


# Variabili di configurazione per WebRTC
ip = "10.10.11.44"
port = "6969"
SIGNALING_SERVER_URL = f'http://{ip}:{port}'
ID = "offerer01"


async def main():
    print("Starting")
    peer_connection = RTCPeerConnection()
    channel = peer_connection.createDataChannel("chat")

    # Crea il nodo ROS2 per ascoltare i tasti premuti
    rclpy.init()
    node = KeySubscriberNode()

    # Funzione per inviare i comandi via WebRTC
    async def send_command(channel):
        while True:
            # Ottieni l'ultimo tasto ricevuto dal topic
            key = node.get_received_key()

            # Se c'è un tasto ricevuto, invialo tramite il canale WebRTC
            if key:
                channel.send(key)
                sys.stdout.write("\033[F")  # Torna su una riga
                sys.stdout.write("\033[K")  # Cancella la riga
                print(f"Comando inviato: {key}")
            
            await asyncio.sleep(0.05)  # Breve attesa per evitare un loop eccessivo

    @channel.on("open")
    def on_open():
        channel.send("Connessione stabilita da peer1!")
        asyncio.ensure_future(send_command(channel))

    @channel.on("message")
    async def on_message(message):
        # Gestione dei messaggi ricevuti dal peer
        print(f"Messaggio ricevuto: {message}")
        sys.stdout.write("\033[F")  # Torna su una riga
        sys.stdout.write("\033[K")  # Cancella la riga

    # Invia l'offerta per WebRTC
    await peer_connection.setLocalDescription(await peer_connection.createOffer())
    message = {
        "id": ID, 
        "sdp": peer_connection.localDescription.sdp, 
        "type": peer_connection.localDescription.type
    }
    r = requests.post(SIGNALING_SERVER_URL + '/offer', data=message)
    print(r.status_code)

    # Poll per ricevere la risposta
    while True:
        resp = requests.get(SIGNALING_SERVER_URL + "/get_answer")
        if resp.status_code == 503:
            print("Answer not ready, trying again")
            sys.stdout.write("\033[F")  # Torna su una riga
            sys.stdout.write("\033[F")  # Torna su una riga
            sys.stdout.write("\033[K")  # Cancella la riga
            await asyncio.sleep(1)
        elif resp.status_code == 200:
            data = resp.json()
            if data["type"] == "answer":
                rd = RTCSessionDescription(sdp=data["sdp"], type=data["type"])
                await peer_connection.setRemoteDescription(rd)
                print("Canale aperto!")
                break
            else:
                print("Wrong type")
            break
        print(resp.status_code)

    # Loop per mantenere viva la connessione WebRTC senza bloccare l'ascolto dei messaggi
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)  # Esegui un ciclo di eventi ROS2 non bloccante
        await asyncio.sleep(0.1)  # Mantieni il loop asincrono attivo

    # Dopo aver chiuso il nodo, termina
    node.destroy_node()
    rclpy.shutdown()

# Avvia il ciclo principale
asyncio.run(main())
