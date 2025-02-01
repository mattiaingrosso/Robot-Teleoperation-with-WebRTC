from aiortc import RTCIceCandidate, RTCPeerConnection, RTCSessionDescription, RTCConfiguration, RTCIceServer
import json
import asyncio
import requests
import keyboard
import sys
import time





#ip =  input("IP: ")
#port = input("Port: ")

ip = "192.168.1.79"
port = "6969"

SIGNALING_SERVER_URL = 'http://'+ip+':'+port
ID = "offerer01"




async def main():
    print("Starting")
    peer_connection = RTCPeerConnection();
    channel = peer_connection.createDataChannel("chat");


    async def send_command(channel):
        while True:
            for key in "abcdefghijklmnopqrstuvwxyz":
                if keyboard.is_pressed(key):
                    channel.send(key)
                    sys.stdout.write("\033[F")  # Torna su una riga
                    sys.stdout.write("\033[K")  # Cancella la riga
                    print(f"Comando inviato: {key}")
                    await asyncio.sleep(0.2)  # Evita spam di invio

            await asyncio.sleep(0.05)  # Breve attesa per evitare un loop eccessivo

    @channel.on("open")
    def on_open():
        channel.send("Connessione stabilita da peer1!")
        asyncio.ensure_future(send_command(channel))





    # send offer
    await peer_connection.setLocalDescription(await peer_connection.createOffer())
    message = {"id": ID, "sdp": peer_connection.localDescription.sdp, "type": peer_connection.localDescription.type}
    r = requests.post(SIGNALING_SERVER_URL + '/offer', data=message)
    print(r.status_code)
    
    #POLL FOR ANSWER
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
                sys.stdout.write("\033[K")  # Cancella la riga
                sys.stdout.write("\033[F")  # Torna su una riga
                sys.stdout.write("\033[K")  # Cancella la riga
                print("Canale aperto!")
                while True:
                    #print("Ready for Stuff")
                    await asyncio.sleep(1)
            else:
                print("Wrong type")
            break

        print(resp.status_code)



asyncio.run(main())
