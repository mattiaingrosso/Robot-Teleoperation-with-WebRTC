from aiortc import RTCIceCandidate, RTCPeerConnection, RTCSessionDescription, RTCConfiguration, RTCIceServer
import json
import asyncio
import requests
import sys
import tty
import termios

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    except KeyboardInterrupt:
        # Ripristina la modalità terminale in caso di interruzione
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        raise  # Rilancia l'eccezione per gestire Ctrl+C
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key



#ip =  input("IP: ")
#port = input("Port: ")

ip = "192.168.1.122"
port = "6969"

SIGNALING_SERVER_URL = 'http://'+ip+':'+port
ID = "offerer01"




async def main():
    print("Starting")
    peer_connection = RTCPeerConnection();
    channel = peer_connection.createDataChannel("chat");


    async def send_command(channel):
        while True:
            print("Premi un tasto: ")
            msg = get_key()
            channel.send(msg.lower())
            await asyncio.sleep(0.2) #più piccolo è il tempo, più sensibile è l'input

    @channel.on("open")
    def on_open():
        channel.send("Connessione stabilita da peer1!")
        asyncio.ensure_future(send_command(channel))

    @channel.on("message")
    def on_message(message):
        print("Received via RTC Datachannel", message)



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
            await asyncio.sleep(1)
        elif resp.status_code == 200:
            data = resp.json()
            if data["type"] == "answer":
                rd = RTCSessionDescription(sdp=data["sdp"], type=data["type"])
                await peer_connection.setRemoteDescription(rd)
                print("Canale aperto!")
                while True:
                    #print("Ready for Stuff")
                    await asyncio.sleep(1)
            else:
                print("Wrong type")
            break

        print(resp.status_code)



asyncio.run(main())