print("Im the server!")


from flask import Flask, request, Response
import json


app = Flask(__name__)


data = {}

# collegamento tra un percorso URL e il codice del server
# che deve essere eseguito quando arriva una richiesta a quel percorso
@app.route('/test') #route di test per verificare che il server funzioni correttamente
def test():
    return Response('{"status":"ok"}', status=200, mimetype='application/json')


@app.route('/offer', methods=['POST']) #salva un'offerta WebRTC (contenente l'SDP) nel dizionario condiviso data
# richiesta POST ->
# invia informazioni che devono essere processate o salvate dal server
def offer():
    # Il client invia una richiesta POST a /offer con un payload che include un SDP come parte del corpo della richiesta. Questo rappresenta l'offerta per avviare una connessione WebRTC.
    if request.form["type"] == "offer":
        data["offer"] = {"id" : request.form['id'], "type" : request.form['type'], "sdp" : request.form['sdp']}
        return Response(status=200)
    else:
        return Response(status=400)

@app.route('/answer', methods=['POST']) #salva una risposta WebRTC nel dizionario data
def answer():
    # Un altro client invia una richiesta POST a /answer con il proprio SDP come risposta all'offerta ricevuta.
    if request.form["type"] == "answer":
        data["answer"] = {"id" : request.form['id'], "type" : request.form['type'], "sdp" : request.form['sdp']}
        return Response(status=200)
    else:
        return Response(status=400)

@app.route('/get_offer', methods=['GET']) #Recupera l'offerta salvata (se presente). Una volta recuperata, la elimina dal dizionario.
def get_offer():
    if "offer" in data:
        j = json.dumps(data["offer"])
        del data["offer"]
        return Response(j, status=200, mimetype='application/json')
    else:
        return Response(status=503)

@app.route('/get_answer', methods=['GET']) #Recupera la risposta salvata (se presente). Anche qui, una volta recuperata, viene eliminata dal dizionario.
def get_answer():
    if "answer" in data:
        j = json.dumps(data["answer"])
        del data["answer"]
        return Response(j, status=200, mimetype='application/json')
    else:
        return Response(status=503)





if __name__ == '__main__':
    app.run(host="0.0.0.0", port=6969, debug=True)


