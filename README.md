# Robot-Teleoperation-with-WebRTC
Dopo aver avviato docker montare l'immagine usando il dockerfile:
docker build -t ros2-volume-aiortc -f Dockerfile.txt .

Una volta pronta l'immagine lanciare 4 container:
docker run -it --rm   --env="DISPLAY=$DISPLAY"   --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"   --privileged   -v /home/studenti/Progetto:/ros2_data   ros2-volume-aiortc

Si può sfruttare un volume per avere gli script pronti su ros2 caricandoli in /home/studenti/Progetto

Usare il primo container per runnare gazebo:
ros2 launch turtlebot3_gazebo empty_world.launch.py

Probabilmente la prima volta gazebo si aprirà senza il turtlebot. In ordine bisogna chiudere la finestra di gazebo, fare Ctrl+c sul nodo e lanciare di nuovo il comando.

Una volta che Gazebo è avviato correttamente lanciare in locale signaling.py

Nel primo nodo lanciare lo script offerer.py e digitare l'ip visualizzato su signaling.py:
python3 ros2_data/offerer.py

Esso funziona correttamente se rimane in attesa del secondo nodo.

Dopodichè avviare answerer.py non dimenticando di inserire lo stesso ip:
python3 ros2_data/answerer.py

Su entrambi i nodi arriverà una conferma che la connessione è avvenuta ed arriveranno messaggi sulla posizione del robot.

Infine è possibile avviare sull'ultimo nodo input.py:
python3 ros2_data/input.py

Il robot è pronto per essere teleoperato!
