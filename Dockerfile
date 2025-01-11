Dockerfile funzionante:
# Usa l'immagine base ufficiale di ROS 2 Humble
FROM ros:humble

# Imposta l'ambiente per evitare input interattivi durante l'installazione
ENV DEBIAN_FRONTEND=noninteractive

# Aggiorna i pacchetti e installa dipendenze generali
RUN apt update && apt upgrade -y && apt install -y \
    lsb-release \
    gnupg2 \
    curl \
    build-essential \
    software-properties-common \
    x11-apps \
    mesa-utils \
    net-tools \
    iputils-ping \
    nano \
    sudo \
    python3-pip

# Aggiungi il repository per Gazebo
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    curl -sSL http://packages.osrfoundation.org/gazebo.key | apt-key add -

# Installa ROS 2 pacchetti per Gazebo, TurtleBot3 e teleop
RUN apt update && apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-simulations \
    ros-humble-teleop-twist-keyboard

# Installa le dipendenze Python
RUN pip install --no-cache-dir \
    aiortc \
    requests \
    keyboard \
    flask

# Imposta il modello TurtleBot3 (può essere modificato a burger, waffle, waffle_pi)
ENV TURTLEBOT3_MODEL=burger

# Imposta il ROS 2 locale per Humble
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Imposta il forwarding X11 per la GUI di Gazebo
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1

# Imposta il comando di default per entrare nel bash shell
CMD ["bash"]
