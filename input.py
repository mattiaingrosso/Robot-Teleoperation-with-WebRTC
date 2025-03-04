import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import curses
import time

import sys
import termios

class InputNode(Node):
    def __init__(self):
        super().__init__('input_node')
        self.publisher = self.create_publisher(String, '/cmd_key', 10)  # Pubblica come topic solo il tasto premuto
        self.get_logger().info("Nodo Input ROS 2: Premi un comando")

        self.key = None  # Variabile per memorizzare il tasto premuto
        self.last_time = time.asctime()

    def run_teleop(self, stdscr):
        """Loop di ingresso per pubblicazione su /cmd_key."""
        curses.curs_set(0)  # Nasconde il cursore
        stdscr.nodelay(1)  # Lettura dell'input non bloccante
        stdscr.clear()  # Pulizia
        stdscr.refresh()  # Refresha lo schermo

        try:
            while rclpy.ok():

                stdscr.addstr(0, 0, "Premere WSAD per controllare il robot, R per resettare la simulazione, ESC per uscire")
                key = stdscr.getch()  # Lettura senza bisogno di mettere Invio

                if key != -1:  # Se premo

                    key_char = chr(key) if key >= 32 and key <= 126 else None  # Gestisce tasti validi con gli ASCII

                    if key_char:
                        # Pulisce la riga prima di stampare e resetta il cursor
                        stdscr.clear()
                        stdscr.addstr(1, 0, f"Tasto premuto: {key_char}")
                        stdscr.refresh()
                        # Pubblicazione di key_char sul topic
                        msg = String()
                        self.last_time = str(time.asctime())
                        msg.data = key_char+" "+ self.last_time
                        self.publisher.publish(msg)
                        stdscr.addstr(2, 0, f"Tasto inviato: {msg.data}") # unico modo per stampare, aggiornare manualmente la riga


                    elif key == 27:  # Se premi ESC (codice ASCII 27)
                        self.get_logger().info("ESC premuto, chiudo il nodo..")
                        break  # Esce dal loop se si preme ESC

                else:
                    msg = String()
                    msg.data = "@"+self.last_time
                    self.publisher.publish(msg)



                rclpy.spin_once(self, timeout_sec=0.1)  # Gestisce gli eventi ROS2 in modo non bloccante
                termios.tcflush(sys.stdin, termios.TCIFLUSH)  # Svuota il buffer di input
                time.sleep(0.1)

        except KeyboardInterrupt:
            self.get_logger().info("Keyboard interrupt, chiusura del nodo...")

def main():
    rclpy.init()
    node = InputNode()

    # Esegui il loop teleoperativo usando curses
    curses.wrapper(node.run_teleop)  # wrapper gestisce automaticamente l'inizializzazione e la pulizia di curses

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
