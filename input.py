import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import curses

class InputNode(Node):
    def __init__(self):
        super().__init__('input_node')
        self.publisher = self.create_publisher(String, '/cmd_key', 10)  # Pubblica solo il tasto premuto
        self.get_logger().info("✅ Input Node ROS 2 avviato! Premi un tasto per inviarlo, Ctrl+C per uscire")

        self.key = None  # Variabile per memorizzare il tasto premuto

    def run_teleop(self, stdscr):
        """Loop per leggere i tasti e pubblicarli su /cmd_key."""
        curses.curs_set(0)  # Nasconde il cursore
        stdscr.nodelay(1)  # Non blocca la lettura dell'input
        stdscr.clear()  # Pulisce lo schermo inizialmente
        stdscr.refresh()  # Rende effettiva la pulizia dello schermo

        try:
            while rclpy.ok():
                
                key = stdscr.getch()  # Legge un singolo tasto senza bisogno di Invio

                if key != -1:  # Se è stato premuto un tasto
                    key_char = chr(key) if key >= 32 and key <= 126 else None  # Gestisce tasti validi
                    if key_char:
                        # Pulisce la riga prima di stampare e resetta il cursore
                        stdscr.clear()
                        #stdscr.move(0, 10)  # Reset del cursore all'inizio
                        stdscr.addstr(0, 0, f"📥 Tasto premuto: {key_char}")
                        stdscr.refresh()  # Rende visibile il testo
                        #print(f"📥 Tasto premuto: {key_char}")

                        # Pubblica il tasto premuto sul topic /cmd_key
                        msg = String()
                        msg.data = key_char
                        self.publisher.publish(msg)
                        stdscr.addstr(1, 0, f"📤 Tasto inviato: {key_char}") #unico modo per stampare, aggiornare manualmente la riga

                    elif key == 27:  # Se premi ESC (codice ASCII 27)
                        self.get_logger().info("🛑 ESC premuto, uscita...")
                        break  # Esce dal loop se si preme ESC

                rclpy.spin_once(self, timeout_sec=0.1)  # Gestisce gli eventi ROS2 in modo non bloccante

        except KeyboardInterrupt:
            self.get_logger().info("🛑 Interruzione con Ctrl+C, chiusura del nodo...")

def main():
    rclpy.init()
    node = InputNode()

    # Esegui il loop teleoperativo usando curses
    curses.wrapper(node.run_teleop)  # wrapper gestisce automaticamente l'inizializzazione e la pulizia di curses

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
