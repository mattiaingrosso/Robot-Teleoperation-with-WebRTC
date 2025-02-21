import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import select
import termios
import tty

class InputNode(Node):
    def __init__(self):
        super().__init__('input_node')
        self.publisher = self.create_publisher(String, '/cmd_key', 10)  # 🔥 Pubblica solo il tasto premuto
        self.original_settings = termios.tcgetattr(sys.stdin)  # 🔥 Salva le impostazioni originali del terminale
        self.get_logger().info("✅ Input Node ROS 2 avviato! Premi un tasto per inviarlo, Ctrl+C per uscire")
        self.run_teleop()

    def get_key(self):
        """Legge un singolo carattere dalla tastiera senza bloccare."""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.original_settings)  # 🔥 Ripristina il terminale
        return key

    def run_teleop(self):
        """Loop per leggere i tasti e pubblicarli su /cmd_key."""
        try:
            while rclpy.ok():
                key = self.get_key()
                if key:
                    msg = String()
                    msg.data = key
                    self.publisher.publish(msg)
                    self.get_logger().info(f"📤 Tasto inviato: {key}")
                rclpy.spin_once(self, timeout_sec=0.1)
        except KeyboardInterrupt:
            self.get_logger().info("🛑 Interruzione con Ctrl+C, chiusura del nodo...")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.original_settings)  # 🔥 Assicura il ripristino del terminale

def main():
    rclpy.init()
    node = InputNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
