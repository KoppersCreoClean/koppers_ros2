import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String

class ArduinoRelayNode(Node):
    def __init__(self):
        super().__init__('arduino_relay_node')
        self.publisher_ = self.create_publisher(String, 'arduino_data', 10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if self.serial_port.in_waiting > 0:
            data = self.serial_port.readline().decode('utf-8').strip()
            msg = String()
            msg.data = data
            self.publisher_.publish(msg)
            self.get_logger().info(f'Received: {data}')

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()