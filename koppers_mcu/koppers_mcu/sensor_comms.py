import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('serial_publisher')
    publisher = node.create_publisher(String, 'sensor_data', 100)
    # serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

    def timer_callback():
        # if serial_port.in_waiting > 0:
        #     serial_data = serial_port.readline().decode('utf-8').strip()
        msg = String()
        msg.data = "hw"#serial_data
        publisher.publish(msg)
        node.get_logger().info(f'Publishing: {msg.data}')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()