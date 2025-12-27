#!/usr/bin/env python3
import threading
import time
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RobotSubscriber(Node):
    def __init__(self):
        super().__init__('robot_subscriber')

        # Parameters (with your requested defaults)
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 57600)
        self.declare_parameter('timeout', 0.1)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        timeout = self.get_parameter('timeout').get_parameter_value().double_value

        # Open serial
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)
        except Exception as e:
            self.get_logger().error(f"Failed to open serial {port} @ {baud}: {e}")
            raise

        # Let MCU (e.g., Arduino) reset if needed
        time.sleep(2.0)
        self.get_logger().info(f"Connected to {port} @ {baud}")

        # Start background thread for reading serial responses
        self._stop_event = threading.Event()
        self.reader_thread = threading.Thread(target=self._serial_reader_loop, daemon=True)
        self.reader_thread.start()

        # Subscriber voor robotcommando's
        self.subscription = self.create_subscription(
            String,
            'robot_commands',
            self.command_callback,
            10
        )

        self.get_logger().info("Robot Subscriber gestart,  wacht op commando's...")

    def command_callback(self, msg):
        """Ontvang commando en stuur naar seriële poort."""
        cmd = msg.data
        try:
            self.ser.write((cmd + "\n").encode())
            self.get_logger().info(f"> {cmd}")
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def _serial_reader_loop(self):
        """Continuously read lines from serial and print them."""
        while not self._stop_event.is_set():
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode(errors='ignore').strip()
                    if line:
                        self.get_logger().info(f"Robot response: {line}")
                else:
                    # small sleep to avoid busy loop
                    time.sleep(0.01)
            except Exception as e:
                self.get_logger().warn(f"Keyboard read error: {e}")
                time.sleep(0.1)

    def destroy_node(self):
        # Stop threads and close serial
        self._stop_event.set()
        try:
            if self.reader_thread.is_alive():
                self.reader_thread.join(timeout=0.5)
        except Exception:
            pass

        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

        super().destroy_node()


def main():
    rclpy.init()
    node = RobotSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
