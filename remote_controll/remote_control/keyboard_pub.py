#!/usr/bin/env python3
import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


KEYMAP = {
    'z': "D 50 50 1",     # forward
    'a': "D 50 -50 1",    # left
    'e': "D -50 50 1",    # right
    's': "D -50 -50 1",   # backward
}

HELP = """\
Controls:
  z = forward        -> D 50 50 1
  a = left           -> D 50 -50 1
  e = right          -> D -50 50 1
  s = backwards      -> D -50 -50 1
  q = quit

Focus the terminal window and press keys (no Enter needed).
"""


class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')

        # Publishe robotcommando's
        self.publisher = self.create_publisher(String, 'robot_commands', 10)

        self.get_logger().info("Keyboard Publisher gestart")
        self.get_logger().info("\n" + HELP)

        # Setup terminal raw mode
        self._orig_term = termios.tcgetattr(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())

        # Use a timer to poll stdin without blocking ROS spin
        self.key_timer = self.create_timer(0.02, self._poll_keyboard)

    def _poll_keyboard(self):
        """Non-blocking single-key read from stdin; map to serial commands."""
        try:
            rlist, _, _ = select.select([sys.stdin], [], [], 0)
            if rlist:
                ch = sys.stdin.read(1)
                if not ch:
                    return

                if ch == 'q':
                    self.get_logger().info("Quit requested.")
                    rclpy.shutdown()
                    return

                if ch in KEYMAP:
                    cmd = KEYMAP[ch]
                    msg = String()
                    msg.data = cmd
                    self.publisher.publish(msg)
                    self.get_logger().info(f"published: {cmd}")
                elif ch in ['h', '?']:
                    self.get_logger().info("\n" + HELP)

        except Exception as e:
            self.get_logger().warn(f"Keyboard read error: {e}")

    def destroy_node(self):
        # Restore terminal
        try:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self._orig_term)
        except Exception:
            pass

        super().destroy_node()


def main():
    rclpy.init()
    node = KeyboardPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
