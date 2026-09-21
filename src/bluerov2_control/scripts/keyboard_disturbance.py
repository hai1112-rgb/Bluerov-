#!/usr/bin/env python3
"""
BlueROV2 Keyboard Disturbance Controller
=========================================
Control the simulated environment with single-key shortcuts:

  S  = Storm             - large gust disturbance
  C  = North current     - current heading North (+Y)
  E  = East current      - current heading East (+X)
  W  = Strong current    - strong northbound current
  N  = Normal            - disable all disturbance modes
  Q  = Quit

Publishes to: /disturbance/mode  (std_msgs/String)
  Values: "normal" | "storm" | "current_n" | "current_e" | "current_strong"

Run:
  ros2 run bluerov2_control keyboard_disturbance
  or:
  python3 keyboard_disturbance.py
"""

import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


HELP_TEXT = """
╔══════════════════════════════════════════════════════╗
║   BlueROV2 Keyboard Disturbance Controller           ║
╠══════════════════════════════════════════════════════╣
║  S  -> Storm - strong gust force                    ║
║  C  -> North current                                ║
║  E  -> East current                                 ║
║  W  -> Strong north current                         ║
║  N  -> Normal - disable disturbance                 ║
║  Q  -> Quit                                         ║
╚══════════════════════════════════════════════════════╝
"""

MODE_MAP = {
    's': 'storm',
    'c': 'current_n',
    'e': 'current_e',
    'w': 'current_strong',
    'n': 'normal',
}

MODE_DESC = {
    'storm':          'STORM - randomized 8 N gust force',
    'current_n':      'NORTH CURRENT - 4 N-equivalent current toward +Y',
    'current_e':      'EAST CURRENT - 4 N-equivalent current toward +X',
    'current_strong': 'STRONG CURRENT - 7 N-equivalent current toward +Y',
    'normal':         'NORMAL - disturbance disabled',
}


class KeyboardDisturbanceNode(Node):
    def __init__(self):
        super().__init__('keyboard_disturbance_controller')
        self.pub = self.create_publisher(String, '/disturbance/mode', 10)
        self.current_mode = 'normal'
        self.get_logger().info('Keyboard Disturbance Controller started')

    def publish_mode(self, mode: str):
        msg = String()
        msg.data = mode
        self.pub.publish(msg)
        self.current_mode = mode
        print(f'\n  → {MODE_DESC.get(mode, mode)}\n')


def get_key(settings):
    """Read one terminal character without requiring Enter."""
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    rclpy.init()
    node = KeyboardDisturbanceNode()

    print(HELP_TEXT)
    print('  Current mode: NORMAL\n')
    print('  Press a key without Enter:')

    settings = termios.tcgetattr(sys.stdin)

    try:
        # Publish initial normal state
        node.publish_mode('normal')

        while rclpy.ok():
            key = get_key(settings)
            k = key.lower()

            if k == 'q':
                print('\n  Exiting...')
                node.publish_mode('normal')  # Reset to normal before exit.
                break
            elif k in MODE_MAP:
                mode = MODE_MAP[k]
                node.publish_mode(mode)
            else:
                print(f'  Invalid key: {repr(key)} '
                      '(use S/C/E/W/N/Q)')

            # Spin once so ROS callbacks are processed.
            rclpy.spin_once(node, timeout_sec=0.0)

    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
