#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64

import sys, select, termios, tty

msg = """
Reading from the keyboard and Publishing to Thrusters !
---------------------------
Moving around:
        z     
   q    s    d
        x     

z/x : increase/decrease speed by 10%
q/d : increase/decrease orientation by 5%
s   : reset all to 0

CTRL-C to quit
"""

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.left_speed_pub = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 5)
        self.right_speed_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 5)
        self.left_turn_pub = self.create_publisher(Float64, '/wamv/thrusters/left/pos', 5)
        self.right_turn_pub = self.create_publisher(Float64, '/wamv/thrusters/right/pos', 5)
        self.main_turn_pub = self.create_publisher(Float64, '/wamv/thrusters/main/pos', 5)
        self.main_speed_pub = self.create_publisher(Float64, '/wamv/thrusters/main/thrust', 5)
        self.get_logger().info('Teleop Keyboard Node Started')
        print(msg)
        self.settings = termios.tcgetattr(sys.stdin)
        self.run()

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def vels(self, speed, turn):
        return "currently:\tspeed %s\tturn %s " % (speed,turn)
    
    def run(self):
        speed = 0.0
        turn = 0.0
        speed_limit = 12000.0
        turn_limit = 0.78539816339
        try:
            print(self.vels(speed,turn))
            while(1):
                key = self.getKey()
                if key == "s":
                    speed = 0.0
                    turn = 0.0
                    print(self.vels(speed,turn))
                elif key == "z":
                    speed = min(speed_limit, speed + speed_limit * 0.1)
                    print(self.vels(speed,turn))
                    if speed == speed_limit:
                        print("Speed max limit reached!")
                elif key == "d":
                    turn = min(turn_limit, turn + turn_limit * 0.05)
                    print(self.vels(speed,turn))
                    if turn == turn_limit:
                        print("Orientation max limit reached!")
                elif key == "x":
                    speed = max(-speed_limit, speed - speed_limit * 0.1)
                    print(self.vels(speed,turn))
                    if speed == speed_limit:
                        print("Speed min limit reached!")
                elif key == "q":
                    turn = max(-turn_limit, turn - turn_limit * 0.05)
                    print(self.vels(speed,turn))
                    if turn == turn_limit:
                        print("Orientation min limit reached!")
                elif key == ' ' or key == 's':
                    speed = 0.0
                    turn = 0.0
                elif key == '\x03':
                    break

                speed_msg = Float64()
                turn_msg = Float64()
                speed_msg.data = speed
                turn_msg.data = turn
                
                self.left_speed_pub.publish(speed_msg)
                self.right_speed_pub.publish(speed_msg)
                self.left_turn_pub.publish(turn_msg)
                self.right_turn_pub.publish(turn_msg)
                self.main_turn_pub.publish(turn_msg)
                self.main_speed_pub.publish(speed_msg)

        except Exception as e:
            print(e)

        finally:
            msg = Float64()
            msg.data = 0.0
            print(self.vels(0.0,0.0))
            self.left_speed_pub.publish(msg)
            self.right_speed_pub.publish(msg)
            self.left_turn_pub.publish(msg)
            self.right_turn_pub.publish(msg)
            self.main_turn_pub.publish(msg)
            self.main_speed_pub.publish(msg)

            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)

    teleop_keyboard = TeleopKeyboard()

    rclpy.spin(teleop_keyboard)
    
    teleop_keyboard.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()