import rclpy
from rclpy.node import Node

class MyNode(Node):
    

def main(args=None):
    rclpy.init(args=args)


    rclpy.shutdown()

if __name__=='__main__':
    main()