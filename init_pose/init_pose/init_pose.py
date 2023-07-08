
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

pose_initialized = True

class InitialPosePubshiler(Node):

    def __init__(self):
        super().__init__('initpose_publisher')
        self.initpose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.InitializePosition()

    def InitializePosition(self):
        global pose_initialized
        print("setting up the initial position")
        pose_initial = PoseWithCovarianceStamped()
        pose_initial.header.stamp = self.get_clock().now().to_msg()
        pose_initial.header.frame_id = 'map'
        pose_initial.pose.pose.position.x = 0.0
        pose_initial.pose.pose.position.y = 0.0
        pose_initial.pose.pose.position.z = 0.0 
        pose_initial.pose.pose.orientation.x = 0.0
        pose_initial.pose.pose.orientation.y = 0.0
        pose_initial.pose.pose.orientation.z = 0.707
        pose_initial.pose.pose.orientation.w = 0.707
        #for i in range(5):
        self.initpose_publisher.publish(pose_initial)
        #time.sleep(0.1)
        #print("setting initial pose")
        pose_initialized = True


def main(args=None):
    rclpy.init(args=args)

    initpose_publisher = InitialPosePubshiler()

    rclpy.spin_once(initpose_publisher)
    initpose_publisher.destroy_node()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rclpy.shutdown()


if __name__ == '__main__':
    main()