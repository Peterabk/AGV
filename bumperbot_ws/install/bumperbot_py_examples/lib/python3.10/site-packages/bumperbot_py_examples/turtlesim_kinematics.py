import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class SimpleTurtlesimKinematics(Node):
    def __init__(self):
            super().__init__("simple_turtlesim_kinematics")

            self.turtle1_pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.turtle1PoseCallback,10)
            self.turtle2_pose_sub = self.create_subscription(Pose, "/turtle2/pose", self.turtle2PoseCallback,10)

            self.last_turtle1_pose = Pose()
            self.last_turtle2_pose = Pose()

    def turtle1PoseCallback(self, msg):
        self.last_turtle1_pose = msg

    def turtle2PoseCallback(self, msg):
        self.last_turtle2_pose = msg
        Tx = self.last_turtle2_pose.x - self.last_turtle1_pose.x
        Ty = self.last_turtle2_pose.y - self.last_turtle1_pose.y

        self.get_logger().info("""\n
                               Transaltion vector turtle1 -> turtle2 \n
                               Tx: %f \n
                               Ty: %f \n
                               """ % (Tx,Ty)
                               )

def main():
    rclpy.init()
    simple_turtlesim_kinematics = SimpleTurtlesimKinematics()
    rclpy.spin(simple_turtlesim_kinematics)
    simple_turtlesim_kinematics.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()