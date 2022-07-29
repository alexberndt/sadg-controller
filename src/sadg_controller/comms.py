import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion

class Comms:
    def __init__(self, agent_ns: str):
        self.ns = agent_ns
        
        self.sub_link = f"/{self.ns}/current"
        self.subscriber = rospy.Subscriber(self.sub_link, Pose, self.callback)

        self.pub_link = f"/{self.ns}/goal"
        self.publisher = rospy.Publisher(self.pub_link, Pose, queue_size=1000)


    def callback(self, pose: Pose) -> None:
        """Callback for subscriber. """
        rospy.loginfo(f"{self.sub_link}: Callback: {parse_pose(pose)}")
    

    def publish(self, pose: Pose) -> None:
        """Publish Pose"""
        rospy.loginfo(f"{self.pub_link}: Publishing: {parse_pose(pose)}")
        self.publisher.publish(pose)


def parse_pose(pose: Pose) -> str:
    """Converts Pose to printable string."""

    position_str = f"Position({pose.position.x},{pose.position.y},{pose.position.z})"
    quaternion_str = f"Quaternion({pose.orientation.x},{pose.orientation.y},{pose.orientation.z},{pose.orientation.w})"

    return f"[{position_str}, {quaternion_str}]"
