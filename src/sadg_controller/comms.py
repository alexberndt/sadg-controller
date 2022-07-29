import rospy
from sadg_controller.sadg.vertex import Vertex
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion

class Comms:
    def __init__(self, agent_ns: str, vertex: Vertex):
        self.ns = agent_ns
        
        self.sub_link = f"/{self.ns}/current"
        self.subscriber = rospy.Subscriber(self.sub_link, Pose, self.callback)

        self.pub_link = f"/{self.ns}/goal"
        self.publisher = rospy.Publisher(self.pub_link, Pose, queue_size=1000)

        self.current_vertex = vertex


    def get_agent_id(self) -> str:
        return self.ns


    def get_curr_vertex(self) -> Vertex:
        return self.current_vertex


    def callback(self, pose: Pose) -> None:
        """Callback for subscriber. """
        rospy.loginfo(f"{self.sub_link}: Callback: {parse_pose(pose)}")
        x = self.current_vertex.get_goal_loc().x
        y = self.current_vertex.get_goal_loc().y
        if pose.position.x == x and pose.position.y == y:
            self.current_vertex = self.current_vertex.get_next()
    

    def publish(self, pose: Pose) -> None:
        """Publish Pose. """
        rospy.loginfo(f"{self.pub_link}: Publishing: {parse_pose(pose)}")
        self.publisher.publish(pose)


def parse_pose(pose: Pose) -> str:
    """Converts Pose to printable string."""

    position_str = f"Position({pose.position.x},{pose.position.y},{pose.position.z})"
    quaternion_str = f"Quaternion({pose.orientation.x},{pose.orientation.y},{pose.orientation.z},{pose.orientation.w})"

    return f"[{position_str}, {quaternion_str}]"
