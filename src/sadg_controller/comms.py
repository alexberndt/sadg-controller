import rospy
from geometry_msgs.msg import Point, Pose, Quaternion

from sadg_controller.sadg.status import Status
from sadg_controller.sadg.vertex import Vertex


class Comms:
    def __init__(self, agent_ns: str, vertex_initial: Vertex):
        """Agent communications.

        Communication module for a single agent.

        Args:
            agent_ns: Namespace of the agent over the ROS network.
                Also used as a unique identifier for this agent.
            vertex_initial: Initial SADG vertex which the agent
                will execute.
        """
        self.ns = agent_ns
        self.current_vertex = vertex_initial

        self.sub_link = f"/{self.ns}/current"
        self.subscriber = rospy.Subscriber(self.sub_link, Pose, self.callback)

        self.pub_link = f"/{self.ns}/goal"
        self.publisher = rospy.Publisher(self.pub_link, Pose, queue_size=10, latch=True)

        self.pub_link_init = f"/{self.ns}/initial"
        self.publisher_init = rospy.Publisher(
            self.pub_link_init, Pose, queue_size=10, latch=True
        )

        pose_initial = get_start_pose(vertex_initial)
        self.publish_pose_initial(pose_initial)

    def get_agent_id(self) -> str:
        return self.ns

    def get_curr_vertex(self) -> Vertex:
        return self.current_vertex

    def callback(self, pose_current: Pose) -> None:
        """Subscriber callback with current pose.

        Updates the current pose of the agent from the
        simulation. If the current pose coincides with the
        current goal pose, the current SADG vertex status is
        updated accordingly.

        Args:
            pose_current: Current agent pose.
        """
        rospy.logdebug(f"{self.sub_link}: Callback: {parse_pose(pose_current)}")

        goal_x = self.current_vertex.get_goal_loc().x
        goal_y = self.current_vertex.get_goal_loc().y

        # Check if location is equal to goal position of current vertex
        if pose_current.position.x == goal_x and pose_current.position.y == goal_y:

            # Update status of vertex
            self.current_vertex.status = Status.COMPLETED

            # Update next vertex if next vertex exists
            if self.current_vertex.has_next():
                self.current_vertex = self.current_vertex.get_next()

    def publish_pose_goal(self, goal_pose: Pose) -> None:
        """Publish goal pose.

        Used to publish a new goal pose for the agent as
        dictated by the SADG controller.

        Args:
            goal_pose: Goal pose which the agent should move
                towards.
        """
        rospy.logwarn(f"{self.pub_link}: Publishing goal pose: {parse_pose(goal_pose)}")
        self.publisher.publish(goal_pose)

    def publish_pose_initial(self, pose_initial: Pose) -> None:
        """Publish initial pose.

        Used during initialization to instantiate the agent in its
        initial (starting) pose for the simulation.

        Args:
            pose_initial: Initial pose of the Agent.
        """
        rospy.logwarn(
            f"{self.pub_link_init}: Publishing initial pose: {parse_pose(pose_initial)}"
        )
        self.publisher_init.publish(pose_initial)


def parse_pose(pose: Pose) -> str:
    """Converts Pose to printable string."""

    position_str = f"Position({pose.position.x},{pose.position.y},{pose.position.z})"
    quaternion_str = f"Quaternion({pose.orientation.x},{pose.orientation.y},{pose.orientation.z},{pose.orientation.w})"

    return f"[{position_str}, {quaternion_str}]"


def get_start_pose(vertex: Vertex) -> Pose:
    """Converts a vertex start location to a pose.

    Args:
        vertex: Vertex whose start location is converted
            to a pose.
    """
    location = vertex.get_start_loc()
    return Pose(Point(location.x, location.y, 0), Quaternion(0, 0, 0, 1))
