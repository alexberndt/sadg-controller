from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile

from sadg_controller.sadg.status import Status
from sadg_controller.sadg.vertex import Vertex


class Comms:
    def __init__(self, node: Node, agent_ns: str, vertex_initial: Vertex):
        """Agent communications.

        Communication module for a single agent.

        Args:
            node: ROS node to assign communication channels to.
            agent_ns: Namespace of the agent over the ROS network.
                Also used as a unique identifier for this agent.
            vertex_initial: Initial SADG vertex which the agent
                will execute.
        """
        self.ns = agent_ns
        self.current_vertex = vertex_initial
        self.logger = node.get_logger()

        self.sub_link = f"/{self.ns}/current"
        self.subscriber = node.create_subscription(
            Pose, self.sub_link, self.callback, 10
        )

        latching_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_ALL,
        )

        self.pub_link_goal = f"/{self.ns}/goal"
        self.publisher_goal = node.create_publisher(
            Pose, self.pub_link_goal, qos_profile=latching_qos
        )

        self.pub_link_init = f"/{self.ns}/initial"
        self.publisher_init = node.create_publisher(
            Pose, self.pub_link_init, qos_profile=latching_qos
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
        self.logger.debug(f"{self.sub_link}: Callback: {parse_pose(pose_current)}")

        goal_x = self.current_vertex.get_goal_loc().x
        goal_y = self.current_vertex.get_goal_loc().y

        # Check if location is equal to goal position of current vertex
        if pose_current.position.x == goal_x and pose_current.position.y == goal_y:

            # Update status of vertex
            self.current_vertex.set_status(Status.COMPLETED)

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
        self.logger.debug(
            f"{self.pub_link_goal}: Publishing goal pose: {parse_pose(goal_pose)}"
        )
        self.publisher_goal.publish(goal_pose)

    def publish_pose_initial(self, pose_initial: Pose) -> None:
        """Publish initial pose.

        Used during initialization to instantiate the agent in its
        initial (starting) pose for the simulation.

        Args:
            pose_initial: Initial pose of the Agent.
        """
        self.logger.debug(
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
    return Pose(
        position=Point(x=location.x, y=location.y, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )
