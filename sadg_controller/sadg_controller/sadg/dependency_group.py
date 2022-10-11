from enum import Enum

from sadg_controller.sadg.dependency_switch import DependencySwitch


class DependencyGroupType(Enum):
    SINGLE = 0
    SAME = 1
    OPPOSITE = 2


class DependencyGroup:
    def __init__(self, dependency_switch: DependencySwitch) -> None:
        self.dependencies = [dependency_switch]
        self.type = DependencyGroupType.SINGLE

        self.last_tail_vertex_idx = dependency_switch.fwd.tail.get_vertex_idx()
        self.last_tail_agent_id = dependency_switch.fwd.tail.get_agent_id()

        self.last_head_vertex_idx = dependency_switch.fwd.head.get_vertex_idx()
        self.last_head_agent_id = dependency_switch.fwd.head.get_agent_id()

    def append_switch(self, dependency_switch: DependencySwitch) -> bool:
        """Append a dependency switch to the group.

        Args:
            dependency_switch: Dependency switch to add to the group.

        Returns:
            True if switch is successfully added to group, otherwise False.
        """

        tail_vertex_idx = dependency_switch.fwd.tail.get_vertex_idx()
        tail_agent_id = dependency_switch.fwd.tail.get_agent_id()
        head_vertex_idx = dependency_switch.fwd.head.get_vertex_idx()
        head_agent_id = dependency_switch.fwd.head.get_agent_id()

        # Check for same
        if (
            tail_agent_id == self.last_tail_agent_id
            and head_agent_id == self.last_head_agent_id
            and tail_vertex_idx == self.last_tail_vertex_idx + 1
            and head_vertex_idx == self.last_head_vertex_idx + 1
            and self.type in [DependencyGroupType.SINGLE, DependencyGroupType.SAME]
        ):

            print("same")
            self.dependencies.append(dependency_switch)
            self.type = DependencyGroupType.SAME
            self.last_tail_vertex_idx = tail_vertex_idx
            self.last_head_vertex_idx = head_vertex_idx

            return True

        # Check for opposite
        elif (
            tail_agent_id == self.last_tail_agent_id
            and head_agent_id == self.last_head_agent_id
            and tail_vertex_idx == self.last_tail_vertex_idx + 1
            and head_vertex_idx == self.last_head_vertex_idx - 1
            and self.type in [DependencyGroupType.SINGLE, DependencyGroupType.OPPOSITE]
        ):

            print("opposite")
            self.dependencies.append(dependency_switch)
            self.type = DependencyGroupType.OPPOSITE
            self.last_tail_vertex_idx = tail_vertex_idx
            self.last_head_vertex_idx = head_vertex_idx

            return True

        # Else cannot append dependency switch
        else:
            print("single")
            return False

    def switch(self) -> None:
        for dependency in self.dependencies:
            dependency.switch()
