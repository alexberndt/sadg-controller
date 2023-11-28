# sadg-controller
# Copyright (c) 2023 Alexander Berndt, Robert Bosch GmbH
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as published
# by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Affero General Public License for more details.
#
# You should have received a copy of the GNU Affero General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

from enum import Enum
from typing import List

from sadg_controller.sadg.dependency_switch import DependencySwitch
from sadg_controller.sadg.status import Status
from sadg_controller.sadg.vertex import Vertex


class DependencyGroupType(Enum):
    SINGLE = 0
    SAME = 1
    OPPOSITE = 2


class DependencyGroup:
    def __init__(
        self,
        dependency_switch: DependencySwitch,
        uid: str,
        contains_unswitchable_dependency: bool = False,
    ) -> None:
        self.uid: str = uid
        self.dependencies: List[DependencySwitch] = [dependency_switch]
        self.type = DependencyGroupType.SINGLE
        self.contains_unswitchable_dependency = contains_unswitchable_dependency

        self.last_tail: Vertex = dependency_switch.get_active().get_tail()
        self.last_tail_vertex_idx: int = self.last_tail.get_vertex_idx()
        self.last_tail_agent_id: int = self.last_tail.get_agent_id()

        self.last_head: Vertex = dependency_switch.get_active().get_head()
        self.last_head_vertex_idx: int = self.last_head.get_vertex_idx()
        self.last_head_agent_id: int = self.last_head.get_agent_id()

        # Create list of "first" vertex heads for group
        self.first_head_active: Vertex = dependency_switch.get_active().get_head()

        if dependency_switch.get_inactive() is None:
            self.contains_unswitchable_dependency = True
        else:
            self.first_head_inactive: Vertex = (
                dependency_switch.get_inactive().get_head()
            )

    def get_uid(self) -> str:
        """Return the unique identifier for this dependency group."""
        return self.uid

    def append_switch(self, new_dependency_switch: DependencySwitch) -> bool:
        """Append a dependency switch to the group.

        Used when creating the SADG, not while optimizing.

        Args:
            dependency_switch: Dependency switch to add to the group.

        Returns:
            True if switch is successfully added to group, otherwise False.
        """

        new_active_dependency = new_dependency_switch.get_active()
        tail = new_active_dependency.get_tail()
        head = new_active_dependency.get_head()

        # Check for same
        if (
            tail.get_agent_id() == self.last_tail_agent_id
            and head.get_agent_id() == self.last_head_agent_id
            and tail.get_vertex_idx() == self.last_tail_vertex_idx + 1
            and head.get_vertex_idx() == self.last_head_vertex_idx + 1
            and self.type in [DependencyGroupType.SINGLE, DependencyGroupType.SAME]
        ):

            self.dependencies.append(new_dependency_switch)
            self.type = DependencyGroupType.SAME

            # Update last tail and head vertex indexes
            self.last_tail_vertex_idx = tail.get_vertex_idx()
            self.last_head_vertex_idx = head.get_vertex_idx()

            return True

        # Check for opposite
        elif (
            tail.get_agent_id() == self.last_tail_agent_id
            and head.get_agent_id() == self.last_head_agent_id
            and tail.get_vertex_idx() == self.last_tail_vertex_idx + 1
            and head.get_vertex_idx() == self.last_head_vertex_idx - 1
            and self.type in [DependencyGroupType.SINGLE, DependencyGroupType.OPPOSITE]
        ):

            self.dependencies.append(new_dependency_switch)
            self.type = DependencyGroupType.OPPOSITE

            # Update last tail and head vertex indexes
            self.last_tail_vertex_idx = tail.get_vertex_idx()
            self.last_head_vertex_idx = head.get_vertex_idx()

            # Update "first" reverse head arrow
            if not self.contains_unswitchable_dependency:
                self.first_head_inactive = new_active_dependency.get_head()

            return True

        # Else cannot append dependency switch
        else:
            return False

    def within_horizon(self, horizon: float) -> bool:
        """
        Check whether the dependencies within the group
        fall within the provided horizon.

        This function is used to ensure only switching 
        of dependency groups within a receding horizon.

        Args:
            horizon: Time in seconds from current vertices 
                where switching should be considered.
        """

        def estimate_time_to_vertex(vertex: Vertex) -> float:
            """
            Determine the time between a STAGED vertex and
            the closest IN-PROGRESS or COMPLETED vertex 
            preceding it, thus calculating the expected 
            remaining time before that vertex is changed
            to IN-PROGRESS.

            This is used to determine how far ahead in time 
            a vertex will be "seen".
            """
            remaining_time = 0
            curr_vertex = vertex
            while curr_vertex.has_prev() and curr_vertex.get_status() == Status.STAGED:
                curr_vertex = curr_vertex.get_prev()
                remaining_time += curr_vertex.get_expected_completion_time()
            return remaining_time
        
        # recursively search through current active head vertex of DG
        time_to_vertex = estimate_time_to_vertex(self.first_head_active)

        if time_to_vertex > horizon:
            print("hello")

        return time_to_vertex < horizon        
        
    def get_dependencies(self) -> List[DependencySwitch]:
        return self.dependencies

    def is_switchable(self) -> bool:
        """
        Checks if the dependencies in the dependency group
        can be switched based on each dependency's vertex
        status.

        A Dependency Group is only switchable iff. both the
        forward and reverse dependencies' statuses are set
        to STAGED.

        To avoid keeping track of ALL the dependencies, we
        only track the "first" head dependency status from
        the forward and reverse dependencies.
        """

        if self.contains_unswitchable_dependency:
            return False
        else:

            return (
                self.first_head_active.get_status() == Status.STAGED
                and self.first_head_inactive.get_status() == Status.STAGED
            )

    def switch(self) -> None:

        # if not self.is_switchable():
        #     raise ValueError("Cannot switch unswitchable dependency ...")

        for dependency in self.dependencies:
            dependency.switch()

        # Create list of "first" vertex heads for group
        self.first_head_active: Vertex = self.dependencies[-1].get_active().get_head()

        if self.type == DependencyGroupType.OPPOSITE:
            self.first_head_inactive: Vertex = (
                self.dependencies[-1].get_inactive().get_head()
            )
        else:
            self.first_head_inactive: Vertex = (
                self.dependencies[0].get_inactive().get_head()
            )

    def __repr__(self) -> str:
        return f"v_{self.last_tail_agent_id}_{self.last_tail_vertex_idx}_to_v_{self.last_head_agent_id}_{self.last_head_vertex_idx}_{self.type}"
