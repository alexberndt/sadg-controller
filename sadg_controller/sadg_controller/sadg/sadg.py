from typing import Dict, List

from sadg_controller.sadg.dependency import Dependency
from sadg_controller.sadg.dependency_group import DependencyGroup
from sadg_controller.sadg.vertex import Vertex
from sadg_controller.se_adg.se_adg import SE_ADG

class SADG:
    def __init__(
        self,
        vertices: Dict[str, List[Vertex]],
        regular_deps: Dict[str, List[Dependency]],
        switch_groups: Dict[str, List[DependencyGroup]],
    ) -> None:
        self.vertices = vertices
        self.regular_deps = regular_deps
        self.switch_groups = switch_groups

    # def get_active_SE_ADG(self, b_vec: List[bool]) -> SE_ADG:
    #     """Get the SE-ADG for a given boolean vector.

    #     Args:
    #         b_vec: Vector of booleans.
    #     """

    #     active_dependencies = self.regular_deps

    #     for agent_id, dep_group in self.switch_groups:

    #         # TODO: use b_vec as input
    #         active_deps = dep_group.get_active()

    #         active_dependencies[agent_id].append(active_deps)

    #     return SE_ADG(self.vertices, active_dependencies)

    def get_agent_first_vertex(self, agent_id: str) -> Vertex:
        return self.vertices[agent_id][0]
