from typing import Dict, List, Union

from sadg_controller.sadg.dependency import Dependency
from sadg_controller.sadg.dependency_switch import DependencySwitch
from sadg_controller.sadg.vertex import Vertex
from sadg_controller.se_adg.se_adg import SE_ADG


class SADG:
    def __init__(
        self,
        vertices: Dict[str, List[Vertex]],
        dependencies: Dict[str, List[Union[Dependency, DependencySwitch]]],
    ) -> None:
        self.vertices = vertices
        self.dependencies = dependencies

    def get_SE_ADG(self, b_vec: List[bool]) -> SE_ADG:
        """Get the SE-ADG for a given boolean vector.

        Args:
            b_vec: Vector of booleans.
        """
