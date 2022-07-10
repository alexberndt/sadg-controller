from typing import Dict, List, Union

from sadg_controller.sadg.dependency import Dependency
from sadg_controller.sadg.dependency_switch import DependencySwitch
from sadg_controller.sadg.vertex import Vertex


class SADG:
    def __init__(
        self,
        vertices: Dict[str, List[Vertex]],
        dependencies: Dict[str, List[Union[Dependency, DependencySwitch]]],
    ) -> None:
        self.vertices = vertices
        self.dependencies = dependencies
