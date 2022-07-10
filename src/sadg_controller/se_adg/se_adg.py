from typing import Dict, List

from sadg_controller.sadg.dependency import Dependency
from sadg_controller.sadg.vertex import Vertex


class SE_ADG:
    def __init__(
        self,
        vertices: Dict[str, List[Vertex]],
        dependencies: Dict[str, List[Dependency]],
    ) -> None:
        self.vertices = vertices
        self.dependencies = dependencies
