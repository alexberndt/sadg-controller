from logging import Logger
from typing import List

logger = Logger(__name__)

from sadg_controller.core.agv import AGV
from sadg_controller.core.roadmap import Roadmap

class MAPFProblem:
    def __init__(self, roadmap: Roadmap, agvs: List[AGV]) -> None:
        self.roadmap = Roadmap
        self.agvs = agvs
        