from logging import Logger
from typing import List

from sadg_controller.core.agv import AGV
from sadg_controller.core.roadmap import Roadmap

logger = Logger(__name__)


class MAPFProblem:
    def __init__(self, roadmap: Roadmap, agvs: List[AGV]) -> None:
        self.roadmap = Roadmap
        self.agvs = agvs
        self.logger = logger


    def solve(self):

        self.logger.info("Solving MAPF problem ...")