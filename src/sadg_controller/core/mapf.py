from logging import Logger
from typing import List

from sadg_controller.core.agv import AGV
from sadg_controller.core.plan import Plan
from sadg_controller.core.roadmap import Roadmap
from sadg_controller.core.locations import Locations

logger = Logger(__name__)


class MAPFProblem:
    def __init__(self, roadmap: Roadmap, starts: Locations, goals: Locations) -> None:
        self.roadmap = roadmap
        self.starts = starts
        self.goals = goals
        self.logger = logger


    def solve(self) -> Plan:

        self.logger.info("Solving MAPF problem ...")

        return Plan()