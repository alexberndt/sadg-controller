import os
import subprocess
from logging import Logger
from typing import Dict, List

import yaml

from sadg_controller.core.location import Location
from sadg_controller.core.plan import Plan
from sadg_controller.core.roadmap import Roadmap
from sadg_controller.utils.constants import RANDOM_SEQUENCE_GENERATOR

yaml.Dumper.ignore_aliases = lambda *args: True
logger = Logger(__name__)


class MAPFProblem:
    def __init__(
        self, roadmap: Roadmap, starts: List[Location], goals: List[Location]
    ) -> None:
        self.roadmap = roadmap
        self.starts = starts
        self.goals = goals
        self.logger = logger
        self.tmp_dir = f"tmp/{next(RANDOM_SEQUENCE_GENERATOR)}"
        os.makedirs(self.tmp_dir)

    def solve(self, suboptimality_factor: float = 1.5) -> Plan:
        """Solve the MAPF problem using ECBS algorithm."""

        self.logger.info("Solving MAPF problem ...")

        # create dict from roadmap
        map_data = self.roadmap.get_map_data()

        # create dict from AGV start/goal locations
        agv_data = self._get_agv_data()

        # write to yaml file
        input_file = f"{self.tmp_dir}/input.yaml"
        ecbs_input = {
            "map": map_data,
            "agents": agv_data,
        }
        write_yaml(input_file, ecbs_input)

        # solve MAPF using the generated yaml file
        solution_file = f"{self.tmp_dir}/solution.yaml"
        cmd = [
            "third_party/libMultiRobotPlanning/build/ecbs",
            "-i",
            input_file,
            "-o",
            solution_file,
            "-w",
            str(suboptimality_factor),
        ]

        result = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )

        del result

        # read solution
        solution = read_yaml(solution_file)

        return Plan(solution)

    def _get_agv_data(self) -> Dict:

        agv_data = []
        for idx, (start, goal) in enumerate(zip(self.starts, self.goals)):

            agv_data.append(
                {
                    "name": f"agent{idx}",
                    "start": start.get_row_col(),
                    "goal": goal.get_row_col(),
                }
            )

        return agv_data


def write_yaml(file: str, payload: Dict) -> None:
    with open(file, "w") as stream:
        yaml.dump(payload, stream, default_flow_style=False)


def read_yaml(file: str) -> Dict:
    with open(file, "r") as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
