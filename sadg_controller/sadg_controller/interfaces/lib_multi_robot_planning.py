# sadg-controller - MAPF execution with Switchable Action Dependency Graphs
# Copyright (c) 2023 Alex Berndt
# Copyright (c) 2023 Niels van Duijkeren, Robert Bosch GmbH
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

import csv
import random
from typing import Dict, List

import yaml

yaml.Dumper.ignore_aliases = lambda *args: True


def convert_roadmap_and_agv(roadmap_file: str, agv_count: int, output_file: str) -> str:
    """Convert the roadmap and AGV definition to single file for ECBS algorithm.

    Args:
        roadmap_file: CSV file containing roadmap.
        agv_file: Yaml file containing AGV start, goal locations.
    """

    roadmap_array = read_roadmap_csv(roadmap_file)
    # agv_locations = read_yaml(agv_file)

    return create_single_file(roadmap_array, agv_count, output_file)


def create_single_file(
    roadmap_array: List[List[str]], agv_count: int, output_file: str
) -> str:
    """Create file for ECBS input."""

    # agents:
    # -   goal: [29, 2]
    #     name: agent0
    #     start: [3, 1]
    # -   goal: [5, 24]
    #     name: agent1
    #     start: [3, 31]
    # -   goal: [14, 6]
    #     name: agent2
    #     start: [25, 29]
    # map:
    #     dimensions: [32, 32]
    #     obstacles:
    # - [29, 1]
    # - [10, 9]
    # - [6, 28]
    # - [25, 25]
    # - [22, 31]

    final_input = {}

    # Add map obstacles from roadmap array
    map_data = get_map_data(roadmap_array)
    final_input["map"] = map_data

    # Determine valid AGV start positions from roadmap
    valid_agv_starts = get_valid_agv_starts(roadmap_array)
    final_input["agents"] = randomize_agv_start_goals(valid_agv_starts, agv_count)

    write_yaml(output_file, final_input)

    return output_file


def randomize_agv_start_goals(
    valid_agv_starts: List[List[int]], agv_count: int
) -> Dict[str, any]:

    random_starts = random.sample(valid_agv_starts, agv_count)
    random_goals = random.sample(valid_agv_starts, agv_count)

    agv_data = []

    for idx, (start, goal) in enumerate(zip(random_starts, random_goals)):

        agv_data.append(
            {
                "name": f"agent{idx}",
                "start": start,
                "goal": goal,
            }
        )

    return agv_data


def get_map_data(roadmap_array: List[List[str]]) -> Dict[str, List[List[int]]]:
    """Convert roadmap array into yaml input for ECBS algorithm"""
    dimensions = [len(roadmap_array), len(roadmap_array[0])]
    obstacles = []

    for row_idx, row in enumerate(roadmap_array):
        for cell_idx, cell in enumerate(row):
            if int(cell) == 9:
                obstacles.append([int(cell_idx), int(row_idx)])

    map_data = {
        "dimensions": dimensions,
        "obstacles": obstacles,
    }

    return map_data


def get_valid_agv_starts(roadmap_array: List[List[str]]) -> List[List[int]]:

    valid_agv_starts = []

    for row_idx, row in enumerate(roadmap_array):
        for cell_idx, cell in enumerate(row):
            if int(cell) == 1:
                valid_agv_starts.append([int(cell_idx), int(row_idx)])

    return valid_agv_starts


def read_roadmap_csv(file: str) -> List[List[str]]:
    """Read roadmap csv file."""

    with open(file, "r") as stream:
        data_reader = csv.reader(stream)
        cols = []
        for row in data_reader:
            cols.append(row)
        return cols


def read_yaml(file: str) -> Dict:
    with open(file, "r") as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)


def write_yaml(file: str, payload: Dict) -> None:
    with open(file, "w") as stream:
        yaml.dump(payload, stream, default_flow_style=False)


if __name__ == "__main__":

    roadmap_file = "data/roadmaps/test/roadmap.csv"
    # agv_file = "data/agvs/agv_locations.yaml"
    agv_count = 4
    output_file = "data/ecbs/test.yaml"

    convert_roadmap_and_agv(roadmap_file, agv_count, output_file)
