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

import csv
import random
from logging import getLogger
from typing import Dict, List

import yaml

from sadg_controller.mapf.roadmap_location import RoadmapLocation

logger = getLogger(__name__)
random.seed(1)


class Roadmap:
    def __init__(self, roadmap_path: str) -> None:

        roadmap_file = f"/{roadmap_path}/roadmap.csv"
        dimensions_file = f"/{roadmap_path}/dimensions.yaml"

        self.array = self._read_roadmap_csv(roadmap_file)
        self.dimensions = self._read_dimensions(dimensions_file)
        self.map_data = self._get_map_data()

    def random_locations(self, agv_count: int) -> List[RoadmapLocation]:
        """Return random locations on roadmap.

        Samples `agv_count` random locations from the roadmap."""

        valid_start_locations = self._get_valid_agv_starts()

        if len(valid_start_locations) < agv_count:
            raise Exception(
                "AGV count is higher than number of valid start/goal locations"
            )

        return random.sample(valid_start_locations, agv_count)

    def get_map_data(self) -> Dict[str, List[List[int]]]:
        return self.map_data

    def get_dimensions(self) -> Dict:
        return self.dimensions

    def _get_valid_agv_starts(self) -> List[RoadmapLocation]:

        valid_locations = []
        for row_idx, row in enumerate(self.array):
            for col_idx, cell in enumerate(row):
                if int(cell) == 1:
                    loc = RoadmapLocation(row=int(row_idx), col=int(col_idx))
                    valid_locations.append(loc)

        return valid_locations

    def _get_map_data(self) -> Dict[str, List[List[int]]]:
        """Convert roadmap array into yaml input for ECBS algorithm"""

        logger.debug("Constructing map data ...")

        # Parse dimensions
        row_count = len(self.array)
        col_count = len(self.array[0])
        dimensions = [row_count, col_count]

        # Parse obstacles
        obstacles = []
        for row_idx, row in enumerate(self.array):
            for col_idx, cell in enumerate(row):
                if int(cell) == 9:
                    obstacles.append([int(row_idx), int(col_idx)])

        map_data = {
            "dimensions": dimensions,
            "obstacles": obstacles,
        }

        return map_data

    def _read_roadmap_csv(self, csv_file: str) -> List[List[str]]:
        """Read roadmap csv file."""

        logger.debug(f"Reading roadmap csv file: {csv_file} ...")
        with open(csv_file, "r") as stream:
            data_reader = csv.reader(stream)
            array = []
            for row in data_reader:
                array.append(row)
            return array

    def _read_dimensions(self, yaml_file: str) -> Dict:
        """Read roadmap dimensions file."""

        logger.debug(f"Reading dimensions yaml file: {yaml_file} ...")
        with open(yaml_file, "r") as stream:
            try:
                return yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
