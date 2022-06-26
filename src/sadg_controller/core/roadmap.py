from logging import Logger

from sadg_controller.core.locations import Locations

logger = Logger(__name__)


class Roadmap:
    def __init__(self, map_file: str) -> None:
        self.logger = logger
        self.map_file = map_file

    def random_locations(self, agv_count: int) -> Locations:

        return Locations()
