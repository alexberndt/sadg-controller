from logging import Logger

logger = Logger(__name__)


class Roadmap:
    def __init__(self, map_file: str) -> None:
        self.logger = logger
        self.map_file = map_file
