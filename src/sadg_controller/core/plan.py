from logging import Logger
from typing import Dict

logger = Logger(__name__)


class Plan:
    def __init__(self, solution: Dict) -> None:
        self.solution = solution
        self.logger = logger
