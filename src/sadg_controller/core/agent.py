from logging import Logger

logger = Logger(__name__)


class Agent:
    def __init__(self, uuid: str) -> None:
        self.uuid = uuid
