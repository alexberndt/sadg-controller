from logging import getLogger

logger = getLogger(__name__)


class Agent:
    def __init__(self, uuid: str) -> None:
        self.uuid = uuid
