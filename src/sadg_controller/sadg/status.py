from enum import Enum


class Status(Enum):
    STAGED = 0
    IN_PROGRESS = 1
    COMPLETED = 2

    def color(self):
        color = {
            0: "#dddddd",  # grey
            1: "#f5aa42",  # orange
            2: "#42f557",  # green
        }[self.value]
        return color
