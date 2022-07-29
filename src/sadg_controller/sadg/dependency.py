from sadg_controller.sadg.status import Status
from sadg_controller.sadg.vertex import Vertex


class Dependency:
    def __init__(self, tail: Vertex, head: Vertex, active: bool = True) -> None:
        self.tail = tail
        self.head = head
        self.active = active

    def is_active(self) -> bool:
        return self.active

    def switch(self) -> bool:
        self.active = not self.active
        return self.active

    def get_tail_status(self) -> Status:
        return self.tail.status

    def __repr__(self) -> str:
        return f"Dependency(tail={self.tail}, head={self.head})"
