from sadg_controller.sadg.vertex import Vertex


class Dependency:
    def __init__(self, tail: Vertex, head: Vertex) -> None:
        self.tail = tail
        self.head = head

    def __repr__(self) -> str:
        return f"Dependency(tail={self.tail}, head={self.head})"
