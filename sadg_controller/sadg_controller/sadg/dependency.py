import sadg_controller.sadg.vertex as vertex


class Dependency:
    def __init__(
        self, tail: vertex.Vertex, head: vertex.Vertex, active: bool = True
    ) -> None:
        self.tail = tail
        self.head = head
        self.active = active

    def is_active(self) -> bool:
        return self.active

    def toggle(self) -> bool:
        """
        Toggle dependency active/inactive
        """
        self.active = not self.active
        return self.active

    def get_tail(self) -> vertex.Vertex:
        return self.tail

    def get_head(self) -> vertex.Vertex:
        return self.head

    def __repr__(self) -> str:
        return f"Dependency(tail={self.tail}, head={self.head})"
