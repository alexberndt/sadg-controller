from sadg_controller.sadg.dependency import Dependency


class DependencySwitch:
    def __init__(
        self,
        forward: Dependency,
        reverse: Dependency,
        b: bool = False,
        switchable: bool = True,
    ) -> None:
        """Dependency switch between fwd-rev dependency pair.

        Args:
            fwd: Forward dependency.
            rev: Reverse counterpart to `fwd`.
            b: Switch logic. Defaults to False, implying forward dependency
                is active.
        """
        self.forward = forward
        self.reverse = reverse
        self.b = b
        self.switchable = switchable

    def get_active(self) -> Dependency:
        return self.forward if not self.b else self.reverse

    def get_inactive(self) -> Dependency:
        return self.forward if self.b else self.reverse

    def switch(self) -> None:
        """
        Switch foward / reverse dependencies to (in)active.
        """
        if self.switchable:
            self.forward.toggle()
            self.reverse.toggle()
            self.b = not self.b
        else:
            raise RuntimeError("DependencySwitch not switchable ...")

    def __repr__(self) -> str:
        return f"DependencySwitch(fwd={self.forward}, rev={self.reverse})"
