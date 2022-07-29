from sadg_controller.sadg.dependency import Dependency


class DependencySwitch:
    def __init__(self, fwd: Dependency, rev: Dependency, b: bool = False) -> None:
        """Dependency switch between fwd-rev dependency pair.

        Args:
            fwd: Forward dependency.
            rev: Reverse counterpart to `fwd`.
            b: Switch logic. Defaults to False, implying forward dependency
                is active.
        """
        self.fwd = fwd
        self.rev = rev
        self.b = b

    def get_active(self) -> Dependency:
        return self.fwd if not self.b else self.rev

    def switch(self) -> None:
        """Switch fwd/rev dependencies to (in)active."""
        self.fwd.switch()
        self.rev.switch()

    def __repr__(self) -> str:
        return f"DependencySwitch(fwd={self.fwd}, rev={self.rev})"
