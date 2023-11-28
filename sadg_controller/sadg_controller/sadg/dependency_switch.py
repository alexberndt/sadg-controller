# sadg-controller
# Copyright (c) 2023 Alexander Berndt, Robert Bosch GmbH
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as published
# by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Affero General Public License for more details.
#
# You should have received a copy of the GNU Affero General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

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
