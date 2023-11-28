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
