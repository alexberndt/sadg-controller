from typing import List


class RoadmapLocation:
    def __init__(self, row: int, col: int) -> None:
        self.row = row
        self.col = col

    def get_row_col(self) -> List[int]:
        return [self.row, self.col]
