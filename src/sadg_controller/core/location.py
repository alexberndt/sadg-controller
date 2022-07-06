from typing import List


class Location:
    def __init__(self, row: int, col: int) -> None:
        self.row = row
        self.col = col

    def get_row_col(self) -> List[int]:
        return [self.row, self.col]
