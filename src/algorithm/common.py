"""
Các định nghĩa chung và Enum được sử dụng trong module thuật toán.
"""

from enum import Enum

class VariableType(Enum):
    """Loại biến"""
    CONTINUOUS = "continuous"
    INTEGER = "integer"
    BINARY = "binary"

class ObjectiveType(Enum):
    """Loại mục tiêu"""
    MINIMIZE = "minimize"
    MAXIMIZE = "maximize"

class SolverStatus:
    """Trạng thái của bộ giải"""
    OPTIMAL = "optimal"
    FEASIBLE = "feasible"
    INFEASIBLE = "infeasible"
    UNBOUNDED = "unbounded"
    UNKNOWN = "unknown"
    ERROR = "error"
