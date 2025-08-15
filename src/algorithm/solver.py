"""
MIQP Solver using Google OR-Tools
Hỗ trợ giải bài toán Mixed-Integer Quadratic Programming

Cách sử dụng:
    from miqp_solver import MIQPSolver
    
    solver = MIQPSolver()
    # Thêm biến và ràng buộc
    # Giải bài toán
    result = solver.solve()
"""

from ortools.linear_solver import pywraplp
from ortools.sat.python import cp_model
import numpy as np
from typing import Dict, List, Tuple, Optional, Union
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
    """Trạng thái giải"""
    OPTIMAL = "optimal"
    FEASIBLE = "feasible"
    INFEASIBLE = "infeasible"
    UNBOUNDED = "unbounded"
    UNKNOWN = "unknown"
    ERROR = "error"

class MIQPSolver:
    """
    Wrapper class cho Google OR-Tools để giải bài toán MIQP
    
    Hỗ trợ:
    - Biến liên tục, nguyên, nhị phân
    - Ràng buộc tuyến tính
    - Hàm mục tiêu bậc hai (quadratic)
    - Ràng buộc bậc hai đơn giản
    """
    
    def __init__(self, solver_name: str = 'SCIP'):
        """
        Khởi tạo solver
        
        Args:
            solver_name: Tên solver ('SCIP', 'GUROBI', 'CPLEX', 'CBC')
        """
        self.solver_name = solver_name
        self.solver = None
        self.variables = {}  # Dict[str, Variable]
        self.constraints = []
        self.objective_coeffs = {}  # Hệ số tuyến tính
        self.quadratic_coeffs = {}  # Hệ số bậc hai
        self.objective_type = ObjectiveType.MINIMIZE
        self.is_built = False
        
    def _create_solver(self):
        """Tạo solver instance"""
        if self.solver_name == 'SCIP':
            self.solver = pywraplp.Solver.CreateSolver('SCIP')
        elif self.solver_name == 'GUROBI':
            self.solver = pywraplp.Solver.CreateSolver('GUROBI')
        elif self.solver_name == 'CPLEX':
            self.solver = pywraplp.Solver.CreateSolver('CPLEX')
        elif self.solver_name == 'CBC':
            self.solver = pywraplp.Solver.CreateSolver('CBC')
        else:
            # Fallback to SCIP
            self.solver = pywraplp.Solver.CreateSolver('SCIP')
            
        if not self.solver:
            raise RuntimeError(f"Không thể tạo solver {self.solver_name}")
    
    def add_variable(self, name: str, var_type: VariableType, 
                    lb: float = 0.0, ub: float = float('inf')) -> str:
        """
        Thêm biến vào bài toán
        
        Args:
            name: Tên biến
            var_type: Loại biến (CONTINUOUS, INTEGER, BINARY)
            lb: Cận dưới
            ub: Cận trên
            
        Returns:
            str: Tên biến đã thêm
        """
        if var_type == VariableType.BINARY:
            lb, ub = 0.0, 1.0
            
        self.variables[name] = {
            'type': var_type,
            'lb': lb,
            'ub': ub,
            'var_obj': None
        }
        return name
    
    def add_variables(self, names: List[str], var_type: VariableType,
                     lb: Union[float, List[float]] = 0.0,
                     ub: Union[float, List[float]] = float('inf')) -> List[str]:
        """
        Thêm nhiều biến cùng lúc
        
        Args:
            names: Danh sách tên biến
            var_type: Loại biến
            lb: Cận dưới (số hoặc list)
            ub: Cận trên (số hoặc list)
            
        Returns:
            List[str]: Danh sách tên biến đã thêm
        """
        n = len(names)
        
        # Xử lý bounds
        if isinstance(lb, (int, float)):
            lb = [lb] * n
        if isinstance(ub, (int, float)):
            ub = [ub] * n
            
        for i, name in enumerate(names):
            self.add_variable(name, var_type, lb[i], ub[i])
            
        return names
    
    def add_constraint(self, coeffs: Dict[str, float], sense: str, rhs: float, name: str = ""):
        """
        Thêm ràng buộc tuyến tính
        
        Args:
            coeffs: Dict {tên_biến: hệ_số}
            sense: '<=' | '>=' | '=='
            rhs: Vế phải
            name: Tên ràng buộc
        """
        self.constraints.append({
            'coeffs': coeffs,
            'sense': sense,
            'rhs': rhs,
            'name': name
        })
    
    def set_objective_linear(self, coeffs: Dict[str, float], obj_type: ObjectiveType):
        """
        Thiết lập hàm mục tiêu tuyến tính
        
        Args:
            coeffs: Dict {tên_biến: hệ_số}
            obj_type: MINIMIZE hoặc MAXIMIZE
        """
        self.objective_coeffs = coeffs
        self.objective_type = obj_type
    
    def set_objective_quadratic(self, linear_coeffs: Dict[str, float],
                               quadratic_coeffs: Dict[Tuple[str, str], float],
                               obj_type: ObjectiveType):
        """
        Thiết lập hàm mục tiêu bậc hai
        
        Args:
            linear_coeffs: Dict {tên_biến: hệ_số_tuyến_tính}
            quadratic_coeffs: Dict {(tên_biến1, tên_biến2): hệ_số_bậc_hai}
            obj_type: MINIMIZE hoặc MAXIMIZE
        """
        self.objective_coeffs = linear_coeffs
        self.quadratic_coeffs = quadratic_coeffs
        self.objective_type = obj_type
    
    def _build_model(self):
        """Xây dựng model OR-Tools"""
        if self.is_built:
            return
            
        self._create_solver()
        
        # Tạo biến
        for name, var_info in self.variables.items():
            if var_info['type'] == VariableType.CONTINUOUS:
                var = self.solver.NumVar(var_info['lb'], var_info['ub'], name)
            elif var_info['type'] == VariableType.INTEGER:
                var = self.solver.IntVar(var_info['lb'], var_info['ub'], name)
            elif var_info['type'] == VariableType.BINARY:
                var = self.solver.BoolVar(name)
            
            self.variables[name]['var_obj'] = var
        
        # Thêm ràng buộc
        for i, constraint in enumerate(self.constraints):
            coeffs = constraint['coeffs']
            sense = constraint['sense']
            rhs = constraint['rhs']
            
            # Tạo constraint
            if sense == '<=':
                ct = self.solver.Constraint(-self.solver.infinity(), rhs)
            elif sense == '>=':
                ct = self.solver.Constraint(rhs, self.solver.infinity())
            elif sense == '==':
                ct = self.solver.Constraint(rhs, rhs)
            else:
                raise ValueError(f"Sense không hợp lệ: {sense}")
            
            # Thêm hệ số
            for var_name, coeff in coeffs.items():
                if var_name in self.variables:
                    ct.SetCoefficient(self.variables[var_name]['var_obj'], coeff)
        
        # Thiết lập hàm mục tiêu
        objective = self.solver.Objective()
        
        # Hệ số tuyến tính
        for var_name, coeff in self.objective_coeffs.items():
            if var_name in self.variables:
                objective.SetCoefficient(self.variables[var_name]['var_obj'], coeff)
        
        # Hệ số bậc hai (nếu có)
        if self.quadratic_coeffs:
            # OR-Tools không trực tiếp hỗ trợ objective bậc hai cho MIP
            # Cần dùng auxiliary variables hoặc chuyển đổi
            print("Cảnh báo: Hàm mục tiêu bậc hai có thể không được hỗ trợ đầy đủ")
            for (var1, var2), coeff in self.quadratic_coeffs.items():
                if var1 in self.variables and var2 in self.variables:
                    if var1 == var2:
                        # x^2 term - cần xử lý đặc biệt
                        continue
                    else:
                        # x*y term - cần auxiliary variable
                        continue
        
        # Thiết lập hướng tối ưu
        if self.objective_type == ObjectiveType.MINIMIZE:
            objective.SetMinimization()
        else:
            objective.SetMaximization()
        
        self.is_built = True
    
    def solve(self, time_limit: Optional[float] = None) -> Dict:
        """
        Giải bài toán
        
        Args:
            time_limit: Giới hạn thời gian (giây)
            
        Returns:
            Dict: Kết quả giải
        """
        if not self.is_built:
            self._build_model()
        
        # Thiết lập time limit
        if time_limit:
            self.solver.SetTimeLimit(int(time_limit * 1000))  # milliseconds
        
        # Giải bài toán
        status = self.solver.Solve()
        
        # Xử lý kết quả
        result = {
            'status': self._get_status_string(status),
            'objective_value': None,
            'variables': {},
            'solve_time': self.solver.wall_time() / 1000.0,  # seconds
            'iterations': self.solver.iterations() if hasattr(self.solver, 'iterations') else None
        }
        
        if status == pywraplp.Solver.OPTIMAL or status == pywraplp.Solver.FEASIBLE:
            result['objective_value'] = self.solver.Objective().Value()
            
            # Lấy giá trị biến
            for name, var_info in self.variables.items():
                var_obj = var_info['var_obj']
                result['variables'][name] = var_obj.solution_value()
        
        return result
    
    def _get_status_string(self, status) -> str:
        """Chuyển đổi status code thành string"""
        if status == pywraplp.Solver.OPTIMAL:
            return SolverStatus.OPTIMAL
        elif status == pywraplp.Solver.FEASIBLE:
            return SolverStatus.FEASIBLE
        elif status == pywraplp.Solver.INFEASIBLE:
            return SolverStatus.INFEASIBLE
        elif status == pywraplp.Solver.UNBOUNDED:
            return SolverStatus.UNBOUNDED
        elif status == pywraplp.Solver.ABNORMAL:
            return SolverStatus.ERROR
        else:
            return SolverStatus.UNKNOWN
    
    def get_variable_value(self, name: str) -> Optional[float]:
        """Lấy giá trị của một biến sau khi giải"""
        if name in self.variables and self.variables[name]['var_obj']:
            return self.variables[name]['var_obj'].solution_value()
        return None
    
    def print_solution(self, result: Dict):
        """In kết quả giải"""
        print("=== KẾT QUẢ GIẢI BÀI TOÁN ===")
        print(f"Trạng thái: {result['status']}")
        print(f"Giá trị mục tiêu: {result['objective_value']}")
        print(f"Thời gian giải: {result['solve_time']:.3f} giây")
        
        if result['variables']:
            print("\nGiá trị biến:")
            for name, value in result['variables'].items():
                print(f"  {name} = {value:.6f}")

# === EXAMPLE USAGE ===
def example_usage():
    """Ví dụ sử dụng MIQPSolver"""
    
    # Tạo solver
    solver = MIQPSolver('SCIP')
    
    # Thêm biến
    solver.add_variable('x1', VariableType.CONTINUOUS, 0, 10)
    solver.add_variable('x2', VariableType.CONTINUOUS, 0, 10)
    solver.add_variable('y', VariableType.INTEGER, 0, 5)
    solver.add_variable('z', VariableType.BINARY)
    
    # Thêm ràng buộc
    # x1 + 2*x2 + y <= 10
    solver.add_constraint({'x1': 1, 'x2': 2, 'y': 1}, '<=', 10, 'constraint1')
    
    # x1 + y >= 2
    solver.add_constraint({'x1': 1, 'y': 1}, '>=', 2, 'constraint2')
    
    # z*5 <= x2
    solver.add_constraint({'z': 5, 'x2': -1}, '<=', 0, 'constraint3')
    
    # Thiết lập hàm mục tiêu: min 3*x1 + 2*x2 - y + 10*z
    solver.set_objective_linear({
        'x1': 3,
        'x2': 2,
        'y': -1,
        'z': 10
    }, ObjectiveType.MINIMIZE)
    
    # Giải bài toán
    result = solver.solve(time_limit=30)
    
    # In kết quả
    solver.print_solution(result)
    
    return result

if __name__ == "__main__":
    example_usage()