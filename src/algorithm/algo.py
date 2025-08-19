"""
Hệ thống Điều khiển Chu vi Phản hồi (Perimeter Control) hoàn chỉnh
Sử dụng Google OR-Tools để giải bài toán MIQP

Tác giả: Sơn Đình and Đức Ngô

MODIFIED: Tích hợp bộ giải PySCIPOpt cho bài toán MIQP phi tuyến thực sự
trong khi vẫn giữ lại MIQPSolver cho các bài toán tuyến tính.
"""

import time
from ortools.linear_solver import pywraplp
from ortools.sat.python import cp_model
import numpy as np
from typing import Dict, List, Tuple, Optional, Union
from enum import Enum
import sys
import os

# Import thư viện mới để giải MIQP phi tuyến
from pyscipopt import Model, quicksum
from pyscipopt.recipes.nonlinear import set_nonlinear_objective

# Add project root to system path
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sys.path.append(project_root)
print(project_root)

# Import config manager
from src.data.intersection_config_manager import IntersectionConfigManager

# === CONSTANTS ===
KP_H = 20.0        # Proportional gain (1/hour)
KI_H = 5.0         # Integral gain (1/hour)
N_HAT = 80.0      # Target accumulation (vehicles)
CONTROL_INTERVAL_S = 90  # Control interval (seconds)
KP_MIN = 0.0
KP_MAX = 0.0
KI_MIN = 100.0
KI_MAX = 20.0

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

class PerimeterController:
    """
    Lớp điều khiển chu vi phản hồi hoàn chỉnh
    Thực hiện thuật toán điều khiển dựa trên bộ điều khiển PI
    """
    
    def __init__(self, kp: float = KP_H, ki: float = KI_H, n_hat: float = N_HAT, 
                 config_file: str = "intersection_config.json", shared_dict: Optional[Dict] = None):
        """Khởi tạo bộ điều khiển với các tham số cần thiết"""
        # Chuyển đổi từ đơn vị giờ sang giây
        control_interval_h = CONTROL_INTERVAL_S / 3600.0
        self.kp = kp * control_interval_h
        self.ki = ki * control_interval_h
        self.n_hat = n_hat
        
        self.shared_dict = shared_dict
        self.is_active = False
        if self.shared_dict is not None:
            self.shared_dict['is_active'] = self.is_active
            self.shared_dict['green_times'] = {}

        self.activation_threshold = 0.85 * self.n_hat  
        self.deactivation_threshold = 0.70 * self.n_hat 

        # Load cấu hình intersection từ JSON
        self.config_manager = IntersectionConfigManager(config_file)
        self.intersection_ids = self.config_manager.get_intersection_ids()
        self.global_params = self.config_manager.get_global_params()
        
        # Lưu trữ giá trị trước đó cho mỗi intersection
        self.previous_green_times = {}
        for intersection_id in self.intersection_ids:
            intersection_data = self.config_manager.get_intersection_data(intersection_id)
            if intersection_data:
                cycle_length = intersection_data.get('cycle_length', 90)
                # Giả sử chia đều cho pha chính và phụ
                self.previous_green_times[intersection_id] = {
                    'main': cycle_length // 2,
                    'secondary': cycle_length // 2
                }
        
        if self.shared_dict is not None:
            self.shared_dict['green_times'] = self.previous_green_times


        print("🚦 Bộ điều khiển chu vi đã được khởi tạo.")
        print(f"Ngưỡng kích hoạt: n(k) > {self.activation_threshold:.0f} xe")
        print(f"Ngưỡng hủy: n(k) < {self.deactivation_threshold:.0f} xe")
        print(f"Tham số: KP={kp:.1f} h⁻¹, KI={ki:.1f} h⁻¹")
        print(f"Số intersection: {len(self.intersection_ids)}")
        print(f"Intersection IDs: {self.intersection_ids}")

    def check_activation_status(self, n_k: float):
        """Kiểm tra xem có nên kích hoạt hay hủy bộ điều khiển"""
        if n_k > self.activation_threshold:
            if not self.is_active:
                print(f"KÍCH HOẠT ĐIỀU KHIỂN CHU VI (n(k)={n_k:.0f} > {self.activation_threshold:.0f})")
            self.is_active = True
        elif n_k < self.deactivation_threshold:
            if self.is_active:
                print(f"HỦY ĐIỀU KHIỂN CHU VI (n(k)={n_k:.0f} < {self.deactivation_threshold:.0f})")
            self.is_active = False
        
        if self.shared_dict is not None:
            self.shared_dict['is_active'] = self.is_active

    def calculate_target_inflow(self, n_k: float, n_k_minus_1: float, qg_k_minus_1: float) -> float:
        """
        BƯỚC 2: Tính toán lưu lượng vào mục tiêu (qg) bằng công thức PI
        qg(k) = qg(k-1) - Kp[n(k) - n(k-1)] + Ki[n_hat - n(k)]
        Đơn vị của qg(k) là [xe/giây] - sau khi đã nhân với control_interval_h
        """
        error = self.n_hat - n_k
        change_in_n = n_k - n_k_minus_1

        # Lưu ý: self.kp và self.ki đã được nhân với control_interval_h
        # nên kết quả qg_k vẫn giữ nguyên đơn vị của qg_k_minus_1 (xe/giờ)
        qg_k = qg_k_minus_1 - (self.kp / (CONTROL_INTERVAL_S / 3600.0)) * change_in_n + (self.ki / (CONTROL_INTERVAL_S / 3600.0)) * error
        
        print(f" Sai số: e(k) = {self.n_hat:.0f} - {n_k:.0f} = {error:.1f} xe")
        print(f" Thay đổi: Δn(k) = {n_k:.0f} - {n_k_minus_1:.0f} = {change_in_n:.1f} xe")
        print(f" PI Output: qg(k) = {qg_k:.2f} xe/giờ")
        
        return max(0, qg_k)  # Đảm bảo không âm

    def solve_optimization_problem(self, target_inflow: float, live_queue_lengths: Optional[Dict] = None) -> Optional[Dict]:
        """
        BƯỚC 3: Giải bài toán tối ưu hóa để chuyển đổi qg thành thời gian đèn xanh
        SỬ DỤNG PYSCIPOPT ĐỂ GIẢI BÀI TOÁN MIQP PHI TUYẾN
        """
        # Chuyển đổi từ xe/giờ sang xe/chu kỳ đèn
        cycle_length = self.global_params['default_cycle_length']
        qg_prime = target_inflow * cycle_length / 3600.0
        
        print(f"🔧 Giải bài toán MIQP với mục tiêu qg = {target_inflow:.2f} [xe/giờ]")
        print(f"   (Tương đương {qg_prime:.2f} [xe / chu kỳ đèn {cycle_length}s])")

        model = Model("MIQP_PerimeterControl")

        # Lấy tham số từ config manager
        theta_1 = self.global_params['theta_1']
        theta_2 = self.global_params['theta_2']
        min_green = self.global_params['min_green_time']
        max_change = self.global_params['max_change']

        # Thêm biến quyết định G_main và G_secondary
        G_vars = {}
        for int_id in self.intersection_ids:
            max_green = self.config_manager.get_cycle_length(int_id) - min_green
            G_main = model.addVar(f'G_{int_id}_main', vtype='INTEGER', lb=min_green, ub=max_green)
            G_sec = model.addVar(f'G_{int_id}_secondary', vtype='INTEGER', lb=min_green, ub=max_green)
            G_vars[int_id] = {'main': G_main, 'secondary': G_sec}

        # Thêm ràng buộc
        for int_id in self.intersection_ids:
            # Ràng buộc 1: Tổng thời gian xanh = chu kỳ đèn
            current_cycle = self.config_manager.get_cycle_length(int_id)
            model.addCons(G_vars[int_id]['main'] + G_vars[int_id]['secondary'] == current_cycle, f"cons_cycle_{int_id}")

            # Ràng buộc 2: Giới hạn thay đổi so với chu kỳ trước
            prev_main = self.previous_green_times[int_id]['main']
            prev_sec = self.previous_green_times[int_id]['secondary']
            model.addCons(G_vars[int_id]['main'] >= prev_main - max_change, f"cons_G_main_min_{int_id}")
            model.addCons(G_vars[int_id]['main'] <= prev_main + max_change, f"cons_G_main_max_{int_id}")
            model.addCons(G_vars[int_id]['secondary'] >= prev_sec - max_change, f"cons_G_sec_min_{int_id}")
            model.addCons(G_vars[int_id]['secondary'] <= prev_sec + max_change, f"cons_G_sec_max_{int_id}")

        # Xây dựng hàm mục tiêu phi tuyến
        # Thành phần 1: Tối thiểu hóa độ lệch so với lưu lượng mục tiêu
        inflow_expr = quicksum(
            G_vars[int_id]['main'] * self.config_manager.get_saturation_flows(int_id)['main'] * self.config_manager.get_turn_in_ratios(int_id)['main']
            for int_id in self.intersection_ids
        )
        deviation = inflow_expr - qg_prime
        first_component = theta_1 * (deviation**2)

        # Thành phần 2: Tối đa hóa việc sử dụng đèn xanh (dựa trên hàng đợi)
        utilization_expr = quicksum(
            (1 - (G_vars[int_id]['main'] * self.config_manager.get_saturation_flows(int_id)['main']) / 
                ((live_queue_lengths[int_id]['main'] if live_queue_lengths and int_id in live_queue_lengths else self.config_manager.get_queue_lengths(int_id)['main']) + 1))**2 +
            (1 - (G_vars[int_id]['secondary'] * self.config_manager.get_saturation_flows(int_id)['secondary']) / 
                ((live_queue_lengths[int_id]['secondary'] if live_queue_lengths and int_id in live_queue_lengths else self.config_manager.get_queue_lengths(int_id)['secondary']) + 1))**2
            for int_id in self.intersection_ids
        )
        second_component = theta_2 * utilization_expr

        # Thiết lập mục tiêu cho model
        # set_nonlinear_objective(model, first_component + second_component, "minimize")
        model.setObjective(first_component + second_component, "minimize")

        # Giải bài toán
        model.hideOutput()
        model.optimize()

        # Xử lý kết quả
        if model.getStatus() == "optimal":
            print(f"  Tìm được nghiệm: {model.getStatus()}")
            result = {
                'status': SolverStatus.OPTIMAL,
                'objective_value': model.getObjVal(),
                'variables': {var.name: model.getVal(var) for var in model.getVars()}
            }
            
            # Cập nhật giá trị trước đó
            new_green_times = {}
            for int_id in self.intersection_ids:
                new_green_times[int_id] = {
                    'main': int(result['variables'][f'G_{int_id}_main']),
                    'secondary': int(result['variables'][f'G_{int_id}_secondary'])
                }
            self.previous_green_times = new_green_times
            
            # Cập nhật shared_dict nếu có
            if self.shared_dict is not None:
                self.shared_dict['green_times'] = new_green_times

            return result
        else:
            print(f"  Không tìm được nghiệm: {model.getStatus()}")
            return None

    def distribute_inflow_to_green_times(self, target_inflow: float, live_queue_lengths: Optional[Dict] = None):
        """Phân bổ lưu lượng mục tiêu thành thời gian đèn xanh"""
        result = self.solve_optimization_problem(target_inflow, live_queue_lengths)
        
        if result:
            print(f"  Thời gian đèn xanh mới:")
            total_inflow = 0
            
            for intersection_id in self.intersection_ids:
                G_main = result['variables'][f'G_{intersection_id}_main']
                G_secondary = result['variables'][f'G_{intersection_id}_secondary']
                
                # Tính lưu lượng vào dự kiến
                saturation_flows = self.config_manager.get_saturation_flows(intersection_id)
                turn_in_ratios = self.config_manager.get_turn_in_ratios(intersection_id)
                
                inflow_main = (G_main * saturation_flows['main'] * turn_in_ratios['main'])
                total_inflow += inflow_main
                
                print(f"   {intersection_id}: G_main={G_main:.0f}s, G_secondary={G_secondary:.0f}s, inflow={inflow_main:.1f} xe/chu kỳ")
            
            print(f"  Tổng lưu lượng dự kiến: {total_inflow:.2f} xe/chu kỳ")
            return result
        
        return None

    def run_simulation_step(self, n_current: float, n_previous: float, qg_previous: float, live_queue_lengths: Optional[Dict] = None) -> Tuple[float, float, bool]:
        """
        Chạy một bước điều khiển chu vi (1 vòng lặp)
        Trả về (n_current, qg_new, controller_active)
        """
        print(f"\n{'='*60}")
        print(f"🔍 BƯỚC 1: Đo lường - Trạng thái hiện tại: n(k) = {n_current:.0f} xe")

        self.check_activation_status(n_current)

        if not self.is_active:
            print("Mục tiêu đã đạt được. Bộ điều khiển không hoạt động.")
            print(f"{ '='*60}\n")
            return n_current, qg_previous, False

        print(f" BƯỚC 2: Tính toán lưu lượng mục tiêu qg")
        qg_new = self.calculate_target_inflow(
            n_k=n_current,
            n_k_minus_1=n_previous,
            qg_k_minus_1=qg_previous
        )

        print(f" BƯỚC 3: Phân bổ thành thời gian đèn xanh")
        self.distribute_inflow_to_green_times(qg_new, live_queue_lengths)
        
        print(f"{ '='*60}\n")
        return n_current, qg_new, True

def simulate_perimeter_control():
    """
    Mô phỏng hoạt động của hệ thống điều khiển chu vi
    """
    print("🚦 BẮT ĐẦU MÔ PHỎNG HỆ THỐNG ĐIỀU KHIỂN CHU VI")
    print("="*70)
    
    # Khởi tạo bộ điều khiển
    # Giả định file "intersection_config.json" tồn tại và hợp lệ
    try:
        controller = PerimeterController(config_file="intersection_config.json")
    except FileNotFoundError:
        print("\n[LỖI] Không tìm thấy file 'intersection_config.json'.")
        print("Vui lòng đảm bảo file cấu hình tồn tại trong cùng thư mục.")
        return
    
    # Dữ liệu mô phỏng: tình huống tắc nghẽn dần tăng rồi giảm
    simulation_data = [
        {'step': 1, 'n_k': 100, 'description': 'Giao thông bình thường'},
        {'step': 2, 'n_k': 120, 'description': 'Lưu lượng tăng nhẹ'}, 
        {'step': 3, 'n_k': 140, 'description': 'Gần ngưỡng kích hoạt'},
        {'step': 4, 'n_k': 160, 'description': 'Vượt ngưỡng - Kích hoạt điều khiển'},
        {'step': 5, 'n_k': 170, 'description': 'Tình trạng tắc nghẽn'},
        {'step': 6, 'n_k': 165, 'description': 'Bắt đầu cải thiện'},
        {'step': 7, 'n_k': 140, 'description': 'Tiếp tục giảm'},
        {'step': 8, 'n_k': 110, 'description': 'Dưới ngưỡng hủy - Tắt điều khiển'},
        {'step': 9, 'n_k': 95, 'description': 'Trở lại bình thường'},
    ]
    
    # Khởi tạo trạng thái
    n_previous = 100.0
    qg_previous = 200.0  # xe/giờ
    
    print(f"\n THÔNG TIN MÔ PHỎNG:")
    print(f"   • Ngưỡng mục tiêu n̂: {N_HAT} xe")
    print(f"   • Khoảng điều khiển: {CONTROL_INTERVAL_S}s")
    print(f"   • Số bước mô phỏng: {len(simulation_data)} bước")
    print("\n" + "="*70)
    
    # Chạy mô phỏng
    for data in simulation_data:
        step = data['step']
        n_current = data['n_k']
        description = data['description']
        
        print(f"\n CHU KỲ {step}: {description}")
        
        n_result, qg_result, active = controller.run_simulation_step(
            n_current, n_previous, qg_previous
        )
        
        # Cập nhật cho chu kỳ tiếp theo
        n_previous = n_current
        qg_previous = qg_result
        
        # Thêm delay để quan sát (có thể bỏ trong thực tế)
        time.sleep(1)
    
    print(" KẾT THÚC MÔ PHỎNG")
    print("="*70)

def example_simple_optimization():
    """Ví dụ đơn giản sử dụng MIQPSolver (KHÔNG THAY ĐỔI)"""
    print("\n" + "="*50)
    print(" VÍ DỤ Đơn GIẢN - SỬ DỤNG MIQPSolver")
    print("="*50)
    
    solver = MIQPSolver('SCIP')
    
    # Thêm biến
    solver.add_variable('x', VariableType.CONTINUOUS, 0, 10)
    solver.add_variable('y', VariableType.INTEGER, 0, 5)
    
    # Thêm ràng buộc: x + 2y <= 10
    solver.add_constraint({'x': 1, 'y': 2}, '<=', 10)
    
    # Mục tiêu: minimize 3x - y
    solver.set_objective_linear({'x': 3, 'y': -1}, ObjectiveType.MINIMIZE)
    
    # Giải và hiển thị kết quả
    result = solver.solve()
    solver.print_solution(result)


if __name__ == "__main__":
    # Chạy mô phỏng hệ thống điều khiển chu vi chính
    simulate_perimeter_control()
    
    # Chạy ví dụ đơn giản
    example_simple_optimization()
