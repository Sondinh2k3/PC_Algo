"""
Module giải bài toán tối ưu hóa phân bổ thời gian đèn xanh.
Sử dụng PySCIPOpt để giải bài toán MIQP (Mixed-Integer Quadratic Programming) phi tuyến.
"""

from pyscipopt import Model, quicksum
from pyscipopt.recipes.nonlinear import set_nonlinear_objective
from typing import Dict, Optional

# Import các thành phần chung và lớp quản lý config
# Sửa lỗi import: Bỏ dấu . và .. để dùng import tuyệt đối từ src
from algorithm.common import SolverStatus
from data.intersection_config_manager import IntersectionConfigManager

def solve_green_time_optimization(
    target_inflow: float,
    config_manager: IntersectionConfigManager,
    previous_green_times: Dict,
    live_queue_lengths: Optional[Dict] = None
) -> Optional[Dict]:
    """
    Giải bài toán tối ưu hóa để chuyển đổi lưu lượng mục tiêu (qg) thành thời gian đèn xanh.

    Args:
        target_inflow: Lưu lượng vào mục tiêu qg(k) [xe/giờ].
        config_manager: Đối tượng quản lý cấu hình intersection.
        previous_green_times: Dict chứa thời gian xanh của chu kỳ trước.
        live_queue_lengths: Dict chứa độ dài hàng đợi thực tế từ mô phỏng.

    Returns:
        Một dict chứa kết quả nếu tìm thấy nghiệm tối ưu, ngược lại trả về None.
    """
    # Lấy các tham số và ID từ config manager
    global_params = config_manager.get_global_params()
    intersection_ids = config_manager.get_intersection_ids()

    # Chuyển đổi từ xe/giờ sang xe/chu kỳ đèn
    cycle_length = global_params.get('default_cycle_length', 90)
    qg_prime = target_inflow * cycle_length / 3600.0
    
    print(f"🔧 Giải bài toán MIQP với mục tiêu qg = {target_inflow:.2f} [xe/giờ]")
    print(f"   (Tương đương {qg_prime:.2f} [xe / chu kỳ đèn {cycle_length}s])")

    model = Model("MIQP_PerimeterControl")

    # Lấy tham số từ config
    theta_1 = global_params.get('theta_1', 1.0)
    theta_2 = global_params.get('theta_2', 0.5)
    min_green = global_params.get('min_green_time', 15)
    max_change = global_params.get('max_change', 5)

    # Thêm biến quyết định G_main và G_secondary
    G_vars = {}
    for int_id in intersection_ids:
        max_green = config_manager.get_cycle_length(int_id) - min_green
        G_main = model.addVar(f'G_{int_id}_main', vtype='INTEGER', lb=min_green, ub=max_green)
        G_sec = model.addVar(f'G_{int_id}_secondary', vtype='INTEGER', lb=min_green, ub=max_green)
        G_vars[int_id] = {'main': G_main, 'secondary': G_sec}

    # Thêm ràng buộc
    for int_id in intersection_ids:
        # Ràng buộc 1: Tổng thời gian xanh = chu kỳ đèn
        current_cycle = config_manager.get_cycle_length(int_id)
        model.addCons(G_vars[int_id]['main'] + G_vars[int_id]['secondary'] == current_cycle, f"cons_cycle_{int_id}")

        # Ràng buộc 2: Giới hạn thay đổi so với chu kỳ trước
        prev_main = previous_green_times[int_id]['main']
        prev_sec = previous_green_times[int_id]['secondary']
        model.addCons(G_vars[int_id]['main'] >= prev_main - max_change, f"cons_G_main_min_{int_id}")
        model.addCons(G_vars[int_id]['main'] <= prev_main + max_change, f"cons_G_main_max_{int_id}")
        model.addCons(G_vars[int_id]['secondary'] >= prev_sec - max_change, f"cons_G_sec_min_{int_id}")
        model.addCons(G_vars[int_id]['secondary'] <= prev_sec + max_change, f"cons_G_sec_max_{int_id}")

    # Xây dựng hàm mục tiêu phi tuyến
    # Thành phần 1: Tối thiểu hóa độ lệch so với lưu lượng mục tiêu
    inflow_expr = quicksum(
        G_vars[int_id]['main'] * config_manager.get_saturation_flows(int_id)['main'] * config_manager.get_turn_in_ratios(int_id)['main']
        for int_id in intersection_ids
    )
    deviation = inflow_expr - qg_prime
    first_component = theta_1 * (deviation**2)

    # Thành phần 2: Tối đa hóa việc sử dụng đèn xanh (dựa trên hàng đợi)
    # Sử dụng giá trị hàng đợi thực tế nếu có, nếu không thì dùng giá trị mặc định từ config
    utilization_expr = quicksum(
        (1 - (G_vars[int_id]['main'] * config_manager.get_saturation_flows(int_id)['main']) / 
            ((live_queue_lengths[int_id]['main'] if live_queue_lengths and int_id in live_queue_lengths else config_manager.get_queue_lengths(int_id)['main']) + 1))**2 +
        (1 - (G_vars[int_id]['secondary'] * config_manager.get_saturation_flows(int_id)['secondary']) / 
            ((live_queue_lengths[int_id]['secondary'] if live_queue_lengths and int_id in live_queue_lengths else config_manager.get_queue_lengths(int_id)['secondary']) + 1))**2
        for int_id in intersection_ids
    )
    second_component = theta_2 * utilization_expr

    # Thiết lập mục tiêu cho model
    set_nonlinear_objective(model, first_component + second_component, "minimize")

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
        return result
    else:
        print(f"  Không tìm được nghiệm: {model.getStatus()}")
        return None