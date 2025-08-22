"""
Lớp Điều khiển Chu vi Phản hồi (Perimeter Control)

Tác giả: Sơn Đình and Đức Ngô

MODIFIED: Tách biệt logic của bộ giải (solver) sang module riêng,
thêm hàm chạy thử nghiệm (mock test), và hỗ trợ nhiều pha phụ.
"""

import time
import os
import sys
from typing import Dict, Optional, Tuple

# Thêm project root vào sys.path để giải quyết vấn đề import
# sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from data.intersection_config_manager import IntersectionConfigManager
from algorithm.solver import solve_green_time_optimization

# === CONSTANTS ===
KP_H = 37.45
KI_H = 19.01
N_HAT = 75.0
CONTROL_INTERVAL_S = 90

class PerimeterController:
    """
    Lớp điều khiển chu vi phản hồi.
    """
    
    def __init__(self, kp: float = KP_H, ki: float = KI_H, n_hat: float = N_HAT, 
                 config_file: str = "src/intersection_config.json", shared_dict: Optional[Dict] = None,
                 control_interval_s: int = CONTROL_INTERVAL_S):
        control_interval_h = control_interval_s / 3600.0
        self.kp = kp * control_interval_h
        self.ki = ki * control_interval_h
        self.n_hat = n_hat
        
        self.shared_dict = shared_dict
        self.is_active = False

        self.activation_threshold = 0.85 * self.n_hat
        self.deactivation_threshold = 0.70 * self.n_hat

        self.config_manager = IntersectionConfigManager(config_file)
        self.intersection_ids = self.config_manager.get_intersection_ids()
        
        self.previous_green_times = {}
        for int_id in self.intersection_ids:
            tl_id = self.config_manager.get_traffic_light_id(int_id)
            if not tl_id:
                print(f"Warning: No traffic_light_id found for intersection {int_id}. Skipping initialization for this intersection.")
                continue

            traffic_light_phases = self.config_manager.config_data.get('traffic_lights', {}).get(tl_id, {}).get('phases', [])
            phase_info = self.config_manager.get_phase_info(int_id)

            if not traffic_light_phases or not phase_info: 
                print(f"Warning: No phase info found for traffic light {tl_id} or intersection {int_id}. Skipping initialization for this intersection.")
                continue

            main_phase_duration = 0
            if 'p' in phase_info and 'phase_indices' in phase_info['p']:
                main_phase_index = phase_info['p']['phase_indices'][0]
                if 0 <= main_phase_index < len(traffic_light_phases):
                    main_phase_duration = int(traffic_light_phases[main_phase_index].get('duration', 0))

            secondary_phase_durations = []
            if 's' in phase_info:
                for s_phase_config in phase_info['s']:
                    if 'phase_indices' in s_phase_config:
                        s_phase_index = s_phase_config['phase_indices'][0]
                        if 0 <= s_phase_index < len(traffic_light_phases):
                            secondary_phase_durations.append(int(traffic_light_phases[s_phase_index].get('duration', 0)))

            self.previous_green_times[int_id] = {
                'p': main_phase_duration,
                's': secondary_phase_durations
            }
        
        if self.shared_dict is not None:
            self.shared_dict['is_active'] = self.is_active
            self.shared_dict['green_times'] = self.previous_green_times

        print("🚦Bộ điều khiển chu vi đã được khởi tạo (hỗ trợ nhiều pha phụ).")
        print(f"Ngưỡng kích hoạt: n(k) > {self.activation_threshold:.0f} xe")
        print(f"Ngưỡng hủy: n(k) < {self.deactivation_threshold:.0f} xe")
        print(f"Số intersection: {len(self.intersection_ids)}")

    def check_activation_status(self, n_k: float):
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
        error = self.n_hat - n_k
        change_in_n = n_k - n_k_minus_1
        qg_k = qg_k_minus_1 - (self.kp / (CONTROL_INTERVAL_S / 3600.0)) * change_in_n + (self.ki / (CONTROL_INTERVAL_S / 3600.0)) * error
        print(f" Sai số: e(k) = {self.n_hat:.0f} - {n_k:.0f} = {error:.1f} xe")
        print(f" Thay đổi: Δn(k) = {n_k:.0f} - {n_k_minus_1:.0f} = {change_in_n:.1f} xe")
        print(f" PI Output: qg(k) = {qg_k:.2f} xe/giờ")
        return max(0, qg_k)

    def distribute_inflow_to_green_times(self, target_inflow: float, live_queue_lengths: Optional[Dict] = None):
        result = solve_green_time_optimization(
            target_inflow=target_inflow,
            config_manager=self.config_manager,
            previous_green_times=self.previous_green_times,
            live_queue_lengths=live_queue_lengths
        )
        
        if result:
            print(f"  Thời gian đèn xanh mới:")
            total_inflow = 0
            new_green_times = {}

            for int_id in self.intersection_ids:
                phase_info = self.config_manager.get_phase_info(int_id)
                if not phase_info: continue

                G_p = result['variables'][f'G_{int_id}_p']
                new_green_times[int_id] = {'p': int(G_p), 's': []}

                inflow_p = (G_p * phase_info['p']['saturation_flow'] * phase_info['p']['turn_in_ratio'])
                total_inflow += inflow_p
                print(f"   {int_id}: G_p={G_p:.0f}s, inflow={inflow_p:.1f} xe/chu kỳ")

                if 's' in phase_info:
                    for i, _ in enumerate(phase_info['s']):
                        G_s = result['variables'][f'G_{int_id}_s_{i}']
                        new_green_times[int_id]['s'].append(int(G_s))
                        print(f"     └─ G_s{i}={G_s:.0f}s")
            
            self.previous_green_times = new_green_times
            if self.shared_dict is not None:
                self.shared_dict['green_times'] = new_green_times

            print(f"  Tổng lưu lượng dự kiến (từ các pha chính): {total_inflow:.2f} xe/chu kỳ")
        else:
            print("  Không tìm được nghiệm tối ưu, giữ nguyên thời gian đèn xanh.")

    def run_simulation_step(self, n_current: float, n_previous: float, qg_previous: float, live_queue_lengths: Optional[Dict] = None) -> Tuple[float, float, bool]:
        print(f"\n{'='*60}")
        print(f"🔍 BƯỚC 1: Đo lường - Trạng thái hiện tại: n(k) = {n_current:.0f} xe")
        self.check_activation_status(n_current)

        if not self.is_active:
            print("Mục tiêu đã đạt được. Bộ điều khiển không hoạt động.")
            print(f"{ '='*60}\n")
            return n_current, qg_previous, False

        print(f" BƯỚC 2: Tính toán lưu lượng mục tiêu qg")
        qg_new = self.calculate_target_inflow(n_k=n_current, n_k_minus_1=n_previous, qg_k_minus_1=qg_previous)

        print(f" BƯỚC 3: Phân bổ thành thời gian đèn xanh")
        self.distribute_inflow_to_green_times(qg_new, live_queue_lengths)
        
        print(f"{ '='*60}\n")
        return n_current, qg_new, True

def run_perimeter_control_mock_test():
    print("🚦 BẮT ĐẦU MÔ PHỎNG THỬ NGHIỆM (MOCK TEST)")
    print("="*70)
    
    try:
        # Chạy từ thư mục gốc của dự án, nên đường dẫn tới config cần là "src/...."
        controller = PerimeterController(config_file="src/intersection_config.json")
    except Exception as e:
        print(f"\n[LỖI] Không thể khởi tạo bộ điều khiển: {e}")
        print("Vui lòng kiểm tra file 'src/intersection_config.json' và đảm bảo bạn đang chạy từ thư mục gốc của dự án.")
        return
    
    simulation_data = [
        {'step': 1, 'n_k': 80, 'description': 'Giao thông bình thường'},
        {'step': 2, 'n_k': 90, 'description': 'Lưu lượng tăng, gần ngưỡng'},
        {'step': 3, 'n_k': 100, 'description': 'Vượt ngưỡng - Kích hoạt'},
        {'step': 4, 'n_k': 110, 'description': 'Tắc nghẽn'},
        {'step': 5, 'n_k': 105, 'description': 'Bắt đầu cải thiện'},
        {'step': 6, 'n_k': 95, 'description': 'Tiếp tục giảm'},
        {'step': 7, 'n_k': 80, 'description': 'Dưới ngưỡng - Hủy kích hoạt'},
        {'step': 8, 'n_k': 75, 'description': 'Trở lại bình thường'},
    ]
    
    n_previous = 80.0
    qg_previous = 250.0
    
    print(f"\n THÔNG TIN MÔ PHỎNG:")
    print(f"   • Ngưỡng mục tiêu n̂: {N_HAT} xe")
    print(f"   • Khoảng điều khiển: {CONTROL_INTERVAL_S}s")
    print(f"   • Số bước mô phỏng: {len(simulation_data)} bước")
    print("\n" + "="*70)
    
    for data in simulation_data:
        step = data['step']
        n_current = data['n_k']
        description = data['description']
        
        print(f"\n CHU KỲ {step}: {description}")
        
        _, qg_result, _ = controller.run_simulation_step(
            n_current, n_previous, qg_previous
        )
        
        n_previous = n_current
        qg_previous = qg_result
        
        time.sleep(0.5)
    
    print(" KẾT THÚC MÔ PHỎNG THỬ NGHIỆM")
    print("="*70)

if __name__ == '__main__':
    run_perimeter_control_mock_test()