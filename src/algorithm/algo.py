"""
Lớp Điều khiển Chu vi Phản hồi (Perimeter Control)

Tác giả: Sơn Đình and Đức Ngô

MODIFIED: Tách biệt logic của bộ giải (solver) sang module riêng
và thêm hàm chạy thử nghiệm (mock test).
"""

import time
from typing import Dict, Optional, Tuple

# Import các thành phần từ các module khác trong src
# Sửa lỗi import: Bỏ dấu .. để dùng import tuyệt đối từ src
from data.intersection_config_manager import IntersectionConfigManager
from algorithm.solver import solve_green_time_optimization

# === CONSTANTS ===
KP_H = 20.0        # Proportional gain (1/hour)
KI_H = 5.0         # Integral gain (1/hour)
N_HAT = 90.0      # Target accumulation (vehicles)
CONTROL_INTERVAL_S = 90  # Control interval (seconds)

class PerimeterController:
    """
    Lớp điều khiển chu vi phản hồi.
    Thực hiện thuật toán điều khiển dựa trên bộ điều khiển PI và ủy quyền
    việc giải bài toán tối ưu cho solver module.
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
        
        # Lưu trữ giá trị trước đó cho mỗi intersection
        self.previous_green_times = {}
        for intersection_id in self.intersection_ids:
            intersection_data = self.config_manager.get_intersection_data(intersection_id)
            if intersection_data:
                cycle_length = intersection_data.get('cycle_length', 90)
                # Giả sử chia đều cho pha chính và phụ ban đầu
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
        BƯỚC 2: Tính toán lưu lượng vào mục tiêu (qg) bằng công thức PI.
        """
        error = self.n_hat - n_k
        change_in_n = n_k - n_k_minus_1

        qg_k = qg_k_minus_1 - (self.kp / (CONTROL_INTERVAL_S / 3600.0)) * change_in_n + (self.ki / (CONTROL_INTERVAL_S / 3600.0)) * error
        
        print(f" Sai số: e(k) = {self.n_hat:.0f} - {n_k:.0f} = {error:.1f} xe")
        print(f" Thay đổi: Δn(k) = {n_k:.0f} - {n_k_minus_1:.0f} = {change_in_n:.1f} xe")
        print(f" PI Output: qg(k) = {qg_k:.2f} xe/giờ")
        
        return max(0, qg_k)  # Đảm bảo không âm

    def distribute_inflow_to_green_times(self, target_inflow: float, live_queue_lengths: Optional[Dict] = None):
        """
        BƯỚC 3: Gọi bộ giải để phân bổ lưu lượng mục tiêu thành thời gian đèn xanh.
        """
        # Ủy quyền việc giải bài toán cho solver module
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

            for intersection_id in self.intersection_ids:
                G_main = result['variables'][f'G_{intersection_id}_main']
                G_secondary = result['variables'][f'G_{intersection_id}_secondary']
                
                new_green_times[intersection_id] = {
                    'main': int(G_main),
                    'secondary': int(G_secondary)
                }

                # Tính lưu lượng vào dự kiến để hiển thị
                saturation_flows = self.config_manager.get_saturation_flows(intersection_id)
                turn_in_ratios = self.config_manager.get_turn_in_ratios(intersection_id)
                inflow_main = (G_main * saturation_flows['main'] * turn_in_ratios['main'])
                total_inflow += inflow_main
                
                print(f"   {intersection_id}: G_main={G_main:.0f}s, G_secondary={G_secondary:.0f}s, inflow={inflow_main:.1f} xe/chu kỳ")
            
            # Cập nhật giá trị cho chu kỳ tiếp theo
            self.previous_green_times = new_green_times
            if self.shared_dict is not None:
                self.shared_dict['green_times'] = new_green_times

            print(f"  Tổng lưu lượng dự kiến: {total_inflow:.2f} xe/chu kỳ")
        else:
            print("  Không tìm được nghiệm tối ưu, giữ nguyên thời gian đèn xanh.")

    def run_simulation_step(self, n_current: float, n_previous: float, qg_previous: float, live_queue_lengths: Optional[Dict] = None) -> Tuple[float, float, bool]:
        """
        Chạy một bước điều khiển chu vi (1 vòng lặp).
        Trả về (n_current, qg_new, controller_active).
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

def run_perimeter_control_mock_test():
    """
    Chạy mô phỏng thử nghiệm cho bộ điều khiển chu vi với dữ liệu giả lập.
    Hàm này dùng để kiểm tra nhanh logic của thuật toán mà không cần SUMO.
    """
    print("🚦 BẮT ĐẦU MÔ PHỎNG THỬ NGHIỆM (MOCK TEST)")
    print("="*70)
    
    # Khởi tạo bộ điều khiển
    try:
        # Giả định file cấu hình nằm trong thư mục src
        controller = PerimeterController(config_file="src/intersection_config.json")
    except FileNotFoundError:
        print("\n[LỖI] Không tìm thấy file 'src/intersection_config.json'.")
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
        
        time.sleep(0.5) # Thêm delay nhỏ để quan sát
    
    print(" KẾT THÚC MÔ PHỎNG THỬ NGHIỆM")
    print("="*70)