import traci
# import libsumo as traci
import yaml
import threading
import time
import os
import sys
from multiprocessing import Manager
from typing import Dict

# Import các thành phần từ các module khác
from sumosim import SumoSim
from data.collector.SqlCollector import SqlCollector
from data.intersection_config_manager import IntersectionConfigManager
from data.detector_config_manager import DetectorConfigManager

# Import thuật toán và hàm chạy thử nghiệm từ module algo
from algorithm.algo import (
    PerimeterController, 
    KP_H, 
    KI_H, 
    N_HAT, 
    CONTROL_INTERVAL_S,
)

# =================================================================
# REAL SUMO SIMULATION
# =================================================================

def traffic_light_controller(shared_dict: Dict, config_file: str, stop_event: threading.Event):
    """
    Luồng riêng để điều khiển đèn giao thông dựa trên dữ liệu từ shared_dict.
    """
    print("[CONTROLLER THREAD] Bắt đầu luồng điều khiển đèn.")
    config_manager = IntersectionConfigManager(config_file)
    intersection_ids = config_manager.get_intersection_ids()

    while not stop_event.is_set():
        try:
            if shared_dict.get('is_active', False):
                green_times = shared_dict.get('green_times', None)
                if green_times:
                    current_green_times = green_times.copy()
                    for int_id in intersection_ids:
                        if int_id in current_green_times:
                            tl_id = config_manager.get_traffic_light_id(int_id)
                            if not tl_id:
                                print(f"[CONTROLLER THREAD] Warning: No traffic_light_id for intersection {int_id}.")
                                continue

                            phase_info = config_manager.get_phase_info(int_id)
                            if not phase_info:
                                continue

                            main_phases = phase_info.get('p', {}).get('phase_indices', [])
                            secondary_phases = [s_phase['phase_indices'][0] for s_phase in phase_info.get('s', []) if s_phase.get('phase_indices')]

                            new_times = current_green_times[int_id]
                            
                            try:
                                logic = traci.trafficlight.getCompleteRedYellowGreenDefinition(tl_id)[0]
                                
                                for phase_index in main_phases:
                                    if 0 <= phase_index < len(logic.phases):
                                        logic.phases[phase_index].duration = new_times['p']

                                for i, phase_index in enumerate(secondary_phases):
                                    if 0 <= phase_index < len(logic.phases) and i < len(new_times['s']):
                                        logic.phases[phase_index].duration = new_times['s'][i]
                                
                                traci.trafficlight.setCompleteRedYellowGreenDefinition(tl_id, logic)
                            except traci.TraCIException as e:
                                print(f"[CONTROLLER THREAD] Error updating TLS {tl_id}: {e}")
            
            time.sleep(1)

        except Exception as e:
            print(f"[CONTROLLER THREAD] Lỗi trong luồng điều khiển: {e}")
            break
    
    print("[CONTROLLER THREAD] Dừng luồng điều khiển đèn.")

def load_simulation_config(config_path: str) -> dict:
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
        if config.get('type') == 'sumo': 
            return config.get('config', {})
        else:
            raise ValueError("Unsupported simulation type. Only 'sumo' is supported.")

def load_application_config(config_path: str) -> dict:
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
        if config is not None:
            return config
        else:
            raise ValueError("Application configuration not found.")

def run_sumo_simulation():
    """Hàm chính để khởi tạo và chạy mô phỏng SUMO thực tế."""
    # --- CÀI ĐẶT CHO VIỆC LẤY MẪU, TỔNG HỢP VÀ ĐIỀU KHIỂN ---
    SAMPLING_INTERVAL_S = 10      # Lấy dữ liệu mỗi 10 giây
    AGGREGATION_INTERVAL_S = 50   # Tổng hợp dữ liệu mỗi 50 giây
    # CONTROL_INTERVAL_S được import từ algo.py và có giá trị là 90 giây

    # Xác định đường dẫn tuyệt đối cho các file cấu hình
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    sim_config_path = os.path.join(project_root, 'src', 'config', 'simulation.yml')
    app_config_path = os.path.join(project_root, 'src', 'config', 'application.yml')
    detector_config_path = os.path.join(project_root, 'src', 'config', 'detector_config.json')
    intersection_config_path = os.path.join(project_root, 'src', 'intersection_config.json')

    # Load cấu hình
    sim_config = load_simulation_config(sim_config_path)
    app_config = load_application_config(app_config_path)
    detector_config_mgr = DetectorConfigManager(detector_config_path)

    # Khởi tạo kết nối DB
    sql_conn = SqlCollector(
        host=app_config['mysql']["host"],
        port=app_config['mysql']["port"],
        user=app_config['mysql']["user"],
        password=app_config['mysql']["password"], 
        database=app_config['mysql']["database"]
    )

    # Lấy ID của các detector
    algorithm_detector_ids = detector_config_mgr.get_algorithm_input_detectors()
    solver_detectors = detector_config_mgr.get_solver_input_detectors()
    print(f"[INFO] Found {len(algorithm_detector_ids)} detectors for algorithm input.")
    print(f"[INFO] Found {len(solver_detectors)} intersections for solver input.")

    with Manager() as manager:
        shared_dict = manager.dict()
        stop_event = threading.Event()

        sumo_sim = SumoSim(sim_config)

        # Định nghĩa đường dẫn output tuyệt đối
        output_dir = os.path.join(project_root, "output")
        os.makedirs(output_dir, exist_ok=True)
        output_filenames = {
            "tripinfo": os.path.join(output_dir, "tripinfo.xml"),
            "vehroute": os.path.join(output_dir, "vehroutes.xml")
        }
        
        sumo_sim.start(output_files=output_filenames)

        # Khởi tạo bộ điều khiển, sử dụng chu kỳ 90s mặc định
        controller = PerimeterController(
            kp=KP_H, ki=KI_H, n_hat=N_HAT, 
            config_file=intersection_config_path,
            shared_dict=shared_dict
        )

        # Bắt đầu luồng điều khiển đèn
        controller_thread = threading.Thread(
            target=traffic_light_controller, 
            args=(shared_dict, intersection_config_path, stop_event)
        )
        controller_thread.start()

        n_previous = 0
        qg_previous = 3600
        total_simulation_steps = 300

        # --- BIẾN LƯU TRỮ DỮ LIỆU ---
        n_samples = []
        queue_samples = {int_id: {'main': [], 'secondary': []} for int_id in solver_detectors.keys()}
        
        # Biến lưu trữ dữ liệu tổng hợp mới nhất
        latest_aggregated_n = 0
        latest_aggregated_queue_lengths = {}

        # Lấy giá trị ban đầu
        sumo_sim.step()
        try:
            n_previous = sum(traci.lanearea.getLastStepVehicleNumber(det_id) for det_id in algorithm_detector_ids)
            latest_aggregated_n = n_previous
        except traci.TraCIException as e:
            print(f"[WARNING] Could not get initial data: {e}")
            n_previous = 0

        try:
            while sumo_sim.get_step_counts() < total_simulation_steps:
                current_step = sumo_sim.get_step_counts()
                sumo_sim.step()

                # --- BƯỚC 1: THU THẬP DỮ LIỆU MẪU (mỗi 10 giây) ---
                if current_step % SAMPLING_INTERVAL_S == 0:
                    n_sample = 0
                    for detector_id in algorithm_detector_ids:
                        try:
                            n_sample += traci.lanearea.getLastStepVehicleNumber(detector_id)
                        except traci.TraCIException:
                            pass
                    n_samples.append(n_sample)

                    for int_id, detectors in solver_detectors.items():
                        try:
                            main_q = sum(traci.lanearea.getLastStepVehicleNumber(d) for d in detectors.get('main_queue_detector', []))
                            sec_q = sum(traci.lanearea.getLastStepVehicleNumber(d) for d in detectors.get('secondary_queue_detector', []))
                            queue_samples[int_id]['main'].append(main_q)
                            queue_samples[int_id]['secondary'].append(sec_q)
                        except traci.TraCIException:
                            queue_samples[int_id]['main'].append(0)
                            queue_samples[int_id]['secondary'].append(0)

                # --- BƯỚC 2: TỔNG HỢP DỮ LIỆU (mỗi 50 giây) ---
                if current_step > 0 and current_step % AGGREGATION_INTERVAL_S == 0:
                    print(f"--- Aggregating data at Sim Time: {current_step}s ---")
                    
                    # Tính trung bình và cập nhật vào biến "mới nhất"
                    if n_samples:
                        latest_aggregated_n = sum(n_samples) / len(n_samples)
                        print(f"New aggregated n(k) = {latest_aggregated_n:.2f} (from {len(n_samples)} samples)")

                    for int_id, data in queue_samples.items():
                        avg_main = sum(data['main']) / len(data['main']) if data['main'] else 0
                        avg_sec = sum(data['secondary']) / len(data['secondary']) if data['secondary'] else 0
                        latest_aggregated_queue_lengths[int_id] = {'main': avg_main, 'secondary': avg_sec}

                    # Xóa các mẫu để bắt đầu chu kỳ mới
                    n_samples.clear()
                    for int_id in queue_samples:
                        queue_samples[int_id]['main'].clear()
                        queue_samples[int_id]['secondary'].clear()

                # --- BƯỚC 3: CHẠY THUẬT TOÁN ĐIỀU KHIỂN (mỗi 90 giây) ---
                if current_step > 0 and current_step % CONTROL_INTERVAL_S == 0:
                    print(f"--- Running Control Step at Sim Time: {current_step}s ---")
                    print(f"Using latest aggregated data. n(k) = {latest_aggregated_n:.2f}")
                    
                    # Chạy thuật toán với dữ liệu tổng hợp mới nhất
                    _, qg_previous, _ = controller.run_simulation_step(
                        latest_aggregated_n, n_previous, qg_previous, latest_aggregated_queue_lengths
                    )
                    
                    # Cập nhật giá trị trước đó cho chu kỳ điều khiển tiếp theo
                    n_previous = latest_aggregated_n
                
                if current_step % 10 == 0:
                    print(f"Step {current_step}s / {total_simulation_steps}s")

        except Exception as err:
            print(f"Stop simulation due to: {err}")

        finally:
            print("Hoàn tất mô phỏng. Dừng các luồng...")
            stop_event.set()
            controller_thread.join()
            try:
                sumo_sim.close()
                sql_conn.close()
            except Exception as e:
                print(f"Error closing connections: {e}")
            print("Simulation finished. Total steps:", sumo_sim.get_step_counts())


if __name__ == "__main__":
    run_sumo_simulation()