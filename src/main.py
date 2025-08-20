import traci
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
    run_perimeter_control_mock_test
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
                            new_times = current_green_times[int_id]
                            
                            try:
                                logic = traci.trafficlight.getCompleteRedYellowGreenDefinition(tl_id)[0]
                                
                                for phase_index in phase_info.get('main_phases', []):
                                    if 0 <= phase_index < len(logic.phases):
                                        logic.phases[phase_index].duration = new_times['main']

                                for phase_index in phase_info.get('secondary_phases', []):
                                    if 0 <= phase_index < len(logic.phases):
                                        logic.phases[phase_index].duration = new_times['secondary']
                                
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

        # Khởi tạo bộ điều khiển
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
        total_simulation_steps = 1200

        # Lấy giá trị ban đầu
        sumo_sim.step()
        for detector_id in algorithm_detector_ids:
            try:
                n_previous += traci.lanearea.getLastStepVehicleNumber(detector_id)
            except traci.TraCIException as e:
                print(f"[WARNING] Could not get initial data for {detector_id}: {e}")

        try:
            while sumo_sim.get_step_counts() < total_simulation_steps:
                sumo_sim.step()

                if sumo_sim.get_step_counts() % CONTROL_INTERVAL_S == 0:
                    n_current = 0
                    for detector_id in algorithm_detector_ids:
                        try:
                            n_current += traci.lanearea.getLastStepVehicleNumber(detector_id)
                        except traci.TraCIException as e:
                            print(f"[WARNING] Could not get data for {detector_id}: {e}")

                    live_queue_lengths = {}
                    for int_id, detectors in solver_detectors.items():
                        try:
                            main_queue = sum(traci.lanearea.getLastStepVehicleNumber(d) for d in detectors.get('main_queue_detector', []))
                            sec_queue = sum(traci.lanearea.getLastStepVehicleNumber(d) for d in detectors.get('secondary_queue_detector', []))
                            live_queue_lengths[int_id] = {'main': main_queue, 'secondary': sec_queue}
                        except traci.TraCIException as e:
                            print(f"[WARNING] Could not get queue data for {int_id}: {e}")
                            live_queue_lengths[int_id] = {'main': 0, 'secondary': 0}
                    
                    print(f"--- Running Control Step at Sim Time: {sumo_sim.get_step_counts()}s ---")
                    _, qg_previous, _ = controller.run_simulation_step(
                        n_current, n_previous, qg_previous, live_queue_lengths
                    )
                    n_previous = n_current
                
                if sumo_sim.get_step_counts() % 10 == 0:
                    print(f"Step {sumo_sim.get_step_counts()}s / {total_simulation_steps}s")

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
    # Mặc định chạy mô phỏng SUMO thực tế.
    # Để chạy mô phỏng thử nghiệm với dữ liệu giả lập, dùng lệnh:
    # python src/main.py mock
    
    if len(sys.argv) > 1 and sys.argv[1] == 'mock':
        # Gọi hàm chạy thử nghiệm đã được chuyển vào module algo
        run_perimeter_control_mock_test()
    else:
        run_sumo_simulation()