import traci
import yaml
import threading
import time
import os
from enum import Enum
from typing import Any, Callable, Dict, Optional
import sched
from datetime import datetime
import sys
from multiprocessing import Manager

from neo4j import GraphDatabase

from traci import trafficlight, lanearea, inductionloop
import libsumo

from sumosim import SumoSim

from data.collector.SqlCollector import SqlCollector
from algorithm.algo import PerimeterController, KP_H, KI_H, N_HAT, CONTROL_INTERVAL_S
from data.intersection_config_manager import IntersectionConfigManager
from data.detector_config_manager import DetectorConfigManager


# =================================================================
# MOCK SIMULATION (from old algo.py)
# This simulation uses mock data to test the controller logic
# without needing a SUMO connection.
# =================================================================
def run_mock_simulation():
    """
    Mô phỏng hoạt động của hệ thống điều khiển chu vi với dữ liệu giả lập.
    """
    print("🚦 BẮT ĐẦU MÔ PHỎNG (DỮ LIỆU GIẢ LẬP)")
    print("="*70)
    
    # Khởi tạo bộ điều khiển
    try:
        # Path is relative to the execution folder (src)
        controller = PerimeterController(config_file="intersection_config.json")
    except FileNotFoundError:
        print("\n[LỖI] Không tìm thấy file 'intersection_config.json'.")
        print("Vui lòng đảm bảo file cấu hình tồn tại trong thư mục src.")
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
    # print(f"   • Khoảng điều khiển: {controller.control_interval_h * 3600}s")
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
        
        # Thêm delay để quan sát
        time.sleep(1)
    
    print(" KẾT THÚC MÔ PHỎNG (DỮ LIỆU GIẢ LẬP)")
    print("="*70)


# =================================================================
# REAL SUMO SIMULATION (original main.py logic)
# This simulation connects to SUMO and a database for a live run.
# =================================================================

def traffic_light_controller(shared_dict: Dict, config_file: str, stop_event: threading.Event):
    """
    Luồng riêng để điều khiển đèn giao thông dựa trên dữ liệu từ shared_dict.
    """
    print("[CONTROLLER THREAD] Bắt đầu luồng điều khiển đèn.")
    config_manager = IntersectionConfigManager(config_file)
    # intersection_ids are now ["junction01", "junction02", ...]
    intersection_ids = config_manager.get_intersection_ids()

    while not stop_event.is_set():
        try:
            if shared_dict.get('is_active', False):
                green_times = shared_dict.get('green_times', None)
                if green_times:
                    # Tạo một bản sao để tránh lỗi thay đổi kích thước dict trong khi lặp
                    current_green_times = green_times.copy()
                    for int_id in intersection_ids: # int_id is "junction01", etc.
                        if int_id in current_green_times:
                            # Get the actual traffic light ID for SUMO
                            tl_id = config_manager.get_traffic_light_id(int_id)
                            if not tl_id:
                                print(f"[CONTROLLER THREAD] Warning: No traffic_light_id found for intersection {int_id}. Skipping.")
                                continue

                            phase_info = config_manager.get_phase_info(int_id)
                            new_times = current_green_times[int_id]
                            
                            # Get the current program logic for the traffic light
                            try:
                                logic = traci.trafficlight.getCompleteRedYellowGreenDefinition(tl_id)[0]
                                
                                # Update the duration for main phases
                                for phase_index in phase_info.get('main_phases', []):
                                    if 0 <= phase_index < len(logic.phases):
                                        logic.phases[phase_index].duration = new_times['main']

                                # Update the duration for secondary phases
                                for phase_index in phase_info.get('secondary_phases', []):
                                    if 0 <= phase_index < len(logic.phases):
                                        logic.phases[phase_index].duration = new_times['secondary']
                                
                                # Set the new program logic for the traffic light
                                traci.trafficlight.setCompleteRedYellowGreenDefinition(tl_id, logic)
                            except traci.TraCIException as e:
                                print(f"[CONTROLLER THREAD] Error updating TLS {tl_id} for intersection {int_id}: {e}")
            
            # Tạm dừng để giảm tải CPU
            time.sleep(1) 

        except Exception as e:
            print(f"[CONTROLLER THREAD] Lỗi trong luồng điều khiển: {e}")
            # Có thể thêm logic để thử kết nối lại hoặc thoát một cách an toàn
            break
    
    print("[CONTROLLER THREAD] Dừng luồng điều khiển đèn.")


# Đọc file cấu hình mô phỏng và trích xuất ra phần cấu hình giành riêng cho SUMO
def load_simulation_config(config_path)-> dict:
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
        if config['type'] == 'sumo': 
            return config['config']
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
    # Corrected paths to be relative to the `src` directory
    sim_config = load_simulation_config(config_path="config/simulation.yml")
    app_config = load_application_config(config_path="config/application.yml")
    detector_config_mgr = DetectorConfigManager(config_file="config/detector_config.json")

    # Initialize database connection
    if app_config is None:
        raise ValueError("Failed to load application configuration.")
    else:
        sql_conn = SqlCollector(
            host=app_config['mysql']["host"],
            port=app_config['mysql']["port"],
            user=app_config['mysql']["user"],
            password=app_config['mysql']["password"], 
            database=app_config['mysql']["database"]
        )

    # Get detector IDs from the new config manager
    algorithm_detector_ids = detector_config_mgr.get_algorithm_input_detectors()
    solver_detectors = detector_config_mgr.get_solver_input_detectors()
    print(f"[INFO] Found {len(algorithm_detector_ids)} detectors for algorithm input.")
    print(f"[INFO] Found {len(solver_detectors)} intersections with detectors for solver input.")

    # Sử dụng Manager để tạo shared_dict
    with Manager() as manager:
        shared_dict = manager.dict()
        stop_event = threading.Event()

        sumo_sim = SumoSim(sim_config)

        # --- NEW: Use absolute paths for output files to avoid errors ---
        # Get project root by going up one level from the current file's directory (src)
        project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        output_dir = os.path.join(project_root, "output")
        
        # Ensure the output directory exists
        os.makedirs(output_dir, exist_ok=True)

        # Define absolute paths for the output files
        output_filenames = {
            "tripinfo": os.path.join(output_dir, "tripinfo.xml"),
            "edgedata": os.path.join(output_dir, "edgedata.xml")
        }
        print(output_filenames)
        
        # Start SUMO with absolute paths for output files
        sumo_sim.start(output_files=output_filenames)

        # Khởi tạo bộ điều khiển chu vi với shared_dict
        intersection_config_path = "intersection_config.json"
        controller = PerimeterController(
            kp=KP_H, 
            ki=KI_H, 
            n_hat=N_HAT, 
            config_file=intersection_config_path,
            shared_dict=shared_dict
        )

        # Bắt đầu luồng điều khiển đèn
        controller_thread = threading.Thread(
            target=traffic_light_controller, 
            args=(shared_dict, intersection_config_path, stop_event)
        )
        controller_thread.start()

        n_current = 0
        n_previous = 0
        qg_previous = 3600

        # --- NEW: Corrected Simulation Loop ---
        total_simulation_steps = 1860 # Total simulation time in seconds (e.g., 1 hour)
        
        # Initialize n_previous with initial vehicle count
        sumo_sim.step()
        n_previous = 0
        for detector_id in algorithm_detector_ids:
            try:
                n_previous += traci.lanearea.getLastStepVehicleNumber(detector_id)
            except traci.TraCIException as e:
                print(f"[WARNING] Could not get initial data for algorithm detector {detector_id}: {e}")


        try:
            while sumo_sim.get_step_counts() < total_simulation_steps:
                sumo_sim.step()  # Advance the simulation by one step

                # Run controller only at the specified interval
                if sumo_sim.get_step_counts() % CONTROL_INTERVAL_S == 0:
                    
                    # 1. Get current total vehicle count for the algorithm
                    n_current = 0
                    for detector_id in algorithm_detector_ids:
                        try:
                            n_current += traci.lanearea.getLastStepVehicleNumber(detector_id)
                        except traci.TraCIException as e:
                            print(f"[WARNING] Could not get data for algorithm detector {detector_id}: {e}")

                    # 2. Get current queue lengths for the solver
                    live_queue_lengths = {}
                    for int_id, detectors in solver_detectors.items():
                        try:
                            # --- FIX: Handle single or multiple main queue detectors ---
                            main_queue = 0
                            main_detectors = detectors.get('main_queue_detector', [])
                            if not isinstance(main_detectors, list):
                                main_detectors = [main_detectors]  # Treat a single string as a list
                            
                            for detector_id in main_detectors:
                                main_queue += traci.lanearea.getLastStepVehicleNumber(detector_id)

                            # --- FIX: Handle single or multiple secondary queue detectors ---
                            sec_queue = 0
                            sec_detectors = detectors.get('secondary_queue_detector', [])
                            if not isinstance(sec_detectors, list):
                                sec_detectors = [sec_detectors] # Treat a single string as a list

                            for detector_id in sec_detectors:
                                sec_queue += traci.lanearea.getLastStepVehicleNumber(detector_id)
                            
                            live_queue_lengths[int_id] = {'main': main_queue, 'secondary': sec_queue}
                        except traci.TraCIException as e:
                            print(f"[WARNING] Could not get queue data for intersection {int_id}: {e}")
                            live_queue_lengths[int_id] = {'main': 0, 'secondary': 0} # Default on error
                    
                    # --- Run the Perimeter Control Step ---
                    print(f"--- Running Control Step at Simulation Time: {sumo_sim.get_step_counts()}s ---")
                    _, qg_previous, _ = controller.run_simulation_step(
                        n_current, n_previous, qg_previous, live_queue_lengths
                    )
                    n_previous = n_current
                
                # Print status every 10 seconds for monitoring
                if sumo_sim.get_step_counts() % 10 == 0:
                    # We can get a fresh vehicle count for printing, or just use the last calculated n_current
                    # For simplicity, let's just print a status message.
                    print(f"Step {sumo_sim.get_step_counts()}s / {total_simulation_steps}s")


        finally:
            print("Hoàn tất mô phỏng. Dừng luồng điều khiển...")
            stop_event.set() # Signal the controller thread to stop
            controller_thread.join() # Đợi luồng kết thúc
            try:
                sumo_sim.close()
                sql_conn.close()
            except Exception as e:
                print(f"Error closing connections: {e}")
            print("Simulation finished. Total steps:", sumo_sim.get_step_counts())


if __name__ == "__main__":
    # You can choose which simulation to run.
    # Default is the live SUMO simulation.
    
    # To run the mock simulation with test data:
    # python main.py mock
    
    if len(sys.argv) > 1 and sys.argv[1] == 'mock':
        run_mock_simulation()
    else:
        run_sumo_simulation()
