import numpy as np
import matplotlib.pyplot as plt
from typing import Callable, Tuple, List, Optional, Any, Dict
import threading
import os
import sys
import json
import time
import pandas as pd
import yaml
import traci
import traci.exceptions
from multiprocessing import Manager

# Add project root to system path
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sys.path.append(project_root)
print(project_root)

from src.sumosim import SumoSim
from src.algorithm.pso import PSO, PSO_MultiCPUs
from src.algorithm.algo import PerimeterController, N_HAT, CONTROL_INTERVAL_S
from src.data.intersection_config_manager import IntersectionConfigManager

# Check SUMO_HOME
if 'SUMO_HOME' not in os.environ:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

N_PARTICLES = 10
MAX_ITERATIONS = 12
KP_MIN = 0.0
KP_MAX = 100.0
KI_MIN = 0.0
KI_MAX = 20.0
TOTAL_SIM_STEPS = 1800

def load_config():
    """Load configuration files."""
    print("Loading configuration files...")
    
    # Load simulation config
    sim_config_path = os.path.join('src', 'config', 'simulation.yml')
    with open(sim_config_path, 'r') as f:
        sim_config = yaml.safe_load(f)['config']
    
    # Load detector config
    detector_config_path = os.path.join('src', 'config', 'detector_config.json')
    with open(detector_config_path, 'r') as f:
        detector_config = json.load(f)
    
    return sim_config, detector_config

def traffic_light_controller(shared_dict: Dict, config_file: str, stop_event: threading.Event):
    """
    Separate thread to control traffic lights based on data from shared_dict.
    """
    print("[CONTROLLER THREAD] Start the traffic light control thread.")
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

def run_sumo(sim_config, detector_config, params):
    """Collect data from e1 and e2 detectors."""
    intersection_config_path = os.path.join('src', 'config', 'detector_config.json')

    # Get detector IDs
    algorithm_detector_ids = detector_config['algorithm_input_detectors']['detector_ids']  # Vehicle count
    solver_detectors = detector_config['solver_input_detectors']['intersections']
    
    print(f"Found {len(algorithm_detector_ids)} e2 detectors for vehicle count and algorithm input")
    print(f"Found {len(solver_detectors)} intersections with detectors for solver input.")
    
    fitness_value = 0
    kp, ki = params[0], params[1]
    print(f"Starting SUMO simulation. Kp = {kp}, Ki = {ki}")

    with Manager() as manager:
        shared_dict = manager.dict()
        stop_event = threading.Event()

        # Initialize SUMO simulation
        sim_config['config_file'] = os.path.join('src', sim_config['config_file'])
        sim_config['gui'] = None
        sumo_sim = SumoSim(sim_config)
        sumo_sim.start()

        controller = PerimeterController(
            kp=kp, 
            ki=ki, 
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
        total_simulation_steps = TOTAL_SIM_STEPS # Total simulation time in seconds (e.g., 1 hour)
        
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

                    fitness_value += abs(n_current - N_HAT)

                    # 2. Get current queue lengths for the solver
                    live_queue_lengths = {}
                    for int_id, detectors in solver_detectors.items():
                        try:
                            main_queue = traci.lanearea.getLastStepVehicleNumber(detectors['main_queue_detector'])
                            sec_queue = 0
                            for detector_id in detectors['secondary_queue_detector']:
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

        except Exception as err:
            print(f"Stop simulation due to: {err}")

        finally:
            print("Hoàn tất mô phỏng. Dừng luồng điều khiển...")
            stop_event.set() # Signal the controller thread to stop
            controller_thread.join() # Đợi luồng kết thúc
            try:
                sumo_sim.close()
            except Exception as e:
                print(f"Error closing connections: {e}")
            print("Simulation finished. Total steps:", sumo_sim.get_step_counts())
    
    return fitness_value

def sumo_obj_func(params: np.ndarray):
    """Main function."""
    # Change to project root
    original_cwd = os.getcwd()
    os.chdir(project_root)
    
    try:
        # Load configuration
        sim_config, detector_config = load_config()
        
        # Collect data
        fitness_value = run_sumo(sim_config, detector_config, params)

        if fitness_value is None:
            raise ValueError("fitness_value is None - simulation may have failed")
    
        return fitness_value
                
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        raise
    finally:
        os.chdir(original_cwd)

if __name__ == "__main__":
    params_bound = ((KP_MIN, KP_MAX), (KI_MIN, KI_MAX)) 

    pso = PSO(
        objective_function=sumo_obj_func,
        bounds=params_bound,
        n_particles=N_PARTICLES,
        max_iterations=MAX_ITERATIONS,
        w=0.9,
        c1=2.0,
        c2=2.0,
        random_seed=42
    )

    # pso = PSO_MultiCPUs(
    #     objective_function=sumo_obj_func,
    #     bounds=params_bound,
    #     n_particles=N_PARTICLES,  # Good number for multiprocessing
    #     max_iterations=MAX_ITERATIONS,  # Just for demo
    #     use_multiprocessing=True,
    #     n_processes=5,  # Fixed number for predictable behavior
    #     random_seed=42
    # )

    best_params, best_fitness = pso.optimize(verbose=True)
    print(f"\nOptimization Results:")
    print(f"Best Kp: {best_params[0]:.4f}")
    print(f"Best Ki: {best_params[1]:.4f}")
    print(f"Best Fitness: {best_fitness:.6f}")
    pso.plot_convergence()
    pso.plot_particles()