import traci
import yaml
import threading
import time
from enum import Enum
from typing import Any, Callable, Dict, Optional
import sched
from datetime import datetime

from neo4j import GraphDatabase

from traci import trafficlight, lanearea, inductionloop
import libsumo

from sumosim import SumoSim

from data.collector.SqlCollector import SqlCollector
from algorithm.algo import MIQPSolver
from algorithm.algo import PerimeterController, KP_H, KI_H, N_HAT

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


if __name__ == "__main__":
    config = load_simulation_config(config_path = "./config/simulation.yml")
    application = load_application_config(config_path = "./config/application.yml")

    # Initialize database connection
    if application is None:
        raise ValueError("Failed to load application configuration.")
    else:
        sql_conn = SqlCollector(
            host= application['mysql']["host"],
            port= application['mysql']["port"],
            user= application['mysql']["user"],
            password= application['mysql']["password"], 
            database= application['mysql']["database"]
        )

    # Get lane area detector IDs from database
    detector_ids = sql_conn.get_lane_area_detector_ids()
    print(f"[INFO] Found {len(detector_ids)} lane area detectors")

    sumo_sim = SumoSim(config)
    sumo_sim.start()

    # Khởi tạo bộ điều khiển chu vi
    controller = PerimeterController(kp=KP_H, ki=KI_H, n_hat=N_HAT)
    n_current = 0
    n_previous = 0
    qg_previous = 3600

    simulation_duration = 90  # seconds
    simulation_start_time = time.time()
    first_loop = True

    while time.time() - simulation_start_time < simulation_duration:
        sumo_sim.step()  # Advance the simulation by one step

        # Lấy số lượng phương tiện tại các detector
        total_vehicles = 0
        for detector_id in detector_ids:
            try:
                vehicle_count = traci.lanearea.getLastStepVehicleNumber(detector_id)
                total_vehicles += vehicle_count
            except traci.TraCIException as e:
                print(f"[WARNING] Could not get data for detector {detector_id}: {e}")

        # Vòng lặp đầu tiên chỉ khởi tạo n_current, không điều khiển
        if first_loop:
            n_current = total_vehicles
            first_loop = False
        else:
            n_previous = n_current
            n_current = total_vehicles
            # --- Tích hợp điều khiển chu vi ---
            n_current, qg_previous, controller_active = controller.run_simulation_step(
                n_current, n_previous, qg_previous
            )
        print(f"Step {sumo_sim.get_step_counts()}: Total vehicles (detectors) = {total_vehicles}")

        time.sleep(0.05)

    try:
        sumo_sim.close()
        sql_conn.close()
    except Exception as e:
        print(f"Error closing connections: {e}")
    print("Simulation finished. Total steps:", sumo_sim.get_step_counts())