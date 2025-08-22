# Step 1: Add modules to provide access to specific libraries and functions
import os  # Module provides functions to handle file paths, directories, environment variables
import sys  # Module provides access to Python-specific system parameters and functions
import subprocess
import logging

# Step 2: Establish path to SUMO (SUMO_HOME)
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

# Step 3: Import SUMO libraries
import numpy as np
import traci
import sumolib

class SumoSim:
    def __init__(self, config: dict):
        self.config = config
        self.sumo_binary = sumolib.checkBinary('sumo-gui' if self.config['gui'] else 'sumo')
        self.step_count = 0

    #start sumo
    #open connection with traci
    #predefined config
    def start(self, output_files: dict = None):
        """Start the SUMO simulation, optionally overriding output files."""
        project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        config_file_path = os.path.join(project_root, 'src', self.config['config_file'])

        sumo_cmd = [self.sumo_binary, "-c", config_file_path,
                    "--step-length", str(self.config['step_length'])]

        # Add output file overrides if provided
        if output_files:
            if 'tripinfo' in output_files:
                sumo_cmd.extend(["--tripinfo-output", output_files['tripinfo']])
            if 'vehroute' in output_files:
                sumo_cmd.extend(["--vehroute-output", output_files['vehroute']])

        if self.config['gui']:
            sumo_cmd.append("--start")

        try:
            traci.start(sumo_cmd, port=self.config['port'])
            logging.info("SUMO simulation started with command: %s", ' '.join(sumo_cmd))
        except Exception as e:
            print(f"Error starting SUMO: {e}")
            sys.exit(1)

    def step(self):
        """Perform a simulation step."""
        traci.simulationStep()
        self.step_count += 1
    
    def close(self):
        traci.close()
    
    def get_step_counts(self)-> int:
        return self.step_count



