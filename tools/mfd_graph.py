#!/usr/bin/env python3
"""
MFD Graph Tool
==============
A simple tool to collect data from e1 (flow) and e2 (vehicle count) detectors
and create a Macroscopic Fundamental Diagram (MFD) graph.

Usage:
    python mfd_graph.py
"""

import os
import sys
import json
import time
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import yaml
import traci
import traci.exceptions

# Add project root to system path
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sys.path.append(project_root)

from src.sumosim import SumoSim

# Check SUMO_HOME
if 'SUMO_HOME' not in os.environ:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

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

def collect_data(sim_config, detector_config):
    """Collect data from e1 and e2 detectors."""
    print("Starting SUMO simulation...")
    
    # Get detector IDs
    e2_detectors = detector_config['algorithm_input_detectors']['detector_ids']  # Vehicle count
    e1_detectors = detector_config['mfd_input_flow_detectors']['detector_ids']   # Flow
    
    print(f"Found {len(e2_detectors)} e2 detectors for vehicle count")
    print(f"Found {len(e1_detectors)} e1 detectors for flow")
    
    # Initialize SUMO simulation
    sim_config['config_file'] = os.path.join('src', sim_config['config_file'])
    sumo_sim = SumoSim(sim_config)
    
    data_points = []
    
    try:
        sumo_sim.start()
        
        # Simulation parameters
        simulation_time = 3000  # 5 minutes
        step_length = float(sim_config.get('step_length', 1.0))
        total_steps = int(simulation_time / step_length)
        
        print(f"Running simulation for {simulation_time} seconds...")
        
        for step in range(total_steps):
            sumo_sim.step()
            
            # Collect vehicle count from e2 detectors
            vehicle_count = 0
            for det_id in e2_detectors:
                try:
                    vehicle_count += traci.lanearea.getLastStepVehicleNumber(det_id)
                except traci.exceptions.TraCIException:
                    pass  # Skip if detector not found
            
            # Collect flow from e1 detectors
            flow_count = 0
            for det_id in e1_detectors:
                try:
                    flow_count += traci.inductionloop.getLastIntervalVehicleNumber(det_id)
                except traci.exceptions.TraCIException:
                    pass  # Skip if detector not found
            
            # Convert flow to vehicles per hour
            flow_per_hour = flow_count * (3600 / step_length)
            
            # Store data point
            data_points.append({
                'vehicle_count': vehicle_count,
                'flow_per_hour': flow_per_hour,
                'time': step * step_length
            })
            
            # Print progress every 30 seconds
            if (step + 1) % 30 == 0:
                print(f"  Step {step + 1}/{total_steps}: "
                      f"Vehicles={vehicle_count}, Flow={flow_per_hour:.1f} veh/h")
        
        print("Simulation completed successfully!")
        
    except Exception as e:
        print(f"Error during simulation: {e}")
        return None
    finally:
        if traci.isLoaded():
            sumo_sim.close()
    
    return data_points

def create_mfd_graph(data_points):
    """Create MFD graph from collected data."""
    if not data_points:
        print("No data collected. Cannot create graph.")
        return
    
    print("Creating MFD graph...")
    
    # Convert to DataFrame
    df = pd.DataFrame(data_points)
    
    # Remove zero values and outliers
    df = df[(df['vehicle_count'] > 0) & (df['flow_per_hour'] > 0)]
    
    if len(df) == 0:
        print("No valid data points after filtering.")
        return
    
    # Create the graph
    plt.figure(figsize=(12, 8))
    
    # Scatter plot
    plt.scatter(df['vehicle_count'], df['flow_per_hour'], 
               alpha=0.6, s=30, color='blue', label='Data Points')
    
    # Add trend line
    if len(df) > 1:
        z = np.polyfit(df['vehicle_count'], df['flow_per_hour'], 2)
        p = np.poly1d(z)
        x_trend = np.linspace(df['vehicle_count'].min(), df['vehicle_count'].max(), 100)
        plt.plot(x_trend, p(x_trend), 'r-', linewidth=2, label='Trend Line')
    
    # Customize graph
    plt.title('Macroscopic Fundamental Diagram (MFD)', fontsize=16, fontweight='bold')
    plt.xlabel('Number of Vehicles (from e2 detectors)', fontsize=12)
    plt.ylabel('Flow Rate (vehicles/hour from e1 detectors)', fontsize=12)
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    # Add statistics
    stats_text = f"""
    Data Points: {len(df)}
    Vehicle Range: {df['vehicle_count'].min():.0f} - {df['vehicle_count'].max():.0f}
    Flow Range: {df['flow_per_hour'].min():.1f} - {df['flow_per_hour'].max():.1f} veh/h
    Max Flow: {df['flow_per_hour'].max():.1f} veh/h
    """
    plt.text(0.02, 0.98, stats_text, transform=plt.gca().transAxes,
             verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    # Save and show
    output_dir = os.path.join(project_root, "output")
    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(output_dir, "mfd_graph.png")
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"Graph saved to: {output_path}")
    
    # Show the graph
    plt.show()

def main():
    """Main function."""
    print("=" * 50)
    print("MFD Graph Tool")
    print("=" * 50)
    
    # Change to project root
    original_cwd = os.getcwd()
    os.chdir(project_root)
    
    try:
        # Load configuration
        sim_config, detector_config = load_config()
        
        # Collect data
        data_points = collect_data(sim_config, detector_config)
        
        # Create graph
        if data_points:
            create_mfd_graph(data_points)
            print("\nMFD analysis completed successfully!")
        else:
            print("Failed to collect data.")
            
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        os.chdir(original_cwd)

if __name__ == "__main__":
    main()
