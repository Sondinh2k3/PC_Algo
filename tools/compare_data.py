import xml.etree.ElementTree as ET
import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np

def parse_detector_xml(file_path):
    """Parses a custom detector XML file and returns a DataFrame."""
    try:
        tree = ET.parse(file_path)
        root = tree.getroot()
        
        data = []
        for detector in root.findall('detector'):
            row = {}
            for child in detector:
                row[child.tag] = child.text
            data.append(row)
                
        df = pd.DataFrame(data)
        
        if df.empty:
            print(f"Warning: No <detector> data found in {file_path}")
            return df

        # Convert relevant columns to numeric
        numeric_cols = ['begin', 'end', 'flow', 'occupancy', 'speed']
        for col in numeric_cols:
            if col in df.columns:
                df[col] = pd.to_numeric(df[col], errors='coerce')
        return df
    except ET.ParseError:
        print(f"Error parsing XML file: {file_path}. It might be empty or malformed.")
        return pd.DataFrame()
    except FileNotFoundError:
        print(f"Error: File not found at {file_path}")
        return pd.DataFrame()

def plot_comparison(df_algo, df_baseline, param, title, ylabel, output_dir, time_col='begin'):
    """Generates a clear, simple comparative line plot for a given parameter."""
    if df_algo.empty or df_baseline.empty:
        print(f"Skipping '{param}' plot due to empty dataframe.")
        return

    plt.style.use('default')
    plt.figure(figsize=(12, 6))
    
    avg_algo = df_algo[param].mean()
    avg_baseline = df_baseline[param].mean()
    
    label_algo = f'Algorithm (Avg: {avg_algo:.2f})'
    label_baseline = f'Baseline (Avg: {avg_baseline:.2f})'

    if param == 'flow':
        # For flow, the sum over the whole period is also a key metric.
        total_algo = df_algo[param].sum()
        total_baseline = df_baseline[param].sum()
        label_algo = f'Algorithm (Avg Interval Flow: {avg_algo:.2f}, Total Veh: {total_algo:,.0f})'
        label_baseline = f'Baseline (Avg Interval Flow: {avg_baseline:.2f}, Total Veh: {total_baseline:,.0f})'

    plt.plot(df_algo[time_col], df_algo[param], label=label_algo, color='blue', linewidth=1.5, marker='o', markersize=4)
    plt.plot(df_baseline[time_col], df_baseline[param], label=label_baseline, color='red', linestyle='--', linewidth=1.5, marker='x', markersize=4)

    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel(ylabel, fontsize=12)
    plt.title(title, fontsize=14, fontweight='bold')
    plt.legend(loc='best', fontsize=10)
    plt.grid(True, which='both', linestyle=':', linewidth=0.6)
    plt.xlim(left=0)
    
    plt.tight_layout()
    output_path = os.path.join(output_dir, f'{param}_comparison_50s_interval.png')
    plt.savefig(output_path)
    print(f"Saved plot to {output_path}")
    plt.close()

if __name__ == "__main__":
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    output_dir = os.path.join(project_root, 'output')
    
    algo_file = os.path.join(output_dir, 'data_algo.xml')
    baseline_file = os.path.join(output_dir, 'data_baseline.xml')

    print(f"Parsing Algorithm data from: {algo_file}")
    df_algo = parse_detector_xml(algo_file)
    
    print(f"Parsing Baseline data from: {baseline_file}")
    df_baseline = parse_detector_xml(baseline_file)

    if df_algo.empty or df_baseline.empty:
        print("\nCould not proceed with plotting due to missing or invalid data.")
        exit()

    # --- Aggregate Data every 50s ---
    sampling_interval = 50  # seconds

    # Create a new column for the time interval
    df_algo['time_interval'] = (df_algo['begin'] // sampling_interval) * sampling_interval
    df_baseline['time_interval'] = (df_baseline['begin'] // sampling_interval) * sampling_interval
    
    # Replace -1 speed (indicates no cars) with NaN to be ignored in mean calculation
    df_algo['speed'] = df_algo['speed'].replace(-1, np.nan)
    df_baseline['speed'] = df_baseline['speed'].replace(-1, np.nan)

    # Group by the new time interval. Sum flow, average the others.
    agg_funcs = {
        'flow': 'sum',
        'speed': 'mean',
        'occupancy': 'mean'
    }

    df_algo_agg = df_algo.groupby('time_interval').agg(agg_funcs).reset_index()
    df_baseline_agg = df_baseline.groupby('time_interval').agg(agg_funcs).reset_index()

    # --- Generate Plots ---
    print("\nGenerating comparison plots (50s sampling)...")
    
    plot_params = {
        'flow': 'Total Flow (vehicles/50s)',
        'speed': 'Average Speed (m/s)',
        'occupancy': 'Average Occupancy (%)'
    }
    
    for param, ylabel in plot_params.items():
        if param in df_algo_agg.columns and param in df_baseline_agg.columns:
            title = f'{param.replace("_", " ").title()} Comparison (50s Interval)'
            plot_comparison(df_algo_agg, df_baseline_agg, param, title, ylabel, output_dir, time_col='time_interval')
        else:
            print(f"Skipping '{param}' plot: column not found in one or both dataframes.")
    
    print(f"\nComparison plots saved to {output_dir}")
