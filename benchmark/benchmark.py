#!/bin/python3

import json
import subprocess
import re
import pandas as pd
import os
import time

# --- Configuration ---
EXECUTABLE = "../raytracer.out" # Ensure this matches your makefile output
CONFIG_FILE = "../config.json"
RESULTS_FILE = "benchmark_results.csv"
ITERATIONS = 5

# Test Scenes
SCENES = [
    "CornellBox-Bunny.obj",
    "CornellBox-Dragon.obj",
    "CornellBox-Hairball.obj",
    "CornellBox-Powerplant.obj",
]

# Map BVH types to the specific regex needed to capture build time from stdout
BVH_TYPES = {
    "bvh_seq2": r"Sequential2 BVH building time:\s+([\d\.]+)\s+ms",
    "bvh_vec": r"Vectorized BVH building time:\s+([\d\.]+)\s+ms",
    "bvh_vec2": r"Vectorized2 BVH building time:\s+([\d\.]+)\s+ms",
    "bvh_par2": r"Parallel2 BVH building time:\s+([\d\.]+)\s+ms",
    "bvh_par2v": r"Parallel2v BVH building time:\s+([\d\.]+)\s+ms",
}

# Thread counts to test (for Parallel implementations)
THREAD_COUNTS = [8, 16, 32, 64, 128, 192, 256]
# Horizontal thresholds (for Parallel implementations)
THRESHOLDS = [1000, 5000, 10000, 20000, 50000, 75000, 100000] 

def run_benchmark():
    results = []

    # Load base config
    with open("default_config.json", "r") as f:
        base_config = json.load(f)

    for scene in SCENES:
        base_config["scene"]["source_file"] = scene
        scene_name = os.path.basename(scene)
        
        for bvh_name, regex_pattern in BVH_TYPES.items():
            base_config["renderer"]["acceleration_data_structure"]["name"] = bvh_name
            
            # Determine parameter sets based on BVH type
            if "par" in bvh_name:
                param_sets = [(t, h) for t in THREAD_COUNTS for h in THRESHOLDS]
            else:
                # Sequential/Vec don't use threads/thresholds
                param_sets = [(1, 0)] 

            for threads, threshold in param_sets:
                print(f"Benchmarking {scene_name} | {bvh_name} | Threads: {threads} | Thresh: {threshold}")
                
                # Update Config
                base_config["renderer"]["acceleration_data_structure"]["horizontal_threshold"] = threshold
                with open(CONFIG_FILE, "w") as f:
                    json.dump(base_config, f, indent=4)

                # Run Iterations
                times = []
                for i in range(ITERATIONS):
                    # Set OMP_NUM_THREADS environment variable
                    env = os.environ.copy()
                    env["OMP_NUM_THREADS"] = str(threads)

                    try:
                        # Run command
                        result = subprocess.run(
                            [EXECUTABLE, CONFIG_FILE], 
                            capture_output=True, 
                            text=True, 
                            env=env
                        )

                        # print(EXECUTABLE, CONFIG_FILE, result.stdout)
                        
                        # Extract time using Regex
                        match = re.search(regex_pattern, result.stdout)
                        if match:
                            times.append(float(match.group(1)))
                        else:
                            print(f"  Warning: Could not parse time on iter {i}")
                    except Exception as e:
                        print(f"  Error: {e}")

                if times:
                    avg_time = sum(times) / len(times)
                    results.append({
                        "Scene": scene_name,
                        "BVH": bvh_name,
                        "Threads": threads,
                        "Threshold": threshold,
                        "AvgBuildTime_ms": avg_time,
                        "RawTimes": times
                    })

    # Save to CSV
    df = pd.DataFrame(results)
    df.to_csv(RESULTS_FILE, index=False)
    print(f"Benchmark complete. Results saved to {RESULTS_FILE}")

if __name__ == "__main__":
    run_benchmark()