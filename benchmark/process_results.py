#!/usr/bin/env python3

# Generated using Gemini 3

import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np

RESULTS_FILE = "benchmark_results.csv"
OUTPUT_DIR = "."

def clean_scene_name(name):
    return name.replace("CornellBox-", "").replace(".obj", "")

def generate_par_comparison_table(df):
    print("Generating Parallel Comparison Table...")
    # Filter for Par2 and Par2V
    df_par = df[df["BVH"].isin(["bvh_par2", "bvh_par2v"])].copy()
    
    if df_par.empty:
        print("No parallel data found.")
        return

    # Find best time for each Scene, BVH, Threads combination (min across Thresholds)
    # We need to keep the threshold that gave the min time.
    # idx = df_par.groupby(["Scene", "BVH", "Threads"])["AvgBuildTime_ms"].idxmin()
    # df_best = df_par.loc[idx]

    # Find the single best threshold for each Scene/BVH pair (global min across all threads)
    best_thresh_idx = df_par.groupby(["Scene", "BVH"])["AvgBuildTime_ms"].idxmin()
    best_thresh_df = df_par.loc[best_thresh_idx, ["Scene", "BVH", "Threshold"]]
    
    # Filter the original data to only include rows with these best thresholds
    df_best = pd.merge(df_par, best_thresh_df, on=["Scene", "BVH", "Threshold"])
    
    # Pivot: Index=[Scene, BVH], Columns=Threads
    pivot_time = df_best.pivot_table(index=["Scene", "BVH"], columns="Threads", values="AvgBuildTime_ms", aggfunc='first')
    pivot_thresh = df_best.pivot_table(index=["Scene", "BVH"], columns="Threads", values="Threshold", aggfunc='first')
    
    # Reorder rows based on scene order
    scene_order = ["CornellBox-Bunny.obj", "CornellBox-Dragon.obj", "CornellBox-Hairball.obj", "CornellBox-Powerplant.obj"]
    
    # Construct LaTeX Table
    threads = sorted(pivot_time.columns)
    latex_str = "\\begin{table*}[htbp]\n\\centering\n\\begin{tabular}{|c|c|" + "c|"*len(threads) + "}\n\\hline\n"
    
    # Header
    latex_str += "\\thead{Scene} & \\thead{BVH} & " + " & ".join([f"\\thead{{{t}}}" for t in threads]) + " \\\\\n\\hline\n"
    
    for scene in scene_order:
        if scene not in pivot_time.index.levels[0]:
            continue
        
        clean_scene = clean_scene_name(scene)
        
        def format_cell(row_time, row_thresh, t):
            if t in row_time.index and not pd.isna(row_time[t]):
                return f"{row_time[t]:.2f} ({int(row_thresh[t])})"
            return "-"
        
        # Par2
        if (scene, "bvh_par2") in pivot_time.index:
            row_vals = pivot_time.loc[(scene, "bvh_par2")]
            row_thresh = pivot_thresh.loc[(scene, "bvh_par2")]
            vals_str = " & ".join([format_cell(row_vals, row_thresh, t) for t in threads])
            latex_str += f"{clean_scene} & Par2 & {vals_str} \\\\\n"
        
        # Par2V
        if (scene, "bvh_par2v") in pivot_time.index:
            row_vals = pivot_time.loc[(scene, "bvh_par2v")]
            row_thresh = pivot_thresh.loc[(scene, "bvh_par2v")]
            vals_str = " & ".join([format_cell(row_vals, row_thresh, t) for t in threads])
            latex_str += f"{clean_scene} & Par2V & {vals_str} \\\\\n"
            
        latex_str += "\\hline\n"

    all_thresholds = sorted(df_par["Threshold"].unique().astype(int))
    thresholds_str = ", ".join(map(str, all_thresholds))

    latex_str += f"\\end{{tabular}}\n\\caption{{Comparison of BvhPar2 and BvhPar2V build times (ms) across thread counts. The threshold used is the one that achieved the global minimum build time for that Scene and BVH type. Values are formatted as: Time (Threshold). Tested thresholds: \\{{{thresholds_str}\\}}.}}\n\\label{{tab:par_comparison}}\n\\end{{table*}}"
    
    output_path = os.path.join(OUTPUT_DIR, "table_par_comparison.tex")
    with open(output_path, "w") as f:
        f.write(latex_str)
    print(f"Saved {output_path}")

def generate_summary_table_and_graph(df):
    print("Generating Summary Table and Graph...")
    scene_order = ["CornellBox-Bunny.obj", "CornellBox-Dragon.obj", "CornellBox-Hairball.obj", "CornellBox-Powerplant.obj"]
    
    summary_data = []
    
    # Prepare data for graph (keep existing logic for graph)
    graph_data = []

    # Prepare LaTeX string
    latex_str = "\\begin{table}[h]\n\\centering\n\\begin{tabular}{|c|c|c|}\n\\hline\n"
    latex_str += "\\textbf{Scene} & \\textbf{BVH types} & \\textbf{Total time [ms]} \\\\\n\\hline\n"

    for scene in scene_order:
        if scene not in df["Scene"].values:
            continue
            
        scene_df = df[df["Scene"] == scene]
        clean_scene = clean_scene_name(scene)
        
        def get_min_time(bvh_name):
            vals = scene_df[scene_df["BVH"] == bvh_name]["AvgBuildTime_ms"]
            return vals.min() if not vals.empty else float('nan')
        
        seq2 = get_min_time("bvh_seq2")
        vec = get_min_time("bvh_vec")
        vec2 = get_min_time("bvh_vec2")
        par2 = get_min_time("bvh_par2")
        par2v = get_min_time("bvh_par2v")
        
        # Add to graph data
        graph_data.append({
            "Scene": clean_scene,
            "BvhSeq2": seq2,
            "BvhVec": vec,
            "BvhVec2": vec2,
            "Best BvhPar2": par2,
            "Best BvhPar2V": par2v
        })
        
        # Add to LaTeX
        latex_str += f"{clean_scene} & BvhSeq2 & {seq2:.2f} \\\\\n"
        latex_str += f"{clean_scene} & BvhVec & {vec:.2f} \\\\\n"
        latex_str += f"{clean_scene} & BvhVec2 & {vec2:.2f} \\\\\n"
        latex_str += f"{clean_scene} & BvhPar2 (Best) & {par2:.2f} \\\\\n"
        latex_str += f"{clean_scene} & BvhPar2V (Best) & {par2v:.2f} \\\\\n"
        latex_str += "\\hline\n"
        
    latex_str += "\\end{tabular}\n\\caption{Build times for all BVH implementations. Parallel versions show the best time across all configurations.}\n\\label{tab:summary}\n\\end{table}"
    
    output_path_tex = os.path.join(OUTPUT_DIR, "table_summary.tex")
    with open(output_path_tex, "w") as f:
        f.write(latex_str)
    print(f"Saved {output_path_tex}")
    
    summary_df = pd.DataFrame(graph_data)
    
    # Plotting Linear
    ax = summary_df.plot(kind="bar", width=0.8, figsize=(12, 6))
    
    plt.title("BVH Build Time Comparison (Linear Scale)")
    plt.ylabel("Time (ms)")
    plt.xlabel("Scene")
    plt.xticks(rotation=0)
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    plt.legend(title="BVH Type")
    
    plt.tight_layout()
    output_path_linear = os.path.join(OUTPUT_DIR, "comparison_graph_linear.png")
    plt.savefig(output_path_linear, dpi=100)
    print(f"Saved {output_path_linear}")
    plt.close()

    # Plotting Logarithmic
    ax = summary_df.plot(kind="bar", width=0.8, figsize=(12, 6), logy=True)
    
    plt.title("BVH Build Time Comparison (Logarithmic Scale)")
    plt.ylabel("Time (ms) - Log Scale")
    plt.xlabel("Scene")
    plt.xticks(rotation=0)
    plt.grid(axis='y', linestyle='--', alpha=0.7, which="both")
    plt.legend(title="BVH Type")
    
    plt.tight_layout()
    output_path_log = os.path.join(OUTPUT_DIR, "comparison_graph_log.png")
    plt.savefig(output_path_log, dpi=100)
    print(f"Saved {output_path_log}")
    plt.close()

def process_results():
    if not os.path.exists(RESULTS_FILE):
        print(f"Error: {RESULTS_FILE} not found.")
        return

    print(f"Reading {RESULTS_FILE}...")
    df = pd.read_csv(RESULTS_FILE)
    
    generate_par_comparison_table(df)
    generate_summary_table_and_graph(df)

if __name__ == "__main__":
    process_results()
