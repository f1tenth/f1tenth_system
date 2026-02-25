import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import glob
import os

def analyze_latency(folder_path, threshold=100, stop_at=600):
    output_folder = 'analysis_graphs'
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    search_path = os.path.join(folder_path, "*.csv")
    files = glob.glob(search_path)

    if not files:
        print(f"No CSV files found in: {os.path.abspath(folder_path)}")
        return

    for file_path in files:
        file_name = os.path.basename(file_path)

        df = pd.read_csv(file_path)
        df = df.iloc[:stop_at]
        
        raw_latency = df['Latency_ms'] 
        sorted_latency = raw_latency.sort_values()

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 9))

        # --- 1. LATENCY TIMELINE PLOT ---
        ax1.plot(df.index, raw_latency, label='Control Latency', color='blue', alpha=0.7)
        ax1.axhline(y=raw_latency.mean(), color='green', linestyle='--', label='Mean')
        
        is_spike = raw_latency > threshold
        diff = np.diff(is_spike.astype(int), prepend=0, append=0)
        starts = np.where(diff == 1)[0]
        ends = np.where(diff == -1)[0]
        for s, e in zip(starts, ends):
            ax1.axvspan(s, e, color='red', alpha=0.2)

        ax1.set_title(f'Timeline: {file_name}') 
        ax1.set_xlabel('Packet Sequence')
        ax1.set_ylabel('Latency (ms)')
        ax1.legend()

        # --- 2. CDF PLOT ---
        cdf_y = np.arange(1, len(sorted_latency) + 1) / len(sorted_latency)
        ax2.plot(sorted_latency, cdf_y, color='darkorange', lw=2)
        p95 = np.percentile(raw_latency, 95)
        ax2.axvline(x=p95, color='red', linestyle=':', label=f'95th Pct: {p95:.1f}ms')
        ax2.set_title(f'CDF: {file_name}')
        ax2.set_xlabel('Latency (ms)')
        ax2.legend()

        # --- 3. STAT ANALYSIS TABLE ---
        table_data = [
            ["Mean", f"{raw_latency.mean():.2f} ms"],
            ["Median", f"{raw_latency.median():.2f} ms"],
            ["Std Dev", f"{raw_latency.std():.2f} ms"],
            ["Min", f"{raw_latency.min():.2f} ms"],
            ["Max", f"{raw_latency.max():.2f} ms"]
        ]

        the_table = plt.table(cellText=table_data,
                              colLabels=["Metric", "Result"],
                              loc='bottom',
                              cellLoc='left',
                              bbox=[0.35, -0.5, 0.3, 0.3]) # Center the table [left, bottom, width, height]

        the_table.set_fontsize(12)

        #save output reports to "analysis_graphs" folder
        plt.tight_layout(rect=[0, 0, 1, 1]) 
        output_name = file_name.replace('.csv', '_analysis.png')
        save_path = os.path.join(output_folder, output_name)
        plt.savefig(save_path, bbox_inches='tight')
        plt.close() 

        print(f"Saved report: {output_name}")

analyze_latency('./data')