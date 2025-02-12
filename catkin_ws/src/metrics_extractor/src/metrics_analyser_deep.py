#!/usr/bin/env python3
import argparse
import csv
import matplotlib.pyplot as plt
import os

# Configure plot settings compatible with all matplotlib versions
plt.rcParams.update({
    'font.family': 'serif',
    'font.size': 12,
    'axes.titlesize': 14,
    'axes.labelsize': 12,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    'savefig.dpi': 300,
    'pdf.fonttype': 42,
    'axes.grid': True,
    'grid.alpha': 0.3
})

def save_individual_plots(time, metrics, events, output_dir):
    """Save each metric as a separate PDF file with event markers"""
    metric_info = [
        ('distance', 'Travelled Distance', 'blue'),
        ('velocity', 'Velocity', 'green'),
        ('instability', 'Instability Index', 'red'),
        ('torque', 'Mean Torque', 'purple'),
    ]

    for metric, title, color in metric_info:
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.plot(time, metrics[metric], color=color, linewidth=1.5)
        ax.set_title(title)
        ax.set_xlabel('Time (seconds)')
        
        # Add event markers
        for event in events:
            event_type, position, *rest = event
            if position < len(time):
                t = time[position]
                label = (f"Goal {rest[0]} Reached" if event_type == 'goal_reached' 
                        else "Mission Aborted")
                line_color = 'green' if event_type == 'goal_reached' else 'red'
                ax.axvline(x=t, color=line_color, linestyle='--', alpha=0.7)
                if metric == 'distance':
                    ax.text(t, ax.get_ylim()[1]*0.95, label,
                           color=line_color, ha='right', va='top', rotation=90)

        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, f"{metric}.pdf"), bbox_inches='tight')
        plt.close()

def main():
    parser = argparse.ArgumentParser(description='Generate publication-quality metrics plots')
    parser.add_argument('metrics_file', type=str, help='Input CSV file path')
    parser.add_argument('-o', '--output', type=str, default='plots', 
                       help='Output directory for PDF files')
    args = parser.parse_args()

    # Create output directory if needed
    os.makedirs(args.output, exist_ok=True)

    # Data storage
    metrics = {
        'distance': [], 'velocity': [], 'instability': [],
        'torque': []
    }
    events = []

    # Read and parse CSV
    with open(args.metrics_file, 'r') as f:
        reader = csv.reader(f)
        line_count = 0
        
        for row in reader:
            if len(row) == 4:
                try:
                    metrics['distance'].append(float(row[0]))
                    metrics['velocity'].append(float(row[1]))
                    metrics['instability'].append(float(row[2]))
                    metrics['torque'].append(float(row[3]))
                    line_count += 1
                except ValueError:
                    continue
            elif len(row) == 1:
                event_text = row[0].strip()
                if "REACHED GOAL" in event_text:
                    goal_number = int(event_text.split()[-1])
                    events.append(('goal_reached', line_count-1, goal_number))
                elif "ABORTED" in event_text:
                    events.append(('aborted', line_count-1))

    # Generate time axis (5Hz sampling)
    time = [i * 0.2 for i in range(len(metrics['distance']))]

    # Generate and save plots
    save_individual_plots(time, metrics, events, args.output)

    print(f"Saved PDF plots to {args.output} directory")


if __name__ == '__main__':
    main()
