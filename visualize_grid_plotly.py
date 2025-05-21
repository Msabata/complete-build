#!/usr/bin/env python3

import json
import argparse
import sys
import os
import numpy as np
import plotly.graph_objects as go # Main Plotly library
from plotly.colors import qualitative # For potential distinct colors if needed

# --- Color definitions (adjust hex codes for desired look) ---
# Using slightly different shades for potentially better visual separation
COLORS = {
    'IMPASSABLE': '#5A5A5A',        # Darker Grey
    'ROAD': '#B0B0B0',              # Lighter Grey
    'WATER_MARSH': '#6495ED',       # CornflowerBlue
    'UNDERGROWTH': '#8FBC8F',       # DarkSeaGreen
    'DEFAULT_TERRAIN': '#F5F5F5',   # WhiteSmoke (slightly off-white)
    'COST_HIGH': '#DC143C',         # Crimson (Strong Red)
    'COST_MEDIUM_HIGH': '#228B22',  # ForestGreen
    'COST_MEDIUM_LOW': '#1E90FF',   # DodgerBlue
    'COST_LOW_4': '#9ACD32',        # YellowGreen
    'COST_LOW_3': '#FFD700',        # Gold
    'COST_LOW_2': '#FFA500',        # Orange
    'COST_LOW_1': '#FFE4B5',        # Moccasin (Light Orange/Yellow)
    'COST_LOW_0': '#FF6347',        # Tomato (Orange-Red)
    'PATH': '#FF00FF',              # Magenta (Fuchsia) - Keep bright for visibility
    'START': '#FF0000',             # Red - Standard Start
    'END': '#0000FF',               # Blue - Standard End
    # No grid line color needed - heatmap handles cell separation
}

# --- Helper functions (Same as before) ---
def index_to_xy(index, width):
    if index < 0 or width <= 0: return None, None
    # Assume index corresponds to row-major order, starting top-left
    y = index // width
    x = index % width
    return x, y

def has_flag(cell_flags, flag_value):
    return (int(cell_flags) & int(flag_value)) != 0

# --- Main Visualization Function ---
def visualize_grid_with_plotly(base_filepath, output_html=None):
    """Loads grid data and visualizes it using Plotly."""

    meta_filepath = base_filepath + "_meta.json"
    data_filepath = base_filepath + "_data.bin"

    if not os.path.exists(meta_filepath):
        print(f"Error: Metadata file not found: {meta_filepath}", file=sys.stderr)
        sys.exit(1)
    if not os.path.exists(data_filepath):
        print(f"Error: Binary data file not found: {data_filepath}", file=sys.stderr)
        sys.exit(1)

    # 1. Load Metadata (same as before)
    try:
        with open(meta_filepath, 'r') as f:
            metadata_json = json.load(f)
            metadata = metadata_json.get("metadata", {})
            flag_defs = metadata_json.get("flag_definitions", {})
            path_indices = metadata_json.get("path_indices", [])
    except Exception as e:
         print(f"Error reading metadata file {meta_filepath}: {e}", file=sys.stderr)
         sys.exit(1)

    width = metadata.get("grid_width")
    height = metadata.get("grid_height")
    start_pt_data = metadata.get("start_point")
    end_pt_data = metadata.get("end_point")
    binary_format_desc = metadata.get("binary_format", "unknown") # Optional info

    if not width or not height or not flag_defs:
        print("Error: Essential metadata (width, height, flags) missing.", file=sys.stderr)
        sys.exit(1)

    print(f"Grid: {width}x{height}. Loading data...")

    # 2. Load Binary Data (same as before)
    try:
        dt = np.dtype([('value', np.float32), ('flags', np.int32)])
        expected_total_bytes = width * height * dt.itemsize
        actual_file_size = os.path.getsize(data_filepath)

        if actual_file_size != expected_total_bytes:
            print(f"Warning: Binary file size mismatch! Expected {expected_total_bytes}, found {actual_file_size}.", file=sys.stderr)
            # Attempting to load anyway...

        grid_data_flat = np.fromfile(data_filepath, dtype=dt, count=width * height)
        if grid_data_flat.size != width * height:
             print(f"Error: Read {grid_data_flat.size} items, expected {width * height}.", file=sys.stderr)
             sys.exit(1)

        grid_data_structured = grid_data_flat.reshape((height, width))
        grid_values = grid_data_structured['value']
        grid_flags = grid_data_structured['flags']
    except Exception as e:
        print(f"Error reading or processing binary file {data_filepath}: {e}", file=sys.stderr)
        sys.exit(1)

    print("Data loaded. Processing categories for visualization...")

    # 3. Prepare Data for Plotly Heatmap
    # We need a grid of *category labels* or *IDs* and a way to map these to colors.
    # Let's create a grid of category names and a mapping from name to color.

    category_grid = np.full((height, width), "Unknown", dtype=object)
    category_colors = {} # Map category name -> color hex

    # Get flag integer values from loaded definitions
    flag_impassable = flag_defs.get("FLAG_IMPASSABLE")
    flag_road = flag_defs.get("FLAG_ROAD_PATH")
    flag_water = flag_defs.get("FLAG_WATER_MARSH")
    flag_undergrowth = flag_defs.get("FLAG_UNDERGROWTH")
    # Add others as needed...

    # Assign categories and colors (similar logic to Matplotlib version)
    for y in range(height):
        for x in range(width):
            c_flags = grid_flags[y, x]
            c_value = grid_values[y, x]
            category = "Terrain (Cost=1.0)" # Default
            color = COLORS['DEFAULT_TERRAIN']

            if flag_impassable is not None and has_flag(c_flags, flag_impassable):
                category, color = "Impassable", COLORS['IMPASSABLE']
            elif flag_road is not None and has_flag(c_flags, flag_road):
                category, color = "Road", COLORS['ROAD']
            elif flag_water is not None and has_flag(c_flags, flag_water):
                category, color = "Water/Marsh", COLORS['WATER_MARSH']
            elif flag_undergrowth is not None and has_flag(c_flags, flag_undergrowth):
                category, color = "Undergrowth", COLORS['UNDERGROWTH']
            elif abs(c_value - 1.0) > 1e-6:
                 if c_value < 0: category, color = "Impassable (Value < 0)", COLORS['IMPASSABLE']
                 elif c_value >= 10.0: category, color = "Terrain (Cost>=10.0)", COLORS['COST_HIGH']
                 elif c_value >= 5.0: category, color = "Terrain (5.0<=Cost<10.0)", COLORS['COST_MEDIUM_HIGH']
                 elif c_value >= 2.0: category, color = "Terrain (2.0<=Cost<5.0)", COLORS['COST_MEDIUM_LOW']
                 elif c_value >= 1.67: category, color = "Terrain (1.67<=Cost<2.0)", COLORS['COST_LOW_4']
                 elif c_value >= 1.43: category, color = "Terrain (1.43<=Cost<1.67)", COLORS['COST_LOW_3']
                 elif c_value >= 1.25: category, color = "Terrain (1.25<=Cost<1.43)", COLORS['COST_LOW_2']
                 elif c_value >= 1.11: category, color = "Terrain (1.11<=Cost<1.25)", COLORS['COST_LOW_1']
                 elif c_value > 1.0: category, color = "Terrain (1.0<Cost<1.11)", COLORS['COST_LOW_0']

            category_grid[y, x] = category
            if category not in category_colors:
                category_colors[category] = color

    # Map category names to integer IDs for Plotly's discrete colorscale
    unique_categories = sorted(list(category_colors.keys()))
    category_to_id = {name: i for i, name in enumerate(unique_categories)}
    grid_category_ids = np.vectorize(category_to_id.get)(category_grid)

    # Create the discrete colorscale for Plotly
    # Format: [[norm_value_0, color_0], [norm_value_1, color_1], ...]
    # For N categories, norm_values are 0, 1/N, 2/N, ..., (N-1)/N, 1.0
    n_categories = len(unique_categories)
    plotly_colorscale = []
    for i, category_name in enumerate(unique_categories):
        norm_start = i / n_categories
        norm_end = (i + 1) / n_categories
        color = category_colors[category_name]
        plotly_colorscale.append([norm_start, color])
        plotly_colorscale.append([norm_end, color]) # Color covers the interval


    print("Categories processed. Creating Plotly figure...")

    # 4. Create Plotly Figure

    # Create the main heatmap trace
    # Note: Plotly heatmap 'y' axis typically goes bottom-up.
    # If your grid_values/flags assume y=0 is the TOP row, you might need to flip
    # the z data vertically: grid_category_ids[::-1]
    # Let's try flipping z.
    heatmap_trace = go.Heatmap(
        z=grid_category_ids[::-1], # Flip data vertically
        colorscale=plotly_colorscale,
        zmin=0,
        zmax=n_categories, # Ensure scale covers all category IDs
        showscale=False, # Hide the default color bar, we use legend
        hoverongaps=False,
        # Custom hover text showing coordinates and category name
        customdata=category_grid[::-1], # Provide category names flipped too
        hovertemplate='X: %{x}<br>Y: %{y}<br>Type: %{customdata}<extra></extra>' # <extra></extra> removes trace name
    )

    fig = go.Figure(data=[heatmap_trace])

    # Add Path trace
    if path_indices:
        path_x, path_y_plotly = [], []
        for index in path_indices:
            x, y_orig = index_to_xy(index, width)
            if x is not None:
                # Adjust y for Plotly's bottom-up coordinate system if z was flipped
                y_plotly_coord = (height - 1) - y_orig
                path_x.append(x)
                path_y_plotly.append(y_plotly_coord)

        path_trace = go.Scatter(
            x=path_x,
            y=path_y_plotly,
            mode='lines+markers',
            name='Path',
            marker=dict(color=COLORS['PATH'], size=6, symbol='circle-open'),
            line=dict(color=COLORS['PATH'], width=2.5),
            hoverinfo='skip' # Don't show hover info for path points (heatmap covers it)
        )
        fig.add_trace(path_trace)

    # Add Start/End point traces
    start_x, start_y_plotly, end_x, end_y_plotly = None, None, None, None
    if start_pt_data and 'x' in start_pt_data and 'y' in start_pt_data:
        start_x = start_pt_data['x']
        start_y_plotly = (height - 1) - start_pt_data['y'] # Adjust y
        start_trace = go.Scatter(
            x=[start_x], y=[start_y_plotly], mode='markers', name='Start',
            marker=dict(color=COLORS['START'], size=12, symbol='circle', line=dict(color='black', width=1)),
            hovertemplate='Start<br>X: %{x}<br>Y: %{y}<extra></extra>'
        )
        fig.add_trace(start_trace)

    if end_pt_data and 'x' in end_pt_data and 'y' in end_pt_data:
        end_x = end_pt_data['x']
        end_y_plotly = (height - 1) - end_pt_data['y'] # Adjust y
        end_trace = go.Scatter(
            x=[end_x], y=[end_y_plotly], mode='markers', name='End',
            marker=dict(color=COLORS['END'], size=12, symbol='square', line=dict(color='black', width=1)),
            hovertemplate='End<br>X: %{x}<br>Y: %{y}<extra></extra>'
        )
        fig.add_trace(end_trace)

    # Add invisible Scatter traces for each terrain category JUST for the legend
    # This is a common Plotly workaround to add heatmap categories to the legend
    for category_name in unique_categories:
        fig.add_trace(go.Scatter(
            x=[None], y=[None], # No visible points
            mode='markers',
            marker=dict(color=category_colors[category_name], size=10, symbol='square'),
            showlegend=True,
            name=category_name # This name appears in the legend
        ))


    # 5. Configure Layout
    fig.update_layout(
        title=f"Grid Visualization ({width}x{height}) - {os.path.basename(base_filepath)}",
        xaxis_title="X Coordinate",
        yaxis_title="Y Coordinate",
        # Ensure square cells by matching axis scale
        yaxis_scaleanchor="x",
        yaxis_scaleratio=1,
        # Set axis ranges to prevent excessive whitespace
        xaxis_range=[-0.5, width - 0.5],
        yaxis_range=[-0.5, height - 0.5],
        # Adjust margins and potentially size
        margin=dict(l=50, r=50, t=80, b=50),
        # width=max(600, width * 1.5 + 200), # Heuristic for figure width
        # height=max(500, height * 1.5 + 100), # Heuristic for figure height
        legend_title_text='Legend',
        # Optional: Darker theme?
        # template='plotly_dark'
    )

    # Ensure the plot respects the aspect ratio even when zooming/panning
    fig.update_xaxes(constrain='domain')
    fig.update_yaxes(constrain='domain')


    # 6. Show or Save Plot
    print("Plot generated. Displaying...")
    if output_html:
        fig.write_html(output_html)
        print(f"Plot saved to: {output_html}")
    else:
        fig.show() # Opens in default browser or viewer


# --- Command Line Argument Parsing ---
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Visualize grid data (binary+JSON) using Plotly."
    )
    parser.add_argument(
        "base_filepath",
        help="Base path for saved data (e.g., 'grid_debug_data_TIMESTAMP')."
    )
    parser.add_argument(
        "-o", "--output",
        metavar="FILENAME.html",
        help="Save the interactive plot to an HTML file instead of displaying it."
    )
    args = parser.parse_args()

    if args.base_filepath.endswith(("_meta.json", "_data.bin")):
         print("Error: Please provide the base filepath without the suffix.", file=sys.stderr)
         sys.exit(1)

    visualize_grid_with_plotly(args.base_filepath, args.output)
