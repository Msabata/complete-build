# OMAP Pathfinding Processor (Visual Studio Project)

This project implements a sophisticated pathfinding processor for orienteering maps (.omap format or compatible XML). It calculates time-optimal routes by considering terrain type, slope (using Tobler's hiking function), and various pathfinding algorithms, including CPU and GPU (CUDA) accelerated versions.

## Features

*   **Map Processing:**
    *   Parses .omap/XML map files (ISOM 2017-2 symbol interpretation).
    *   Handles complex geometries including points, lines, and areas with holes, gaps, and dashes.
    *   Generates a 2D logical grid representing base terrain traversal costs.
    *   Utilizes a two-pass rasterization process:
        *   Pass 1: Parallel boundary drawing (OpenMP).
        *   Pass 2: Feature-parallel area filling using a robust Scanline algorithm and rule application.
    *   Automatic calculation of logical grid resolution based on map coordinates.
*   **Elevation Data Integration:**
    *   Fetches Digital Elevation Model (DEM) data via an external Python script (`elevation_logic.py`) using `pybind11` for C++/Python interop.
    *   Handles potentially different resolutions and origins between the map's logical grid and the elevation grid.
    *   Uses `ElevationSampler` for bilinear interpolation of elevation values.
*   **Pathfinding Algorithms:**
    *   **CPU Implementations:**
        *   A* (Optimized)
        *   Dijkstra's Algorithm
        *   Breadth-First Search (BFS)
        *   Theta* (Any-Angle)
        *   Lazy Theta* (Optimized Any-Angle)
    *   **GPU (CUDA) Implementations:**
        *   Delta-Stepping
        *   HADS (Heuristic-Accelerated Delta-Stepping)
        *   A* (Experimental, one-node expansion per iteration)
    *   Dynamic "on-the-fly" edge cost calculation considering:
        *   Geometric distance (axial/diagonal).
        *   Base terrain cost from the processed map grid.
        *   Slope penalty derived from Tobler's hiking function: `exp(-3.5 * abs(Slope + 0.05))`.
*   **Waypoint Processing:**
    *   Extracts Start (701), Finish (706), and Control (703) points from a separate .omap "controls" file.
    *   Calculates the path sequentially between waypoints.
*   **Path Export:**
    *   Saves the calculated path back into a new .omap file, overlaying the route on a copy of the controls file.
*   **GUI (Optional - if `main_window.cpp` is part of this solution):**
    *   User-friendly interface built with Qt 6.x.
    *   File loading, parameter configuration for map processing and all algorithms.
    *   Asynchronous backend processing to keep UI responsive.
    *   Light/Dark theme support.
    *   (Future: Map and path visualization within the app).

## Project Structure (Key Components)

*   `complete build.sln`: Visual Studio Solution file.
*   `complete build/`: Main project directory.
    *   `complete build.vcxproj`: MSVC project file.
    *   `*.cpp`, `*.hpp`, `*.h`: C++ source and header files for all modules (map processing, algorithms, I/O, logic).
    *   `*.cu`, `*.cuh`: CUDA kernel implementations and device utility headers.
    *   `elevation_logic.py`: Python script for fetching elevation data.
    *   `tinyxml2.cpp/.h`: XML parsing library.
    *   `httplib.h`: Single-file C++ HTTP/HTTPS client/server library (if used by `ApiClient.cpp`).
    *   `json.hpp`: JSON for Modern C++ (if used).
*   (Other directories like `x64/Debug`, `x64/Release` are build outputs).

## Prerequisites

1.  **Microsoft Visual Studio:** 2022 or a compatible version with C++17 support.
    *   Ensure "Desktop development with C++" workload is installed.
    *   (Optional) If using CMake features within VS, CMake tools for Visual Studio.
2.  **NVIDIA CUDA Toolkit:** Version 12.5 (or as specified in project settings).
    *   Must be correctly integrated with Visual Studio.
3.  **NVIDIA GPU Driver:** A recent driver compatible with your CUDA Toolkit version.
4.  **Python:** Version 3.9 or compatible.
    *   Ensure Python is added to your system's PATH.
    *   Required Python packages:
        ```bash
        pip install pybind11 requests pyproj
        ```
5.  **(Optional) Qt 6 Framework:** If the GUI (`main_window.cpp`) is part of this solution and you intend to build it.
    *   Qt 6.3 or a compatible version.
    *   Qt VS Tools extension for Visual Studio.

## Building the Project

1.  **Clone the repository:**
    ```bash
    git clone <repository-url>
    cd <repository-directory>
    ```
2.  **Open the Solution:** Double-click `complete build.sln` to open it in Visual Studio.
3.  **Install Python Dependencies:** If not already installed globally or in a virtual environment accessible by the project, install the required Python packages (see Prerequisites).
4.  **Configure Build:**
    *   Select the desired solution configuration (e.g., `Release` or `Debug`).
    *   Select the target platform (e.g., `x64`).
5.  **Check Project Properties (First time or if issues occur):**
    *   Right-click on the "complete build" project in Solution Explorer -> Properties.
    *   **C/C++ -> General -> Additional Include Directories:** Ensure paths to CUDA Toolkit includes, Python includes, and Pybind11 includes are correctly set up. (Visual Studio often picks these up from CUDA/Python installations if they are in PATH or standard locations).
    *   **Linker -> General -> Additional Library Directories:** Ensure paths to CUDA Toolkit libraries and Python libraries are correct.
    *   **Linker -> Input -> Additional Dependencies:** Ensure `cudart_static.lib` (or `cudart.lib`), Python's `.lib` file, and any other required CUDA libraries are listed.
    *   **CUDA C/C++:** Verify "Target Machine Platform" (e.g., 64-bit) and "Code Generation" (e.g., `compute_XX,sm_XX` for your target GPU architectures - Pascal, Volta, Turing are good defaults: `compute_61,sm_61;compute_70,sm_70;compute_75,sm_75`).
6.  **Build the Solution:**
    *   From the Visual Studio menu, select Build -> Build Solution (or press F7).
7.  **Run:**
    *   The executable (e.g., `x64/Release/complete build.exe`) will be generated.
    *   Ensure `elevation_logic.py` is in the same directory as the executable, or the path to it is correctly configured within the C++ code (e.g., relative to the executable or an absolute path if using Python's `sys.path` manipulation).
    *   (If building the Qt GUI) Ensure Qt DLLs are available in the executable's directory or in the system PATH.

## Usage (Console Application - if `complete build.cpp` is the main)

The console application is typically run with command-line arguments specifying the input map, controls file, and other parameters. (Describe command-line interface here if applicable).

```bash
./complete_build.exe --map <map_file.omap> --controls <controls_file.omap> --algo <algorithm_name> ...
