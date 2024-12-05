
# Autonomous Vehicle Simulator Project

This project is designed to simulate an autonomous vehicle using the FSDS (Flight Sim Data Server) package and integrate real-time camera feedback for visual monitoring. The system employs a simulated vehicle in a test environment to detect obstacles (cones) using LiDAR data and navigate autonomously.

## Overview of the Work and Approach

### Objective
To create an autonomous driving script that:
- Reads LiDAR data from the simulated environment.
- Detects cones on the road and navigates around them.
- Displays a real-time camera feed to provide visual feedback of the vehicle's environment.

### Approach
1. **Sensor Integration**: The script connects to the simulator using FSDS and utilizes LiDAR and camera data.
2. **LiDAR Processing**: The script processes LiDAR point cloud data to detect clusters representing cones.
3. **Steering and Throttle Control**: The algorithm calculates steering angles and throttle adjustments based on the detected cones' positions.
4. **Camera Integration**: The script captures live camera images from the simulator and displays them using OpenCV for real-time monitoring.

### Operating System Used
- The development environment was set up on **Windows 10**.

## Installation and Run Instructions

### Prerequisites
Ensure you have the following software installed:
- **Python 3.x**
- **FSDS Python package** (for interacting with the simulator)
- **OpenCV** (for displaying the live camera feed)
- **Matplotlib** (for visual plotting)

### Installation Steps

1. **Clone the repository**:
    ```bash
    git clone https://github.com/yourusername/your-repo.git
    cd your-repo
    ```

2. **Install dependencies**:
    Create a virtual environment (optional but recommended) and install the necessary packages:
    ```bash
    python -m venv env
    source env/bin/activate  # On Windows, use `env\Scriptsctivate`
    pip install -r requirements.txt
    ```

    Add `requirements.txt` content:
    ```
    fsds
    opencv-python
    matplotlib
    numpy
    ```

3. **Prepare the simulator**:
    Ensure the FSDS server is running and configured to work with your vehicle and sensors. Refer to the FSDS documentation for proper setup.

### Run the Script

Navigate to the directory containing `autonomous_example.py` and run the script:
```bash
python autonomous_example.py
```

The script will begin running, control the vehicle, and display live camera feed and a real-time plot of the detected cones.

## Notes
- Ensure the simulator is configured to send camera data and LiDAR data to your script.
- If any error occurs, double-check the FSDS client configuration and the simulator's setup.
