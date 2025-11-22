# Integrated Sensing and Communication (ISAC) with AIM-UKF-JPDA Tracking

## Overview

This project implements a high-fidelity simulation framework for **Integrated Sensing and Communication (ISAC)** using 5G New Radio (NR) waveforms. It focuses on tracking **highly maneuvering targets** (e.g., vehicles making sharp turns or accelerating) by fusing range, angle, and Doppler measurements.

The core innovation is the **AIM-UKF-JPDA** algorithm:
*   **AIM (Adaptive Interacting Multiple Model)**: Dynamically adjusts process noise based on maneuver detection.
*   **UKF (Unscented Kalman Filter)**: Handles non-linear bistatic measurements better than EKF.
*   **JPDA (Joint Probabilistic Data Association)**: Robustly associates measurements to tracks in cluttered environments.

## Key Features

*   **5G Waveform Generation**: Uses MATLAB's 5G Toolbox to generate realistic OFDM waveforms.
*   **Physics-Based Channel**: Simulates multipath propagation and target scattering using the Phased Array System Toolbox.
*   **3D Detection**: Extracts Range, Angle of Arrival (AoA), and Radial Velocity (Doppler) from the received signal.
*   **Advanced Tracking**:
    *   **IMM**: Switches between Constant Velocity (CV), Constant Turn (CT), and Constant Acceleration (CA) models.
    *   **Doppler Fusion**: Directly uses radial velocity measurements to improve tracking accuracy.
    *   **Adaptive Noise**: Increases process noise ($Q$) during maneuvers to prevent track loss.

## Project Structure

### Core Scripts

*   **`run_ISAC_Simulation_Core.m`**: The main physics engine. It takes a configuration and scenario, runs the 5G simulation, processes the channel, performs CFAR detection, runs the tracker, and returns performance metrics (RMSE, track loss).
*   **`AIM_UKF_JPDA_Experiment.m`**: The experiment runner. It conducts ablation studies (e.g., Baseline vs. Proposed) by running Monte Carlo simulations using the core engine.
*   **`ISAC_Scenario.m`**: A standalone script for visualizing a single run with real-time plots of the scenario, detections, and tracks.

### Configuration

*   **`FiveG_Waveform_Config.m`**: Configures the 5G waveform parameters (carrier frequency, bandwidth, subcarrier spacing).
*   **`Supporting_Functions/`**: Contains helper functions for tracking, plotting, and data processing.
    *   `helperConfigureTracker.m`: Sets up the IMM-UKF-JPDA tracker.
    *   `helperInitIMM.m`: Initializes the IMM filter structure.
    *   `isacBistaticMeasurementFcn.m`: The non-linear measurement model.

## Getting Started

### Prerequisites

*   MATLAB R2021b or later
*   5G Toolbox
*   Phased Array System Toolbox
*   Sensor Fusion and Tracking Toolbox
*   Communications Toolbox

### Running a Visualization Demo

To see the system in action with real-time plots:

```matlab
% Run the scenario visualization
ISAC_Scenario
```

### Running Experiments

To run the full ablation study and generate performance metrics:

```matlab
% Run the experiment loop
AIM_UKF_JPDA_Experiment
```

This will compare different algorithms (e.g., Baseline CV-EKF vs. Proposed AIM-UKF-JPDA) across multiple scenarios (High Maneuver, Acceleration, Crossing).

## Algorithm Details

### Tracking Architecture

The system uses an **Interacting Multiple Model (IMM)** filter with three sub-models:
1.  **Constant Velocity (CV)**: For non-maneuvering phases.
2.  **Constant Turn (CT)**: For turning maneuvers.
3.  **Constant Acceleration (CA)**: For longitudinal maneuvers.

### Adaptive Process Noise (AIM)

The **AIM** module monitors the likelihood of the maneuver models (CT and CA). If the probability of a maneuver model exceeds a threshold, the process noise covariance ($Q$) is temporarily boosted to allow the filter to adapt quickly to the change in motion.

### Measurement Model

The tracker uses a **bistatic measurement model**:
$$ z = [R_{bistatic}, \theta_{AoA}, \dot{R}_{bistatic}]^T $$
Where:
*   $R_{bistatic}$ is the bistatic range (Tx -> Target -> Rx).
*   $\theta_{AoA}$ is the Angle of Arrival at the receiver.
*   $\dot{R}_{bistatic}$ is the bistatic range rate (Doppler), derived from the relative velocities.

## License

This project is for academic research and educational purposes.

