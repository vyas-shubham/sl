# SL Library Examples

This directory contains examples demonstrating the key features and usage patterns of the SL (Simulation Lab) Core Libraries. These examples are designed to help you understand the library's data structures, algorithms, and best practices.

## Overview

The SL libraries provide essential functionality for robot control and simulation. These examples focus on the mathematical and algorithmic components that can be understood without the full robot simulation environment.

## Examples

### 01_data_structures.c
**Core Data Structures**

Demonstrates the fundamental data structures used throughout SL:
- `SL_Jstate`: Joint state (position, velocity, acceleration, torque)
- `SL_DJstate`: Desired joint state for control
- `SL_Cstate`: Cartesian state (3D position, velocity, acceleration)
- `SL_quat`: Quaternion orientation representation
- `SL_link`: Link parameters for rigid body dynamics

Key concepts covered:
- Structure initialization and field access
- Index conventions (`_X_`, `_Y_`, `_Z_`, etc.)
- Units and typical value ranges

### 02_quaternion_operations.c
**Quaternion and Rotation Operations**

Demonstrates quaternion math for 3D orientation handling:
- Euler angle to quaternion conversion (`eulerToQuat`, `eulerToQuatInv`)
- Quaternion to Euler angle conversion (`quatToEuler`, `quatToEulerInv`)
- Quaternion to rotation matrix (`quatToRotMat`, `quatToRotMatInv`)
- Quaternion derivatives (`quatDerivatives`)
- Quaternion error computation (`quatError`, `quatErrorVector`)

Key concepts covered:
- SL's Euler angle convention (a-b-g, X-Y-Z rotations)
- Forward vs. inverse transformations
- Angular velocity representation
- Orientation error for feedback control

### 03_min_jerk_trajectory.c
**Minimum Jerk Trajectory Generation**

Demonstrates smooth trajectory planning for point-to-point movements:
- 5th order polynomial trajectory generation
- Position, velocity, and acceleration profiles
- Multi-DOF coordinated movements
- Trajectory blending from non-zero initial conditions

Key concepts covered:
- Minimum jerk criterion for smooth motion
- Bell-shaped velocity profiles
- Iterative trajectory computation (as used in SL's goto_task)
- Servo-rate trajectory updates

### 04_signal_filtering.c
**Signal Filtering with Butterworth Filters**

Demonstrates the low-pass filtering utilities:
- 2nd order Butterworth filter implementation
- Filter structure and initialization
- Effect of cutoff frequency on smoothing
- Multi-channel filtering
- Step response characteristics

Key concepts covered:
- Difference equation implementation
- Cutoff frequency selection
- Trade-offs between smoothing and phase lag
- Real-time filter processing

### 05_realtime_ipc.c
**Real-Time Inter-Process Communication**

Demonstrates the multi-process architecture used in SL robot control with PREEMPT-RT Linux:
- Three concurrent processes (Motor Servo, Task Servo, Display)
- Shared memory for data exchange
- Mutex-protected access
- Condition variables for synchronization
- Sequence numbers and timestamps for data ordering

Key concepts covered:
- SL's servo architecture (motor, task, simulation, vision, openGL)
- Real-time communication patterns
- Thread-safe shared memory access
- Synchronization between processes running at different rates
- Compatibility with PREEMPT-RT and Xenomai

## Building the Examples

These examples are standalone and can be compiled without the full SL build environment.

### Quick Start

```bash
cd examples

# Compile all examples
gcc -Wall -o 01_data_structures 01_data_structures.c -lm
gcc -Wall -o 02_quaternion_operations 02_quaternion_operations.c -lm
gcc -Wall -o 03_min_jerk_trajectory 03_min_jerk_trajectory.c -lm
gcc -Wall -o 04_signal_filtering 04_signal_filtering.c -lm
gcc -Wall -o 05_realtime_ipc 05_realtime_ipc.c -lpthread -lm

# Run examples
./01_data_structures
./02_quaternion_operations
./03_min_jerk_trajectory
./04_signal_filtering
./05_realtime_ipc
```

### With Optimization

```bash
gcc -O2 -Wall -o 01_data_structures 01_data_structures.c -lm
```

### Using with Full SL Environment

When SL is properly installed with its dependencies:

```bash
# Set up SL environment
source <sl_install_path>/setup.sh

# Compile with SL libraries
gcc -o example examples/01_data_structures.c -I$LAB_INCLUDES -L$LAB_LIBDIR -lSLcommon -lm
```

## Key Concepts

### Index Conventions

SL uses 1-based indexing for many arrays to match robotics conventions:
- `_X_`, `_Y_`, `_Z_`: Cartesian indices (1, 2, 3)
- `_Q0_`, `_Q1_`, `_Q2_`, `_Q3_`: Quaternion indices (1, 2, 3, 4)
- `_A_`, `_B_`, `_G_`: Euler angle indices (1, 2, 3)

### Coordinate Frames

- **Global frame**: World/base coordinates
- **Local frame**: Link/body-fixed coordinates
- "Inv" suffix functions: local-to-global transformations
- Without "Inv": global-to-local transformations

### Units

Standard SI units are used throughout:
- Position: meters (m) or radians (rad)
- Velocity: m/s or rad/s
- Acceleration: m/s² or rad/s²
- Torque: Newton-meters (Nm)
- Mass: kilograms (kg)
- Inertia: kg·m²

## Further Reading

- **SL Documentation**: See the `doc/` directory for full API documentation
- **Doxygen**: Run `make -f Makefile.docs docs` to generate HTML documentation
- **Source Code**: The `src/` directory contains the full implementation

## License

These examples follow the same license as the SL libraries. See the main repository for license details.
