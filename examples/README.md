# SL Library Examples

This directory contains examples demonstrating the key features and usage patterns of the SL (Simulation Lab) Core Libraries. These examples use the actual SL library headers and link against the SL libraries.

## Prerequisites

These examples require a properly configured SL environment:
- SL libraries built and installed
- `$LAB_ROOT` environment variable set to the SL installation root
- `$LAB_LIBDIR` environment variable set to the SL library directory
- `$LAB_INCLUDES` environment variable set to the SL include directory

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

**Libraries used:** SLcommon, utility

### 02_quaternion_operations.c
**Quaternion and Rotation Operations**

Demonstrates quaternion math for 3D orientation handling using SL library functions:
- `eulerToQuat`, `eulerToQuatInv`: Euler angles to quaternion conversion
- `quatToEuler`, `quatToEulerInv`: Quaternion to Euler angle conversion
- `quatToRotMat`, `quatToRotMatInv`: Quaternion to rotation matrix
- `quatDerivatives`: Compute quaternion derivatives from angular velocity
- `quatError`, `quatErrorVector`: Quaternion error computation

Key concepts covered:
- SL's Euler angle convention (a-b-g, X-Y-Z rotations)
- Forward vs. inverse transformations
- Angular velocity representation
- Orientation error for feedback control

**Libraries used:** SLcommon, utility

### 03_min_jerk_trajectory.c
**Minimum Jerk Trajectory Generation**

Demonstrates smooth trajectory planning for point-to-point movements:
- 5th order polynomial trajectory generation
- Position, velocity, and acceleration profiles
- Multi-DOF coordinated movements using SL_DJstate arrays
- Trajectory computation similar to SL's goto_task

Key concepts covered:
- Minimum jerk criterion for smooth motion
- Bell-shaped velocity profiles
- Iterative trajectory computation
- Using SL data structures for trajectory representation

**Libraries used:** SLcommon, SLtask, utility

### 04_signal_filtering.c
**Signal Filtering with Butterworth Filters**

Demonstrates the low-pass filtering utilities from SL_filters:
- Filter structure and initialization
- Using the `filt()` function for real-time filtering
- Filter cutoff selection guidelines
- Multi-channel filtering patterns

Key concepts covered:
- 2nd order Butterworth filter implementation
- Cutoff frequency selection
- Trade-offs between smoothing and phase lag
- Integration with SL sensor processing

**Libraries used:** SLcommon, utility

### 05_realtime_ipc.c
**Real-Time Inter-Process Communication**

Demonstrates the multi-process architecture used in SL robot control:
- SL servo architecture (motor, task, simulation, vision, openGL, ROS)
- Shared memory structures (smJointStates, smJointDesStates, etc.)
- Semaphore synchronization between servos
- Real-time mutex wrappers (SL_rt_mutex)
- Message passing between servos

Key concepts covered:
- SL's concurrent servo architecture
- Real-time communication patterns
- Thread-safe shared memory access
- Synchronization between processes running at different rates
- Compatibility with PREEMPT-RT and Xenomai

**Libraries used:** SLcommon, utility, pthread

## Building the Examples

### Using CMake (Recommended)

These examples can be built using the CMake build system:

```bash
cd examples
mkdir build
cd build
cmake ..
make
```

### Manual Compilation

Each example can be compiled manually with the proper includes and libraries:

```bash
# Example: Compile data structures example
gcc -I$LAB_ROOT/include -I../include -L$LAB_LIBDIR \
    -o 01_data_structures 01_data_structures.c \
    -lSLcommon -lutility -lm

# Example: Compile quaternion operations
gcc -I$LAB_ROOT/include -I../include -L$LAB_LIBDIR \
    -o 02_quaternion_operations 02_quaternion_operations.c \
    -lSLcommon -lutility -lm

# Example: Compile min-jerk trajectory
gcc -I$LAB_ROOT/include -I../include -L$LAB_LIBDIR \
    -o 03_min_jerk_trajectory 03_min_jerk_trajectory.c \
    -lSLcommon -lSLtask -lutility -lm

# Example: Compile signal filtering
gcc -I$LAB_ROOT/include -I../include -L$LAB_LIBDIR \
    -o 04_signal_filtering 04_signal_filtering.c \
    -lSLcommon -lutility -lm

# Example: Compile real-time IPC
gcc -I$LAB_ROOT/include -I../include -L$LAB_LIBDIR \
    -o 05_realtime_ipc 05_realtime_ipc.c \
    -lSLcommon -lutility -lpthread -lm
```

## Key Concepts

### SL Headers

The examples use the following SL headers:
- `SL.h`: Core data structures and definitions
- `SL_common.h`: Common utility functions (quaternion operations, etc.)
- `SL_filters.h`: Signal filtering structures and functions
- `SL_shared_memory.h`: Shared memory structures for IPC
- `SL_rt_mutex.h`: Real-time mutex wrappers

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

## SL Library Dependencies

These examples link against the following SL libraries:
- **SLcommon**: Common utilities, quaternion operations, filtering
- **SLtask**: Task servo functionality, trajectory generation
- **utility**: Utility library (Matrix, Vector operations)

## Further Reading

- **SL Documentation**: See the `doc/` directory for full API documentation
- **Doxygen**: Run `make -f Makefile.docs docs` to generate HTML documentation
- **Source Code**: The `src/` directory contains the full implementation

## License

These examples follow the same license as the SL libraries. See the main repository for license details.
