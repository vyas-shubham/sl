# SL Core Libraries

The SL (Simulation Lab) Core Libraries are the foundation for robot control and simulation systems. These libraries provide essential functionality for managing robot controllers, simulations, and various I/O operations.

## Overview

The SL Core Libraries include several modules:

- **SLcommon**: Shared functions across different SL processes
- **SLtask**: User task management process
- **SLmotor**: Controller and I/O for robot/simulation
- **SLvision**: Visual input processing
- **SLsimulation**: Physical simulation management
- **SLopenGL**: Graphics visualizations with OpenGL
- **SLros**: ROS-specific functionality
- **SLparameterEstimation**: Rigid Body Dynamics parameter estimation
- **SLskeletons**: Robot-specific include file skeletons

## Documentation

This repository includes comprehensive Doxygen documentation that covers all APIs, functions, and modules.

### Generating Documentation

To generate the complete API documentation:

#### Prerequisites
```bash
# Ubuntu/Debian
sudo apt-get install doxygen graphviz

# macOS
brew install doxygen graphviz
```

#### Generate Documentation
```bash
# Option 1: Use the convenience script
./generate_docs.sh

# Option 2: Use the documentation Makefile
make -f Makefile.docs docs

# Option 3: Run doxygen directly
doxygen doc/Doxyfile.standalone
```

The generated documentation will be available at `doc/html/html/index.html`.

### Documentation Features

The generated documentation includes:
- Complete API reference for all functions and data structures
- Module organization with detailed descriptions
- Function call graphs and dependency diagrams
- Source code cross-references
- Searchable interface with tree navigation

## Building

The project uses a combination of CMake and traditional Makefiles. See the individual module directories for specific build instructions.

## License

This software is provided under a slightly modified version of the LGPL license. For details, see: http://www-clmc.usc.edu/software/license

## Credits

Originally developed for the CLMC/AMD labs at the University of Southern California and the Max-Planck-Institute for Intelligent Systems.

Copyright by Stefan Schaal, 2014