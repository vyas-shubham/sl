# SL Core Libraries Documentation

This directory contains the Doxygen configuration and generated documentation for the SL (Simulation Lab) Core Libraries.

## Prerequisites

To generate the documentation, you need:
- Doxygen (version 1.8 or later)
- Graphviz (for generating call graphs and dependency diagrams)

### Installation on Ubuntu/Debian:
```bash
sudo apt-get install doxygen graphviz
```

### Installation on macOS:
```bash
brew install doxygen graphviz
```

## Generating Documentation

### Quick Start
From the repository root directory, run:
```bash
./generate_docs.sh
```

### Manual Generation
Alternatively, you can generate the documentation manually:
```bash
doxygen doc/Doxyfile.standalone
```

## Output

The generated documentation will be placed in:
- **HTML documentation**: `doc/html/html/index.html`
- **LaTeX documentation**: `doc/html/latex/`

Open `doc/html/html/index.html` in your web browser to view the complete documentation.

## Documentation Features

The generated documentation includes:
- Complete API reference for all functions and data structures
- Module organization showing the different SL libraries:
  - SLcommon: Shared functions across SL processes
  - SLtask: User task management
  - SLmotor: Controller and I/O for robot/simulation
  - SLvision: Visual input processing
  - SLsimulation: Physical simulation management
  - SLopenGL: Graphics visualizations
  - SLros: ROS-specific functionality
  - And more...
- Function call graphs and caller graphs
- Include dependency diagrams
- Source code cross-references

## Configuration

The documentation is configured using `Doxyfile.standalone`, which is a self-contained configuration file that doesn't depend on external environment variables.

Key configuration options:
- Extracts documentation from `src/` and `include/` directories
- Generates both HTML and LaTeX output
- Includes call graphs and dependency diagrams
- Uses a tree view for easy navigation

## Troubleshooting

If you encounter issues:
1. Make sure doxygen and graphviz are properly installed
2. Verify you're running the script from the repository root directory
3. Check that the `src/` and `include/` directories exist and contain source files
4. Some include files may not be found during generation - this is normal for robot-specific headers and doesn't affect the main documentation