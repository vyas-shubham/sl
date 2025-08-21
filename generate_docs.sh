#!/bin/bash

# Script to generate Doxygen documentation for SL Core Libraries
# Author: Auto-generated for repository documentation

echo "Generating Doxygen documentation for SL Core Libraries..."

# Check if doxygen is installed
if ! command -v doxygen &> /dev/null; then
    echo "Error: doxygen is not installed. Please install it first:"
    echo "  sudo apt-get install doxygen graphviz"
    exit 1
fi

# Check if we're in the right directory
if [ ! -f "doc/Doxyfile.standalone" ]; then
    echo "Error: doc/Doxyfile.standalone not found. Please run this script from the repository root."
    exit 1
fi

# Generate documentation
echo "Running doxygen..."
doxygen doc/Doxyfile.standalone

if [ $? -eq 0 ]; then
    echo "Documentation generated successfully!"
    echo "HTML documentation: doc/html/html/index.html"
    echo "LaTeX documentation: doc/html/latex/"
    echo ""
    echo "To view the documentation in a web browser:"
    echo "  open doc/html/html/index.html"
    echo "or"
    echo "  xdg-open doc/html/html/index.html"
else
    echo "Error: Documentation generation failed."
    exit 1
fi