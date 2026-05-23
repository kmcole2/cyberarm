# Cyberarm Project Setup Guide

This guide covers how to initialize the repository layout, download and compile the hardware SDKs, and successfully integrate them with the main Python application code.

## Repository Architecture

Our project isolates third-party hardware SDKs within the `sdks/` directory to prevent dependency bleed. The core application logic lives entirely in `src/`.

```text
cyberarm/
├── sdks/                    # Hardware SDK modules
│   ├── pyorbbecsdk/         # Camera SDK (Compiled C++ Wrapper)
│   └── piper_sdk/           # Robotic Arm SDK
├── src/                     # Core application code
│   └── hello_orbbec.py      # Camera streaming entrypoint
└── README.md


Prerequisite: System Dependencies

Ensure you have a modern C++ compiler (gcc-11 or higher) and basic development utilities installed on your host Linux machine:
Bash

sudo apt update
sudo apt install build-essential cmake git python3-dev python3-pip

Installation & Build Steps

Run these commands from your main project development workspace folder (/home/kmcole/projects/Hackathon/):
1. Repository Initialization & Branch Target

Clone the official Orbbec SDK repository and switch to the stable v2-main branch:
Bash

git clone [https://github.com/orbbec/pyorbbecsdk.git](https://github.com/orbbec/pyorbbecsdk.git)
cd pyorbbecsdk
git checkout v2-main
cd ..

2. Update Python Compilation Tools

Install a modern version of pybind11 via pip. This ensures the C++ wrapper can build its modern structures on an older Python environment (Python 3.6):
Bash

pip3 install "pybind11>=2.10"

3. Native Binary Compilation

Navigate back into the SDK folder, clear out old configurations, and let CMake dynamically locate the new pybind11 headers to compile the library:
Bash

cd pyorbbecsdk
rm -rf build && mkdir build && cd build
cmake -Dpybind11_DIR=$(pybind11-config --cmakedir) -DBUILD_EXAMPLES=OFF ..
make -j$(nproc)
make install
cd ../..

4. Organize Directory Structure

Move the SDK folders into an isolated sdks/ ecosystem directory and shift your primary python application code files into src/:
Bash

mkdir -p cyberarm/sdks
mkdir -p cyberarm/src

# Move the SDK units
mv pyorbbecsdk/ cyberarm/sdks/
mv piper_sdk/ cyberarm/sdks/

# Move development scripts into the source group
mv cyberarm/*.py cyberarm/src/



Python Project Integration

Because the local directory tree layout can conflict with Python’s default module imports, we use Runtime Path Injection at the top of the application script to safely prioritize our compiled binary engine.
Code Setup Example (src/hello_orbbec.py)

Add this block at the absolute top of your entry script (before executing any camera imports):

import sys
import os

# 1. Calculate the absolute path targeting the compiled binaries
# Adjust string manually if executing outside the nested structure
BINARY_DIR = "/home/kmcole/projects/Hackathon/cyberarm/sdks/pyorbbecsdk/build"

# 2. Inject at position 0 to override raw subfolders
if BINARY_DIR not in sys.path:
    sys.path.insert(0, BINARY_DIR)

# 3. Safe to import now!
from pyorbbecsdk import Context, Pipeline

print("Orbbec SDK loaded successfully!")