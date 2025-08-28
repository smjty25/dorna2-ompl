# dorna2-ompl

C++ / Python bindings for [OMPL](https://ompl.kavrakilab.org/) (Open Motion Planning Library) with [pybind11](https://pybind11.readthedocs.io/) and additional robotics utilities (FCL, URDFDOM, Eigen).  
The project builds a Python extension module (`dornaompl`) that you can import directly in Python for planning experiments.

---

## Features
- C++ motion planning using OMPL + FCL + URDFDOM
- Python bindings via pybind11
- Automatic copy/install of dependent DLLs on Windows
- Resource management (URDF, configs) included in the package

---

## Requirements
- **CMake** ≥ 3.21  
- **Visual Studio 2022** (Windows) or any modern compiler (Linux/macOS)  
- **vcpkg** for dependencies (OMPL, FCL, Eigen3, URDFDOM, pybind11)  
- **Python** 3.8–3.12 (64-bit, from [python.org](https://www.python.org/downloads/))  

Make sure Python was installed with *Development headers* (it should include `include/Python.h` and `libs/pythonXY.lib`).

---

## Build Instructions

### 1. Clone repo and submodules
For Linux:

```powershell

sudo apt update
sudo apt install pkg-config autoconf libtool intltool automake autoconf-archive gettext


git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
./bootstrap-vcpkg.sh
echo 'export VCPKG_ROOT="$HOME/vcpkg"' >> ~/.bashrc
echo 'export PATH="$VCPKG_ROOT:$PATH"' >> ~/.bashrc
source ~/.bashrc

vcpkg install ompl
vcpkg install urdfdom
vcpkg install urdfdom-headers
vcpkg install fcl
vcpkg install pybind11

cd ~
git clone dorna2-ompl
cd dorna2-ompl

cmake --preset rpi-arm64   -D Python3_EXECUTABLE="$(command -v python3)"   -D Python3_INCLUDE_DIR="/usr/include/python3.11"   -D Python3_LIBRARY="/usr/lib/aarch64-linux-gnu/libpython3.11.so"
cmake --build build/rpi-arm64 --config Release -j
sudo cmake --install build/rpi-arm64 --config Release



