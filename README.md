Requirements (use pip to install Python dependencies):
- CPP: [Eigen](https://eigen.tuxfamily.org/) C++ Library 
- CPP: CMake
- Python: Numpy, Matplotlib, Pandas
- Python: [flightgear_python](https://pypi.org/project/flightgear-python/) (optional)
- [FlightGear](https://www.flightgear.org/download/) (optional)

For better understanding of the GN&C algorithm and aircraft dynamics, please understand and run the files inside **sandbox_py**, before going over cpp/.

**Build the C++ application:**
Run these cmake commands in sequence:
- cd cpp/build
- cmake ..
- cmake --build .
- run exe inside the build/Release folder

A file "trajectory.csv" should appear either in your home folder or in cpp/build/Release (recommended: run the exe from terminal).

**Visualize flight trajectory in Python**
- Run fg_sim.py inside **py_fg**
- If you do not have FlightGear, comment out runFG() 
- The 3D and 2D plots will open and load the trajectory / actuator inputs

**Run simulation in FlightGear**
- Launch FlightGear
- Double check UDP port settings
- Start sim
- Run fg_sim.py (The 3D plots may open first prior to FG loading the trajectory)


