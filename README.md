**Video WalkThrough**
- [Here](https://www.youtube.com/watch?v=vNyCzOTHBpg) is the YouTube link!

**Dependencies** (use pip to install Python packages):
- CPP: [Eigen](https://eigen.tuxfamily.org/) C++ Library 
- CPP: CMake
- Python: Numpy, Matplotlib, Pandas
- Python: [flightgear_python](https://pypi.org/project/flightgear-python/) (optional)
- [FlightGear](https://www.flightgear.org/download/) (optional)

For better understanding of the GN&C algorithm and aircraft dynamics, please examine and run the files in **sandbox_py**.

**Build the C++ application:**
Run these commands (tested on Win/Mac/Linux):
- cd cpp
- mkdir build
- cmake ..
- cmake --build .
- run FixedWingGNC exe in build/Release, ideally from command line

**NOTE:** If you get an Eigen related error, be sure you have downloaded **and extracted** the eigen-#.#.# folder, it does not matter where the folder is located.

A file: trajectory.csv, will appear either in your home folder or directly in cpp/build/Release (recommended to run FixedWingGNC from terminal).

**Visualize flight trajectory in Python**
- Run fg_sim.py inside **py_fg**
- If you do not have FlightGear, comment out runFG() 
- The 3D and 2D plots will open and load the trajectory / actuator inputs

**Run simulation in FlightGear**
- Launch FlightGear
- Double check UDP port settings
- Start sim
- Run fg_sim.py (The 3D plots may open first prior to FG loading the trajectory)


