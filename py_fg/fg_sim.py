import time
from flightgear_python.fg_if import FDMConnection
import pandas as pd
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#counter, FG heading offset (leave as is)
i = 0
headingOffset = (4*math.pi/180)

#runway elevation - change
runwayElev = 175

#max surface deflection (rad) - matches stabPosLimit/ailPosLimit in Aircraft.h (25 deg)
maxDeflection = 25*math.pi/180

def surfNorm(rad):
    #radians -> FlightGear normalized surface position, clipped to -1..1
    return max(-1.0, min(1.0, rad/maxDeflection))

def runFG():
    """
    Start FlightGear with `--native-fdm=socket,out,30,localhost,5501,udp --native-fdm=socket,in,30,localhost,5502,udp`
    (you probably also want `--fdm=null` and `--max-fps=30` to stop the simulation fighting with
    these external commands)
    """    
    #-------FG Connection---------#
    fdm_conn = FDMConnection(fdm_version=24)  # May need to change version from 24
    fdm_event_pipe = fdm_conn.connect_rx('localhost', 5501, fdm_callback)
    fdm_conn.connect_tx('localhost', 5502)
    fdm_conn.start()  # Start the FDM RX/TX loop

    while True:
        time.sleep(0.025)

def fdm_callback(fdm_data, event_pipe):
    global i, Lat, Lon, Alt, phi, theta, psi, del_e, del_a, del_r, V
    fdm_data.lat_rad = Lat[i]
    fdm_data.lon_rad = Lon[i]
    fdm_data.alt_m = Alt[i]
    fdm_data.phi_rad = phi[i]
    fdm_data.theta_rad = theta[i]
    fdm_data.psi_rad = psi[i] - headingOffset
    #FlightGear surface positions are normalized (-1..1), so scale radians by the max deflection.
    #Sim: +elevator = nose down, +aileron = roll left, +rudder = yaw left.
    #FG:  +elevator = nose down, +aileron = roll right, +rudder = yaw right.
    fdm_data.elevator = surfNorm(del_e[i])
    fdm_data.left_aileron = -surfNorm(del_a[i])
    fdm_data.right_aileron = surfNorm(del_a[i])
    fdm_data.rudder = -surfNorm(del_r[i])
    fdm_data.vcas = V[i]*1.94384   # m/s -> knots
    #Gear down - try it
    if (Alt[i] < 900):
        print("Gear down")
        fdm_data.gear_pos = [1,1,1]
        fdm_data.nose_wheel = 1
    i += 1
    #Stop condition
    if (i >= len(X) or Alt[i] < runwayElev):
        i = 0
    print("% complete: ", int((i/len(X))*100))
    return fdm_data  # return the whole structure

# Read CSV (semicolon-separated) - inside: cpp/build/Release 
df = pd.read_csv("../cpp/build/Release/trajectory.csv", sep=";", header=0)

# Store all waypoints
wp_df = df[df["Function"].str.contains("NavGuidance::loadWaypointLLA", na=False)]
coords = (
    wp_df["Message"]
    .str.extract(r"NED Waypoint Loaded:\s*([-0-9.]+),([-0-9.]+),([-0-9.]+)")
    .astype(float)
)
X_NED = coords[0].tolist()
Y_NED = coords[1].tolist()
Z_NED = coords[2].tolist()
print("X_NED =", X_NED)
print("Y_NED =", Y_NED)
print("Z_NED =", Z_NED)

# Drop rows containing "WaypointLLA"
df = df[~df["Function"].str.contains("WaypointLLA", na=False)]

# Keep only Simulator::runSim@rows (optional but safer)
df = df[df["Function"].str.contains("Simulator::runSim", na=False)]

# Extract the quoted numeric payload from Message column
# Example: "0.04,-0.462568,1.61506.........,-799.535"
numeric_data = df["Message"].str.strip('"')

# Split into 8 columns
#t, X_a, Y_a, Z_a, Lat, Lon, phi, theta, psi, del_e, del_a, del_r, X_SWP, Y_SWP, Z_SWP
cols = ["t", "X_a", "Y_a", "Z_a", "Lat", "Lon", "Alt", "phi", "theta",  "psi",
        "del_e", "del_a", "del_r", "XWP", "YWP", "ZWP", "V"]
data = numeric_data.str.split(",", expand=True)
data.columns = cols

# Convert all columns to float
data = data.astype(float)

# Result: each column stored as requested
t   = data["t"].to_list()
X   = data["X_a"].to_list()
Y   = data["Y_a"].to_list()
Z   = data["Z_a"].to_list()
Lat = data["Lat"]*(math.pi/180)
Lat = Lat.to_list()
Lon = data["Lon"]*(math.pi/180)
Lon = Lon.to_list()
Alt = data["Alt"].to_list()
phi = data["phi"].to_list()
theta = data["theta"].to_list()
psi = data["psi"].to_list()
del_e = data["del_e"].to_list()
del_a = data["del_a"].to_list()
del_r = data["del_r"].to_list()
WPX = data["XWP"].to_list()
WPY = data["YWP"].to_list()
WPZ = data["ZWP"].to_list()
V = data["V"].to_list()   # airspeed (m/s)

# Optional: view result
#print(data.head())

#3D plot
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')
# Plot CSV trajectory
ax.plot(X, Y, Z, label="Acft", color="blue")
ax.grid(False)
# Plot waypoints
ax.scatter(X_NED, Y_NED, Z_NED, color="red", s=60, label="Waypoints")
ax.plot(X_NED, Y_NED, Z_NED, color="red", linestyle="--")
ax.plot(WPX, WPY, WPZ, label="SWP", color="red")
# Labels
ax.set_xlabel("X (NED)")
ax.set_ylabel("Y (NED)")
ax.set_zlabel("Z (NED)")
ax.set_title("3D NED Trajectory")
ax.legend()
#Top down view
ax_xy = fig.add_subplot(3, 3, 3)
ax_xy.plot(X, Y, color="blue")
ax_xy.scatter(X_NED, Y_NED, color="red", s=20)
ax_xy.set_xlabel("X (North)")
ax_xy.set_ylabel("Y (East)")
ax_xy.set_title("Ground Track (X-Y)")
ax_xy.grid(False)

#2D plots
# Convert radians to degrees
r2d = 57.2957
del_e_deg = [item * r2d for item in del_e]
del_a_deg = [item * r2d for item in del_a]
del_r_deg = [item * r2d for item in del_r]
phi_deg   = [item * r2d for item in phi]
theta_deg = [item * r2d for item in theta]
psi_deg   = [item * r2d for item in psi]

# Create 3x2 subplot layout
fig, axs = plt.subplots(3, 2, figsize=(14, 10), sharex=True)

# ----------------------
# Left Column: Controls
# ----------------------

# Elevator
axs[0, 0].plot(t, del_e_deg, color='blue', linewidth=1.5)
axs[0, 0].set_ylabel(r'$\delta_e$ (deg)')
axs[0, 0].set_title('Control Surface Deflections')
axs[0, 0].grid(True, alpha=0.3)

# Aileron
axs[1, 0].plot(t, del_a_deg, color='red', linewidth=1.5)
axs[1, 0].set_ylabel(r'$\delta_a$ (deg)')
axs[1, 0].grid(True, alpha=0.3)

# Rudder
axs[2, 0].plot(t, del_r_deg, color='green', linewidth=1.5)
axs[2, 0].set_ylabel(r'$\delta_r$ (deg)')
axs[2, 0].set_xlabel('Time (s)')
axs[2, 0].grid(True, alpha=0.3)

# ----------------------
# Right Column: Attitude
# ----------------------

# Roll angle (phi)
axs[0, 1].plot(t, phi_deg, color='purple', linewidth=1.5)
axs[0, 1].set_ylabel(r'$\phi$ (deg)')
axs[0, 1].set_title('Attitude Angles')
axs[0, 1].grid(True, alpha=0.3)

# Pitch angle (theta)
axs[1, 1].plot(t, theta_deg, color='orange', linewidth=1.5)
axs[1, 1].set_ylabel(r'$\theta$ (deg)')
axs[1, 1].grid(True, alpha=0.3)

# Yaw angle (psi)
axs[2, 1].plot(t, psi_deg, color='brown', linewidth=1.5)
axs[2, 1].set_ylabel(r'$\psi$ (deg)')
axs[2, 1].set_xlabel('Time (s)')
axs[2, 1].grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

if __name__ == '__main__':  # NOTE: This is REQUIRED on Windows!
    runFG()
