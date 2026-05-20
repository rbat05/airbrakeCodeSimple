# Rocket Desktop Simulation & Visualization Guide

This guide explains how to iterate on your flight math (Extended Kalman Filter and Model Predictive Control), run the desktop simulation, and visualize the results instantly without needing to flash the ESP32.

---

## 1. Where to Change What

The native simulation ignores the ESP32 hardware files (`main.cpp`, `baro.cpp`, `imu.cpp`, `Wire.h`) and strictly crunches the math using your recorded `hitl_data.h` metrics. 

**Files you can safely edit to tune the flight algorithm:**
* **`src/ekf.cpp` & `include/ekf.h`**: Modify your Kalman Filter constants, state transition matrices, noise variables, or the `ekf_predict`/`ekf_update` functions.
* **`src/Dynamics.cpp` & `include/Dynamics.h`**: Change your physical rocket integration logic, the MPC algorithm (`OptimiseControlInputBinarySearchConstraint`), drag calculation (`PredictApogee`), or launch/apogee detection logic.
* **`src/RocketVariables.c` & `include/RocketVariables.h`**: Update physical constants like empty mass, coefficient of drag limits, or reference area.
* **`desktop_sim/desktop_main.cpp`**: Edit what variables are fed into the simulator, how frequently they update, or add/remove headers and data exported to the final `.csv`.

---

## 2. Compiling and Running the Simulation

Whenever you make a tweak to the files above, you need to recompile the native C++ script and execute it to generate a new dataset.

**Step 1: Compile the Code**
Open a terminal in your project workspace and run the PlatformIO `native` build command:
```bash
pio run -e native
```
*(If the `pio` command isn't recognized, run the PlatformIO Core CLI from the VS Code bottom toolbar, or use the full path: `C:\Users\Raaghav Batra\.platformio\penv\Scripts\platformio.exe run -e native`)*

**Step 2: Execute the Math**
Once it compiles successfully, run the generated executable:
```bash
.pio\build\native\program.exe
```
This instantly processes the thousands of recorded frames and outputs the results to a new file in your project folder called **`out_sim.csv`**.

---

## 3. Visualizing the Data

To see the effects of your changes graphically, use the dark-mode interactive Python GUI `data_vis.py`.

**Step 1: Launch the Visualizer**
In your Python terminal, run:
```bash
python data_vis.py
```

**Step 2: Load the Data**
1. Click **1. Load CSV File** on the left panel.
2. Select the freshly generated **`out_sim.csv`** file in your project directory.
3. The GUI will confirm how many records were loaded, and automatically extract your Time start and end limits.

**Step 3: Graph Your Variables**
1. Click **+ Add Subplot** to create a custom plot window.
2. In the listbox that appears, hold `Ctrl` and select the variables you want to compare (e.g., select `altitudeM` and `ekf_height` to compare your raw baro data against the kalman filter's smoothing).
3. Click **+ Add Subplot** again to add a second stacked graph underneath (e.g., select `servo_cmd` to see when the airbrakes fired!)
4. Click **3. Draw / Update Plots** to render them on the right side.

If you further edit your tuning/math, just re-run the `pio run` and `.pio\build\native\program.exe` commands, then reload the CSV in the visualizer to see how the graph changed!
