# **Rocketry Ground Systems: State Estimation & Analysis**

## **Project Objective**

This document outlines the software architecture for a ground-systems application designed for state estimation and performance analysis of a liquid bipropellant rocket. The primary objective of this project is to process raw sensor telemetry and produce a single, high-fidelity, time-consistent estimate of the rocket's complete state vector, including its position, velocity, and orientation.

This state estimation is a critical task, as no single sensor provides perfect data. Global Positioning System (GPS) receivers, for example, offer absolute position but are characterized by low update rates and significant noise. Barometric pressure sensors provide altitude data that can be influenced by weather conditions. Conversely, Inertial Measurement Units (IMUs), which consist of accelerometers and gyroscopes, deliver high-frequency data but are subject to compounding integration errors, known as drift. This software is designed to intelligently fuse these disparate data sources, leveraging their respective strengths to mitigate their individual weaknesses.

The resulting "fused state" serves as the authoritative source of truth for all post-flight analysis. It enables engineering teams to conduct rigorous performance characterizations and answer critical diagnostic questions by correlating the vehicle's observed flight dynamics with subsystem telemetry.

## **High-Level Software Design**

The software is founded upon a modular, four-part architecture. This design deliberately segregates the high-performance core processing logic (implemented in C++) from the flexible analysis and visualization tools (implemented in Python). Such a design facilitates both high-throughput computation and agile data analysis. Furthermore, this modularity ensures the system is highly testable, as each component can be independently validated.

### **Module 1: Data Ingestion & Pre-processing (C++)**

* **Purpose:** This module is responsible for the ingestion, parsing, and sanitization of all incoming sensor data from myriad sources, including pre-recorded log files and real-time telemetry streams. It functions as the system's primary data intake and conditioning layer, isolating the core logic from the complexities of raw data formats.  
* **Responsibilities:**  
  * **Parsers:** Implementation of robust parsing routines for each discrete sensor data format, which may include binary log files from an onboard SD card, text-based NMEA sentences from a GPS, or custom-defined telemetry packets arriving via a radio link.  
  * **Timestamp Normalization:** This is a fundamental prerequisite for accurate sensor fusion. Sensor data will arrive at different rates and potentially reference different system clocks. This subsystem must convert all data point timestamps to a single, high-resolution, monotonic time base to ensure correct temporal correlation.  
  * **Unit Conversion:** Standardization of all sensor readings to SI units. This ensures all subsequent calculations within the physics model are consistent.  
  * **Coordinate System Alignment:** A given sensor's local coordinate system may not align with the vehicle's defined reference frame. This subsystem transforms all relevant sensor data vectors to align with a single, common vehicle frame.  
* **Output:** Clean, time-synchronized, and standardized sensor data streams, which are passed to the subsequent module via well-defined C++ data structures.

### **Module 2: State Estimation Engine (C++)**

* **Purpose:** This module serves as the computational core of the project. It fuses the clean sensor data from Module 1 to produce an optimal, drift-corrected state estimate.  
* **Core Algorithm:** An **Extended Kalman Filter (EKF)** is employed. A standard Kalman Filter is restricted to linear systems; however, rocket flight dynamics are inherently non-linear. The EKF accommodates this by performing a first-order linearization (using Jacobians) at each time step, thereby providing a highly accurate estimate for non-linear systems.  
* **Process:** The EKF operates via a continuous, two-step recursive loop:  
  1. **Predict Step (High-Frequency):** For every high-rate IMU measurement (accelerometer, gyroscope), the filter utilizes a 6-Degrees-of-Freedom (6-DOF) physics-based motion model to *predict* the vehicle's next state. The IMU data functions as the "control input" to this dynamic model. This step provides a smooth, high-frequency state estimate but will accumulate integration drift if uncorrected.  
  2. **Update Step (Low-Frequency):** When a slower, absolute measurement becomes available, the filter performs a *correction*. It compares its internal *prediction* with the external *measurement* and computes a "Kalman Gain." This gain is an optimal weighting factor that determines the degree to which the new measurement is trusted and incorporated to correct the predicted state.  
     * **GPS data** corrects the (x, y, z) position vector.  
     * **Barometer data** corrects the altitude (z) and vertical velocity.  
     * Magnetometer data corrects the orientation, primarily yaw (heading).  
       This update step is what anchors the high-frequency prediction to ground truth and systematically eliminates IMU drift.  
* **Output:** A high-frequency, time-stamped log of the rocket's fully estimated state (e.g., a 10-element vector containing position, velocity, and orientation represented as a quaternion) at the same sampling rate as the IMU.

### **Module 3: Logging & Output (C++)**

* **Purpose:** This module collects all processed data (both raw and estimated) and serializes it to a master log file. This file becomes the "single source of truth" for a given flight, ensuring that all subsequent analysis is both repeatable and consistent.  
* **Responsibilities:**  
  * **Data Merging:** Intelligently combines the high-frequency estimated state data (from Module 2\) with the lower-frequency, time-synced raw sensor data (from Module 1\) into a single, comprehensive data structure.  
  * **File Writing:** Writes the consolidated log data to a standardized, high-performance file format. While .csv offers simplicity, binary formats such as **HDF5** or **Apache Feather** are strongly preferred. These formats offer significantly faster read/write operations, more efficient storage of large data volumes, and the ability to store critical metadata within the file itself.  
* **Output:** A single, self-contained master log file that encapsulates all data necessary for comprehensive post-flight analysis.

### **Module 4: Analysis & Visualization (Python)**

* **Purpose:** This module is designed to ingest the master log file and generate human-readable plots, dashboards, and reports for all engineering teams (Propulsion, Structures, GNC, etc.). Its function is to translate the raw numerical data into actionable engineering insights.  
* **Responsibilities:**  
  * **Data Loading:** Ingest the master log file into a standard DataFrame structure for analysis.  
  * **Derived Metrics:** Calculate key performance indicators (KPIs) from the fused state vector. This step is where significant value is generated. Examples include:  
    * **Apogee & Maximum Velocity:** Fundamental flight performance metrics.  
    * **Dynamic Pressure (Max-Q):** A critical metric for the Structures team to validate aerodynamic stress models against flight data.  
    * **Thrust Curve Estimation:** Plotting the vehicle's fused acceleration (after accounting for gravity and estimated drag) provides a real-world thrust curve, enabling comparison against engine test stand data for the Propulsion team.  
    * **Attitude Error:** Plotting the difference between the rocket's *fused orientation* and its *commanded orientation* (if applicable).  
  * **Visualization:** Generate static 2D plots, 3D flight path visualizations, and interactive dashboards.  
* **Output:** Shareable reports and web-based dashboards that permit engineers to interactively explore the flight data.

## **Technology Stack**

This project utilizes a hybrid C++/Python stack to leverage the distinct advantages of each language.

* **Modules 1-3 (Core Processing):**  
  * **Language:** C++ (Selected for its high performance, fine-grained memory control, and deterministic execution, which are critical for high-frequency EKF calculations. This choice also enhances the code's portability for potential future migration to an onboard flight computer).  
  * **Key Libraries:**  
    * **Eigen:** The industry standard for high-performance linear algebra in C++. It is header-only, highly optimized, and possesses an expressive API well-suited for implementing Kalman Filter matrix equations.  
    * **fstream / iostream:** For standard file I/O operations.  
    * **cmath:** For standard mathematical operations.  
    * **(Optional) HDF5-C-lib:** The official C-language API is required if HDF5 is chosen as the logging format.  
* **Module 4 (Analysis & Viz):**  
  * **Language:** Python (Selected for its world-class data science ecosystem, which permits rapid development, iteration, and the creation of sophisticated visualizations with minimal code).  
  * **Key Libraries:**  
    * **Pandas:** The primary tool for loading, cleaning, and performing time-series analysis on the log file data.  
    * **NumPy:** The foundational library for all numerical computing in Python, used for efficient vectorized operations on data arrays.  
    * **Matplotlib / Seaborn:** The standard for generating high-quality, publication-ready static 2D plots.  
    * **Plotly / Dash:** The preferred solution for creating interactive, web-based 2D/3D visualizations and dashboards, allowing for deeper data exploration by the end-user.

## **Data Flow**

This architecture establishes a clean interface between the C++ processing pipeline and the Python analysis suite, using the master log file as the sole data-exchange medium.

1. Raw log files or live telemetry streams are fed into **Module 1 (C++)** for cleaning, parsing, and normalization.  
2. The clean data streams are passed in-memory to **Module 2 (C++)**.  
3. Module 2 executes the EKF (either in real-time or in post-processing), generating a high-frequency state estimate.  
4. **Module 3 (C++)** collects all relevant data and writes the final, comprehensive master log file (e.g., flight\_data.hdf5).  
5. **(Analysis Phase)** At this juncture, the C++ application's execution is complete. A user then executes the **Module 4 (Python)** analysis scripts.  
6. The Python scripts read the specified flight\_data.hdf5 into a Pandas DataFrame.  
7. All analysis, metric derivation, plotting, and dashboard generation is performed within the Python environment, which outputs the final reports.

This separation of concerns means the computationally intensive C++ process can be executed on a dedicated build server or high-performance machine, while any engineer can subsequently perform the Python-based analysis on a standard laptop using only the resulting log file.