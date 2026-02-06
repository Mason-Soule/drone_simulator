# Drone Simulator

A Python-based drone simulation environment designed to work alongside **ArduPilot** for testing, experimentation, and development of drone behaviors and MAVLink-based control systems.

This project is intended for learning, experimentation, and future expansion into more advanced flight simulation and control logic.

---

## Features

* Python-based simulation logic
* Integration with **ArduPilot**
* MAVLink communication via `pymavlink`
* Isolated Python virtual environment (inside the simulator directory)

---

## Project Structure

```
Drone_SIM/
├── ardupilot/              # Forked and cloned ArduPilot repository
└── drone_simulator/        # This repository
    ├── venv/               # Python virtual environment (local to simulator)
    ├── src/                # Simulator source code (if applicable)
    └── README.md
```

---

## Setup Instructions

### 1. Create a New Project Folder

Create a parent directory to hold both ArduPilot and the drone simulator.

```bash
mkdir Drone_SIM
cd Drone_SIM
```

---

### 2. Fork ArduPilot

1. Go to the official ArduPilot repository:
   [https://github.com/ArduPilot/ardupilot](https://github.com/ArduPilot/ardupilot)
2. Fork the repository to your GitHub account.

---

### 3. Clone ArduPilot

Clone **your fork** of ArduPilot into the parent directory.

```bash
git clone https://github.com/<your-username>/ardupilot.git
```

---

### 4. Clone the Drone Simulator Repository

Clone this repository into the same parent directory.

```bash
git clone https://github.com/Mason-Soule/drone_simulator.git
```

---

### 5. Create the Virtual Environment **Inside the Drone Simulator Folder**

Move into the simulator directory and create the virtual environment there.

```bash
cd drone_simulator
python3 -m venv venv
```

Activate the virtual environment:

* **Windows (PowerShell / CMD)**

  ```bash
  venv\Scripts\activate
  ```

* **Linux / macOS / WSL**

  ```bash
  source venv/bin/activate
  ```

You should now see `(venv)` in your shell prompt.

---

### 6. Install Required Python Packages

pip3 install pymavlink pexpect

---

## Usage

With the virtual environment activated, you can begin running the simulator code and communicating with ArduPilot via MAVLink.

Detailed run instructions and examples will be added as the simulator develops.

---

## Dependencies

* Python 3.9+
* ArduPilot
* pymavlink
* pexpect

---

## Future Work

* Physics-based flight modeling
* Sensor simulation (IMU, GPS, barometer)
* Autonomous mission scripting
* Visualization and telemetry tools

---

## License

This project is provided for educational and experimental purposes.

Refer to the ArduPilot license for A
