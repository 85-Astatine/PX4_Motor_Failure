
# PX4 Repository

This repository contains code and configuration files to work with the PX4 Autopilot for UAVs in simulation using Gazebo. The following instructions will guide you through the requirements, setup, and basic usage, including simulating motor failure.

---

## Requirements

Before setting up the project, ensure that you have the following tools and dependencies installed:

- **PX4 Autopilot**: Follow the [official PX4 installation guide](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html) to set up the PX4 environment.
- **Gazebo**: For simulation. Install it using:
  ```bash
  sudo apt install gazebo
  ```
- **ROS 2 Humble** (optional, if you need ROS integration with PX4)
- **MAVSDK or MAVROS** (optional, for interfacing with PX4)

## Initial Setup

1. Download and unzip:
   ```bash
   cd PX4-Autopilot
   ```

2. Install PX4 dependencies:
   ```bash
   ./Tools/setup/ubuntu.sh
   ```

3. Build the PX4 firmware with Gazebo support:
   ```bash
   make px4_sitl gazebo
   ```

   This will compile PX4 and open the Gazebo simulation environment.

---

## Running the Simulation

To launch the simulation in Gazebo:

```bash
make px4_sitl gazebo
```

This command initializes PX4 and starts the Gazebo simulation.

---

## Simulating Motor Failure

To simulate a motor failure, use the following `gz topic` command in a new terminal:

```bash
gz topic -p /gazebo/motor_failure_num --msg "data: 1"
```

This command sets motor 1 to fail. You can change the `data` field to specify different motor numbers.

---

## Additional Tools

Here are some additional tools that might be useful for working with PX4 and Gazebo:

- **QGroundControl**: For real-time monitoring and controlling UAVs in PX4. Download from the [QGroundControl website](https://qgroundcontrol.com/).
- **jMAVSim** (optional): If you prefer a lightweight simulator over Gazebo.
- **MAVLink Inspector**: For viewing MAVLink messages in real-time.

---

## Useful Links

- [PX4 Autopilot Documentation](https://docs.px4.io/)
- [Gazebo Documentation](http://gazebosim.org/)
