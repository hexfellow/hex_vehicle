# Adding HEXFELLOW APT Software Repository Guide
## Overview
This guide will help you to add the HEXFELLOW APT software repository to your system. This is necessary to install the latest version of the software.

## Prerequisites
- **Ubuntu 20.04 LTS (Focal Fossa)** or **Ubuntu 22.04 LTS (Jammy Jellyfish)** or **Ubuntu 24.04 LTS (Noble Numbat)**
- **ROS 1 Noetic** or **ROS 2 Humble** or **ROS 2 Jazzy**

## Steps
1. Must be root user or sudo, run the following command:
   ```bash
   chmod +x add_hexfellow_apt_source.py && sudo ./add_hexfellow_apt_source.py
   ```

2. Update the package list:
   ```bash
   sudo apt update
   ```

3. Install the software:
   ```bash
   sudo apt install ros-<distro>-xpkg-bridge
   ```
