# MPC Navigation Tracker

## Installation

1. Clone this repository into the `src` folder of your active ros2 workspace:

   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/manu68-debugs/mpc_nav.git
   ```

2. Build the workspace :

   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   ```

3. Source the environment:
   ```bash
   source install/setup.bash
   ```

## Usage

Launch

```bash
ros2 launch mpc_nav launch_mpc.launch.py
```
[VIDEO](https://drive.google.com/file/d/1nSRdwurPM72rQvlXO0zevIfBaZK28wCv/view?usp=sharing)