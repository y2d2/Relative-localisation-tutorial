# ARP Utility Code

Reusable Python utilities for ARP localization experiments.

`arp_utility_code` contains measurement loaders, simulation helpers, SGal3
filters, and robot/trajectory utilities extracted from ARP and `UPF_RPE`
workflows. The repository uses a `src` package layout.

## Documentation

The full documentation lives in `docs/` and is configured for Sphinx and Read
the Docs.

Build it locally with:

```bash
python -m pip install -e ".[docs]"
python -m sphinx -b html docs docs/_build/html
```

Open `docs/_build/html/index.html` after the build completes.

## Install

Editable install from the repository root:

```bash
python -m pip install -e .
```

For development in this workspace, prefer the shared Python environment:

```bash
/workspace/Python/python_venv/py311_arp/bin/python -m pip install -e .
```

Optional lidar SLAM helpers require:

```bash
python -m pip install -e ".[lidar_slam]"
```

## Package Areas

- `arp_utility_code.Measurements`: ESP, VOXL, MILUV, ROS, Vicon, joint-state,
  and lidar-related measurement readers.
- `arp_utility_code.Simulation`: trajectory generation, simulated sensors,
  robot simulation, and NLOS helpers.
- `arp_utility_code.SGal3_filters`: EKF and UKF implementations for
  `manifpy.SGal3` states.
- `arp_utility_code.UtilityCode`: sensors, trajectories, robot geometry,
  transformation helpers, manifpy helpers, UWB/IMU utilities, and robot-specific
  helpers for Franka and TurtleBot workflows.

## Example Imports

```python
from arp_utility_code.Measurements import EspMeasurement
from arp_utility_code.SGal3_filters.SGal3_EKF import SGal3EKF
from arp_utility_code.Simulation.Sensors_simulation import InterRobotDistanceSensorSimulation
from arp_utility_code.UtilityCode.Sensors import Vio
from arp_utility_code.UtilityCode.utility_fuctions import get_4d_rot_matrix
```

## Quick Examples

Load ESP measurements and convert them to reusable sensor objects:

```python
from arp_utility_code.Measurements import EspMeasurement

measurement = EspMeasurement.from_csv_folder("path/to/esp/folder")
sensors = measurement.to_sensors()
imu = sensors.imu
uwb_by_peer = sensors.uwb_by_id
```

Create a simple simulated UWB range sensor:

```python
from arp_utility_code.Simulation.Sensors_simulation import InterRobotDistanceSensorSimulation

uwb = InterRobotDistanceSensorSimulation(sig_d=0.0)
distance = uwb.get_new_measurement(5.0, time=0.1)
```

Use the shared utility sensors:

```python
import numpy as np
from arp_utility_code.UtilityCode.Sensors import Vio

vio = Vio(np.eye(4), np.zeros(3), time=0.0)
vio.get_new_measurement(None, np.array([1.0, 0.0, 0.0]), np.zeros(3), time=1.0)
position = vio.trajectory.get_current_position()
```

## Tests

Run the default test suite:

```bash
python -m pytest
```

Interactive plotting tests are marked as `manual_plot` and are excluded by
default:

```bash
python -m pytest -m manual_plot -s
```

## Scope Notes

Large simulation datasets, rosbags, notebooks, UPF particle filters,
baseline algorithms, and dataloggers remain outside this repository.

Some legacy simulation paths still refer to UPF-specific algorithm classes.
Those paths raise a clear `ImportError` unless the original `UPF_RPE` package is
available in the environment.
