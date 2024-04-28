# Evaluation Guideline

## Evaluate in Simulation

- [Record rosbag from simulator](#record-rosbag-from-simulator)
- [Generate csv file for reference poses](#generate-csv-file-for-reference-poses)
- [Run SLAM](#run-slam)

## Evaluate with Rosbags with Pseudo Ground Truth

- [Generate csv file for reference poses](#generate-csv-file-for-reference-poses)
- [Run SLAM](#run-slam)

## Record rosbag from simulator

```bash
rosbag record /odom /scan /reference_localization

# Run simulator with true location messages:
cd $HOME/ut_automata/
./bin/simulator --localize

# Finish rosbag recording (Ctrl+C)
```

## Generate csv file for reference poses

```bash
cd $HOME/cs393r_starter/
./bin/listen_rosbag_pose
rosbag play <rosbag_file> --topics /reference_localization

# After rosbag finishes, stop listener (Ctrl+C)
```

## Run SLAM

First adjust SLAM configs (See [Configure SLAM](#configure-slam))

```bash

cd $HOME/cs393r_starter/
./bin/slam
rosbag play <rosbag_file> --topics /odom /scan

# After rosbag finishes, stop slam to trigger offline optimization.
./bin/stopSlam
```

## Configure SLAM

Configurations are stored in `config/slam.lua`.

- Before running SLAM with rosbags with reference poses (either simulation or pseudo ground truth), you need to adjust initial pose: `initial_node_global_x`, `initial_node_global_y`, `initial_node_global_theta`.

- To change optimization method:
    - Keep offline optimization on (`runOffline = true`)
    - Two csv files will be generated: `optim_before.csv` and `optim_after.csv` after SLAM is stopped (`./bin/stopSlam`)
        - Without online optimization (`runOnline = false`):
            - `optim_before.csv`: odometry only
            - `optim_after.csv`: offline optimization
        - With online optimization (`runOnline = true`):
            - `optim_before.csv`: online optimization
            - `optim_after.csv`: online + offline optimization
