# SLAM-estimator
SLAM estimator using gtsam. Configured for snapdragon dataset, which we use.
For now, all settings are embedded into source code.

-------------------------------------------------------------------------------

## Data sources

## IMU
We used IMU data from
[snapdragon dataset #3](https://fpv.ifi.uzh.ch/datasets/) (`imu.txt`),
which have to be placed in `data` folder.

## Visual measurements
We have generated our own visual measurements via
[SLAM-generator](https://github.com/Northeus/SLAM-generation)
(`projections.csv`), which have to be placed in `data` folder.
We further require to have some imu measurements before the first visual
measurement.

-------------------------------------------------------------------------------

## Output

Estimator store last best position into `positions.csv`.
