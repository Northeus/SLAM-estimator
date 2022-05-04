# SLAM-estimator
SLAM estimator using gtsam. Configured for snapdragon dataset, which we use.
For now, all settings are embedded into source code.

-------------------------------------------------------------------------------

## Data sources

### IMU
We used IMU data from
[snapdragon dataset #3](https://fpv.ifi.uzh.ch/datasets/) (`imu.txt`),
which have to be placed in `data` folder and trimed, so there are only
a few measurements before first image measurement.

### Visual measurements
We have generated our own visual measurements via
[SLAM-generator](https://github.com/Northeus/SLAM-generation)
(`projections.csv`), which have to be placed in `data` folder.

-------------------------------------------------------------------------------

## Output
Estimator store last best position into `positions.csv`, where first three
numbers represent position and last two nubers represent rotation in quaternion
(`w x y z`), all numbers are separated with space.

-------------------------------------------------------------------------------

## How to run
Make sure you have installed `GTSAM` library.
Then build the program using `cmake .. && make` in the `build` folder.
After that you can run the estimator using `./estimator`.
Output will be stored into `positions.csv`.

