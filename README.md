# ROSBAG2CSV

## CSV2ROSbag
Convert multiple trajectory CSV files into a bag file. 

```commandline
aaunav_data_analysis_py/rosbag2csv$ python CSV2ROSbag.py -h 
usage: CSV2ROSbag.py [-h] [--bagfile_name BAGFILE_NAME]
                     [--topics [TOPICS [TOPICS ...]]]
                     [--filenames [FILENAMES [FILENAMES ...]]]
                     [--fmt_list [{GEOMETRY_MSGS_POINTSTAMPED,GEOMETRY_MSGS_POSE,GEOMETRY_MSGS_POSESTAMPED,GEOMETRY_MSGS_POSEWITHCOVARIANCE,GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED,GEOMETRY_MSGS_QUATERNION,GEOMETRY_MSGS_QUATERNIONSTAMPED,GEOMETRY_MSGS_TRANSFORM,GEOMETRY_MSGS_TRANSFORMSTAMPED,GEOMETRY_MSGS_VECTOR3,GEOMETRY_MSGS_VECTOR3STAMPED,NOT_SUPPORTED} [{GEOMETRY_MSGS_POINTSTAMPED,GEOMETRY_MSGS_POSE,GEOMETRY_MSGS_POSESTAMPED,GEOMETRY_MSGS_POSEWITHCOVARIANCE,GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED,GEOMETRY_MSGS_QUATERNION,GEOMETRY_MSGS_QUATERNIONSTAMPED,GEOMETRY_MSGS_TRANSFORM,GEOMETRY_MSGS_TRANSFORMSTAMPED,GEOMETRY_MSGS_VECTOR3,GEOMETRY_MSGS_VECTOR3STAMPED,NOT_SUPPORTED} ...]]]
                     [--result_dir RESULT_DIR] [--verbose]

CSV2ROSbag: read CSV files and convert lines to specified ROS msg and store
them into a rosbag

optional arguments:
  -h, --help            show this help message and exit
  --bagfile_name BAGFILE_NAME
                        name of bag file (no path!)
  --topics [TOPICS [TOPICS ...]]
                        topics to create
  --filenames [FILENAMES [FILENAMES ...]]
                        csv filename of corresponding topic
  --fmt_list [{GEOMETRY_MSGS_POINTSTAMPED,GEOMETRY_MSGS_POSE,GEOMETRY_MSGS_POSESTAMPED,GEOMETRY_MSGS_POSEWITHCOVARIANCE,GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED,GEOMETRY_MSGS_QUATERNION,GEOMETRY_MSGS_QUATERNIONSTAMPED,GEOMETRY_MSGS_TRANSFORM,GEOMETRY_MSGS_TRANSFORMSTAMPED,GEOMETRY_MSGS_VECTOR3,GEOMETRY_MSGS_VECTOR3STAMPED,NOT_SUPPORTED} [{GEOMETRY_MSGS_POINTSTAMPED,GEOMETRY_MSGS_POSE,GEOMETRY_MSGS_POSESTAMPED,GEOMETRY_MSGS_POSEWITHCOVARIANCE,GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED,GEOMETRY_MSGS_QUATERNION,GEOMETRY_MSGS_QUATERNIONSTAMPED,GEOMETRY_MSGS_TRANSFORM,GEOMETRY_MSGS_TRANSFORMSTAMPED,GEOMETRY_MSGS_VECTOR3,GEOMETRY_MSGS_VECTOR3STAMPED,NOT_SUPPORTED} ...]]
                        CSV format
  --result_dir RESULT_DIR
                        directory to store results [otherwise bagfile name
                        will be a directory]
  --verbose

```

#### Example
```commandline
aaunav_data_analysis_py/rosbag2csv$ python CSV2ROSbag.py --bagfile_name dummy.bag --topics /pose_est /pose_gt --filenames ../sample_data/ID1-pose-est-cov.csv ../sample_data/ID1-pose-gt.csv --fmt_list GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED GEOMETRY_MSGS_POSESTAMPED --verbose
```

## ROSbag2CSV

Convert a ROS bagfile into multipe CSV files. 
```commandline
aaunav_data_analysis_py/rosbag2csv$ python ROSbag2CSV.py -h
usage: ROSbag2CSV.py [-h] [--bagfile BAGFILE] [--topics [TOPICS [TOPICS ...]]]
                     [--filenames [FILENAMES [FILENAMES ...]]]
                     [--result_dir RESULT_DIR] [--verbose]
                     [--format {PoseCov,PoseWithCov,TUM,TUM_short,none}]

ROSbag2CSV: extract and store given topics of a rosbag into a CSV file

optional arguments:
  -h, --help            show this help message and exit
  --bagfile BAGFILE     input bag file
  --topics [TOPICS [TOPICS ...]]
                        desired topics
  --filenames [FILENAMES [FILENAMES ...]]
                        csv filename of corresponding topic
  --result_dir RESULT_DIR
                        directory to store results [otherwise bagfile name
                        will be a directory]
  --verbose
  --format {PoseCov,PoseWithCov,TUM,TUM_short,none}
                        CSV format
```

#### Example

```commandline
aaunav_data_analysis_py/rosbag2csv$ python ROSbag2CSV.py --bagfile ../sample_data/dummy.bag --topics /pose_est /pose_gt --verbose --filename ../sample_data/NEW-ID1-pose-est-cov.csv ../sample_data/NEW-ID1-pose-gt.csv --format PoseWithCov
```