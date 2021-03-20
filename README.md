#  cnspy_rosbag2csv -- Package

A package to convert different [ROS1] messages for **POSES**. Messages contained in rosbag files can be converted into CSV files in different output formats using [ROSBAG2CSV](./cnspy_rosbag2csv/ROSbag2CSV.py). 
The other way round is supported by the class [CSV2ROSbag](./cnspy_rosbag2csv/CSV2ROSbag.py), taking multiple CSV files and writing them into a single bag file. 

The supported ROS1 message types are defined in [ROSMessageTypes](./cnspy_rosbag2csv/ROSMessageTypes.py) (basically all ROS1 [geometry_msgs](http://docs.ros.org/melodic/api/geometry_msgs/html)). 


The package is easily extendable ([Open-Closed-Principle](https://en.wikipedia.org/wiki/Open%E2%80%93closed_principle)). Just create a new [ROSMessageType](./cnspy_rosbag2csv/ROSMessageTypes.py), and make an addition to
 * [ROSMsg2CSVLine](./cnspy_rosbag2csv/ROSMsg2CSVLine.py)
 * [CSVLine2ROSMsg](./cnspy_rosbag2csv/CSVLine2ROSMsg.py) 
 
 The supported CSV formats for poses are defined in the enum [CSVFormatPose](https://github.com/aau-cns/cnspy_spatial_csv_formats/cnspy_spatial_csv_formats/CSVFormatPose.py) of the package [cnspy_spatial_csv_formats](https://github.com/aau-cns/cnspy_spatial_csv_formats). 
 
## Installation

Install the current code base from GitHub and pip install a link to that cloned copy
```
git clone https://github.com/aau-cns/cnspy_rosbag2csv.git
cd cnspy_rosbag2csv
pip install -e .
```

## Dependencies

It is part of the [cnspy eco-system](https://github.com/aau-cns/cnspy_eco_system_test) of the [cns-github](https://github.com/aau-cns) group.  
Main dependencies are:

* [PyYAML]()
* [tqdm]()
* [rospy (ROS)]()
* [spatialmath-python](https://github.com/petercorke/spatialmath-python)
* [cnspy_script_utils](https://github.com/aau-cns/cnspy_script_utils)
* [cnspy_spatial_csv_formats](https://github.com/aau-cns/cnspy_spatial_csv_formats)



## [CSV2ROSbag](./CSV2ROSbag.py)
Convert multiple trajectory CSV files into a bag file. 

```commandline
rosbag2csv$ python CSV2ROSbag.py -h 
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
rosbag2csv$ python CSV2ROSbag.py --bagfile_name dummy.bag --topics /pose_est /pose_gt --filenames ../sample_data/ID1-pose-est-cov.csv ../sample_data/ID1-pose-gt.csv --fmt_list GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED GEOMETRY_MSGS_POSESTAMPED --verbose
```

## [ROSBAG2CSV](./ROSbag2CSV.py)

Convert a ROS bagfile into multipe CSV files. 
```commandline
rosbag2csv$ python ROSbag2CSV.py -h
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
rosbag2csv$ python ROSbag2CSV.py --bagfile ../sample_data/dummy.bag --topics /pose_est /pose_gt --verbose --filename ../sample_data/NEW-ID1-pose-est-cov.csv ../sample_data/NEW-ID1-pose-gt.csv --format PoseWithCov
```

## License

Software License Agreement (GNU GPLv3  License), refer to the LICENSE file.

*Sharing is caring!* - [Roland Jung](https://github.com/jungr-ait)  
