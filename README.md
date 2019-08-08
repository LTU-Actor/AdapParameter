# ROS Adaptable Parameter Server

This package provides a ROS system that allows for automatic tuning of
registered nodes.  The tuning server runs as a separate node, and exposes a
service-based API for client nodes to register through.  This allows for a
separation of components (much of the benefit of ROS), so that the tuning
algorithms can be updated, downloaded, or recompiled without rebuilding all
client nodes.

## Getting Started

This package is intended to be cloned directly into the `src` directory of a
catkin workspace (or a subdirectory within), or worked on standalone using the
python build scripts.

### Prerequisites

To use the provided python build scripts:

```sh
# Only if you want to use ./build.py or ./graph.py
sudo apt-get install python3-tk
python3 -m pip install --user seaborn psutil
```

### Installing / Building into catkin workspace

```sh
cd /path/to/my/workspace
git clone 'https://github.com/LTU-AutoEV/Adaptable-Parameter-Server.git' src/adap_parameter
catkin_make
```

### Installing / Building standalone (for adap_parameter development)
```sh
git clone --recursive 'https://github.com/LTU-AutoEV/Adaptable-Parameter-Server.git'
cd Adaptable-Parameter-Server
./buily.py # build repo, check to make sure compile scripts work
```


## Provided Examples

#### Tuning the threshold of OpenCV's binary threshold the achieve a 50% cut.
```sh
# Using build scripts (see below!)
./build.py -r test_thresh
# OR in a parent workspace
roslaunch adap_parameter test_thresh
```
![thresh desk](doc/assets/thresh_desk.png)

#### Tuning the threshold of OpenCV's canny edge detection to achieve a good
   "edge density."
```sh
# Using build scripts (see below!)
./build.py -r test_canny
# OR in a parent workspace
roslaunch adap_parameter test_canny
```

![canny desk](doc/assets/canny_desk.png)


## Development Tools

**Running these scripts will break this package if it is inside a parrent
workspace**. To fix, delete the `./build_ws` directory.

### build.py

This repository includes a python build script, `build.py`, that will compile
this package inside a subdirectory workspace. It is meant to be used when
developing this package to simplify the work flow.

This will create the subdirectory `./build_ws` with a syslink to `./package`,
then call `catkin_make` inside this directory. If the `-r, --run` argument is
given, it will automatically run `roslaunch` on the file in
`./package/launch/`. It will automatically append the `.launch` suffix if it is
not already supplied.

```
usage: build.py [-h] [-r RUN]

optional arguments:
  -h, --help         show this help message and exit
  -r RUN, --run RUN  Automatically start a .launch file
```

##### Examples:
```sh
./build.py -r test_thresh
./build.py -r test_canny
./build.py -r test_v4l
```

### graph.py

As as extension of `build.py`, this repo includes a script that will
automatically graph information from the included client nodes. These nodes
print information in CSV format to stdout, whis is then graphed with Seaborn
once they exit. If a node does not exit automatically, pressing ^C will stop it
short and graph the output.

This script will automatically run a ros executable found in
`./build_ws/devel/lib/adap_parameter/`. See `./package/CMakeLists.txt` for
their generation.

```
usage: graph.py [-h] -r RUN

optional arguments:
  -h, --help         show this help message and exit
  -r RUN, --run RUN  Automatically start a client
```

##### Examples:
```sh
./graph.py -r client_synth_single
./graph.py -r client_synth_multiple
./graph.py -r client_synth_poly
./graph.py -r client_synth_codependent
```

## Synthetic Tests

![synth-single](doc/assets/synth_single.png)

![synth-multiple](doc/assets/synth_multiple.png)

![synth-poly](doc/assets/synth_poly.png)

![synth-codependent](doc/assets/synth_codependent.png)

## Other Docs

[Requirements Specification](doc/Requirements_Specification.md)

[Technical Tidbits](doc/Technical_Tidbits.md)

[Create a Client](doc/Create_a_Client.md)

