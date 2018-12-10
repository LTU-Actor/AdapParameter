# ROS Self Adapting Parameter Server

## Usage

This package is intended to be cloned directly into the `src` directory of a
catkin workspace (or a subdirectory within).

### ./build.py (for standalone development)

**Running this script will break this package if it is inside a parrent
workspace**. To fix, delete the `build_ws` directory.

This repo includes a python build script, `build.py`, that will compile this
package inside a subdirectory workspace. It is ment to be used when developing
this package to simplify the workflow.

##### To install psutil (required):

```text
python3 -m pip install --user psutil
```

##### To use this script:

```text
usage: build.py [-h] [-r RUN]

optional arguments:
  -h, --help         show this help message and exit
  -r RUN, --run RUN  Automatically start a .launch file
```

This will create the subdirectory `./build_ws` with a syslink to `./package`,
then call `catkin_make` inside this directory. If the `-r, --run` argument is
given, it will automatically run `roslaunch` on the file in
`./package/launch/`. It will automatically append the `.launch` suffix if it is
not already supplied.

### ./graph.py

As as extention of `build.py`, this repo includes a script that will
automatically graph information from the included client nodes. These nodes
print information in CSV format to stdout, whis is then graphed with seaborn
once they exit. If a node does not exit automatically, pressing ^C will stop it
short and graph the output.

##### To install required python modules:
```text
sudo apt-get install python3-tk
python3 -m pip install --user seaborn psutil
```

##### To use this script:

```text
usage: graph.py [-h] -r RUN

optional arguments:
  -h, --help         show this help message and exit
  -r RUN, --run RUN  Automatically start a client
```

## Other Docs

[Requirements Specification](doc/Requirements%20Specification.md)

[Technical Tidbits](doc/Technical%20Tidbits.md)

[Create a Client](doc/Create%20a%20Client.md)
