#! /usr/bin/env python3

import os
import subprocess
import argparse
import time

argparser = argparse.ArgumentParser()
argparser.add_argument(
    '-r',
    '--run',
    dest='run',
    type=str,
    required=False,
    help='Automatically start a .launch file')
args = argparser.parse_args()

root = os.path.dirname(os.path.realpath(__file__))
path_buildws = root + '/build_ws'
path_aplink = path_buildws + '/src/adap_parameter'
path_package = root + '/package'

os.makedirs(path_buildws + '/src', exist_ok=True)

if (not os.path.islink(path_aplink)):
    os.symlink(path_package, path_aplink, True)

subprocess.run(
    'catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DSANITIZE_ADDRESS=1',
    shell=True,
    cwd=path_buildws)

if (not os.path.islink(root + '/compile_commands.json')):
    os.symlink(
        path_buildws + '/build/compile_commands.json',
        root + '/compile_commands.json')

if (args.run):
    try:
        process = subprocess.Popen(
            '. devel/setup.sh && roslaunch adap_parameter ' +
            args.run +
            '.launch',
            shell=True,
            cwd=path_buildws)
        process.wait()
    except KeyboardInterrupt:
        # ROS puts some junk on the terminal after the main process dies. Wait
        # for it.
        process.kill()
        time.sleep(1)
