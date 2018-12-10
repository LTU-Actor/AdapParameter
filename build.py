#! /usr/bin/env python3

import os
import subprocess
import argparse
import time
import atexit
import signal
import psutil

path_root = os.path.dirname(os.path.realpath(__file__))
path_buildws = path_root + '/build_ws'
path_aplink = path_buildws + '/src/adap_parameter'
path_package = path_root + '/package'
path_direct_launch = './devel/lib/adap_parameter/'
roscore = None


def signal_recursive(process, sig=signal.SIGTERM):
    try:
        parent = psutil.Process(process.pid)
    except psutil.NoSuchProcess:
        return
    children = parent.children(recursive=True)
    for c in children:
        print("kill process: " + str(c))
        c.send_signal(sig)
    process.send_signal(sig)


def roscore_start():
    global roscore
    roscore = subprocess.Popen(
        '. devel/setup.sh && roscore',
        shell=True,
        cwd=path_buildws,
        universal_newlines=True,
        stdout=subprocess.DEVNULL,
        stderr=None,
        start_new_session=True)
    time.sleep(1)


def roscore_stop():
    global roscore
    if(not (roscore is None)):
        signal_recursive(roscore)
        roscore.wait()
        roscore = None


def make_build_ws():
    os.makedirs(path_buildws + '/src', exist_ok=True)
    if (not os.path.islink(path_aplink)):
        os.symlink(path_package, path_aplink, True)


def link_compile_cmds():
    if (not os.path.islink(path_root + '/compile_commands.json')):
        os.symlink(
            path_buildws + '/build/compile_commands.json',
            path_root + '/compile_commands.json')


def catkin_make():
    make_build_ws()
    subprocess.run(
        'catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DSANITIZE_ADDRESS=1',
        shell=True,
        cwd=path_buildws)


def build():
    catkin_make()
    link_compile_cmds()


def launch_launchfile(name):
    if (not name.endswith('.launch')):
        name += '.launch'

    return subprocess.Popen(
        '. devel/setup.sh && roslaunch adap_parameter ' +
        name,
        shell=True,
        cwd=path_buildws,
        universal_newlines=True,
        stdout=subprocess.DEVNULL,
        stderr=None,
        start_new_session=True)


def launch_direct(name):
    return subprocess.Popen(
        '. devel/setup.sh && ' +
        path_direct_launch + name,
        shell=True,
        cwd=path_buildws,
        universal_newlines=True,
        stdout=subprocess.PIPE,
        stderr=None,
        start_new_session=True)


def waiton(proc):
    try:
        print('Waiting on proc... ^C to kill it an continue.')
        proc.wait()
    except KeyboardInterrupt:
        signal_recursive(proc, signal.SIGKILL)
        time.sleep(1)
    finally:
        print('Done waiting on proc.')


def kill_node(node):
    signal_recursive(node)
    node.wait()


def main():
    argparser = argparse.ArgumentParser()
    argparser.add_argument(
        '-r',
        '--run',
        dest='run',
        type=str,
        required=False,
        help='Automatically start a .launch file')
    args = argparser.parse_args()

    build()

    if(args.run):
        waiton(launch_launchfile(args.run))


atexit.register(roscore_stop)
if __name__ == "__main__":
    main()
