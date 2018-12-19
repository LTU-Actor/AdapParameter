# ROS Services

The standard way of passing data between ROS nodes is ros messages/topics.
However, there is another way to pass data called ROS services. Instead of a
many-to-many communication of topics, services allow for a strong
request-response communication.

This project chose to use services as they are much more robust to things such
as multiple nodes being tuned at once, and they allow for cleaner
implementation.

# build.py

One of the annoyances of working with many separate ROS projects is the
requirement to build every package in a workspace. This requires putting every
package in a separate `./workspace/src` folder, and makes changing into the
project's directory more of a hassle. Further frustration can be had from the
many commands it takes to actually run `roscore`, the `server` and the client
that you want to actually test (all of which may want to be launched in
separate terminals.

As a quality-of-life improvement while working on this project, `build.py` was
created that will automatically create a virtual-workspace within this repo,
automatically build the package in it, and run the specified launch file for
testing. This dramatically improves the cycle time to test a new feature, as
well as making development much less frustration. See [README.md](../README.md)
for instructions on using this script.

In order for this script to work, all of the ROS code is packaged into a
sub-folder, `package`. This then gets symlinked into a `build_ws/src` folder
for compilation. When cloning this repo into an existing workspace, then catkin
is smart enough to find the package sub-folder. In this case, the `build_ws`
folder must be deleted to avoid confusing catkin with duplicate targets.

The simple version of this script can be found in git commit
`e3d143fe59e3d67267baf1f3ba8a1ea0cb5780bd`. In later scripts it becomes much
more complicated to help support graphing and robust killing of ROS components.

# graph.py

In addition to the build script, `graph.py` was added to automatically graph
information about a client while it is being tuned. This dramatically helps
understand what is going on inside a node and the parameter server.

To make this script better, the script only requires that clients print the
information they want shown to `stdout` in CSV form with the first row being
the header.

This script uses a python modules `pandas` and `seaborn` to wrangle the data
and graph it. These are very useful tools that are worth learning.

# Launching ROS components from Python

When working with the early versions of `build.py` and `graph.py`, it becomes
obvious that cleaning up ROS processes from a script takes some work. Initially
the scripts were using process groups and killing the entire process groups
together, but even this was not good enough. Very often, a stuck ROS program or
core would be left hanging around and mess up subsequent executions.

Possibly the simplest way to solve this problem is to use the `psutil` python
module. This is a **very** well supported module that allows a script to query
information about system processes that are not easily available.

The build script will query each of the launched ROS processes for their entire
child-trees, which will will find processes even when ROS changes the
process-group id. Once the scripts are done the process, it will send SIGTERM
to the entire process's child tree to prevent ROS from re-spawning or doing
anything funky. SIGKILL was found to work usually, but not always.

# cmake-sanitizers

On Linux, both gcc and upside clang support a number of sanitizers. Possibly
most importantly is the address sanitizer, that checks for stack overflows, use
after free, and more with almost zero execution speed penalty. Unfortunately,
these arguments are not part of the C++ standard, so compilers all have their
own ways of enabling these features.

To solve this, an optional submodule, `cmake-sanitizers`, was added (cmake will
disable this feature if the submodule is empty). This submodule adds a
cross-platform/cross-compiler way to enable sanitizers whenever supported.

In this project, these sanitizers are only enabled when using the `build.py`
script to avoid any memory overheads when cloning into an external workspace.
This is another good reason to use the build script.

# clang-format

To keep this repo stylistically consistent, a `.clang-format` file exists.
[clang-format](https://clang.llvm.org/docs/ClangFormatStyleOptions.html) is a
tool that will deterministically reformat C/C++ source according to the
formatting file. The tool will search up directories until it finds the format
file.

This tool speeds up development by removing the time spent formatting.

To use this tool, either:
 1. Install a plugin for your editor
 2. Use it from the command line
 3. Use it as a part of git

 `git clang-format` is a series of git commands that will reformat any files
 stages for the commit.
