# user_interface

The UI target fetches a pinned raylib revision with CMake FetchContent and builds
it as a static library with raylib's bundled GLFW. Teammates do not need to
install raylib separately. The first CMake configure requires Git and internet
access; the fetched source is cached in the workspace build directory.

The target system still needs the native graphics and window-system libraries
required by raylib for its platform.

Build the package with:

```sh
colcon build --packages-select user_interface
```

This package has an auto mated test suite

Run the tests with:

```sh
colcon test --packages-select user_interface
```

view test results with:

```sh
colcon test-result --all
```

or

```sh
colcon test-result --all --verbose
```

This package consists of 2 major parts

1) **The UI**

- The UI will be a simple C++ program that will no have any ros2 functionality
- It will handle reading input and displaying information about the ROVER
- The UI has been separated from the ROS2 Node that it relies on so that we didn't have any timing issues between
our drawing loop and all of our ros timers for topics and whatnot

2) **The Launch Node**

- The Launch node will have 2 jobs
- It will communicate with all the other nodes and relay that information to the UI
- It will launch/terminate other nodes at the request of the UI by using launch scripts

The UI and the Launch node will be running on the same machine but in two separate threads. They will communicate via a single ton that will contain all the data that the UI is to display as well as a queue of requests for the launch node to process