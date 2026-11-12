# user_interface

A RAYLIB based user interface for DaVE.

This package consists of 2 major parts

1) **The UI**

- The UI will be a simple C++ program that will not have any ros2 functionality
- It will handle reading input and displaying information about the ROVER
- The UI has been separated from the ROS2 Node that it relies on so that we didn't have any timing issues between
our drawing loop and all of our ros timers for topics and whatnot

2) **The Launch Node**

- The Launch node will have 2 jobs
- It will communicate with all the other nodes and relay that information to the UI
- It will launch/terminate other nodes at the request of the UI by using launch scripts

The UI and the Launch node will be running on the same machine but in two separate threads. They will communicate via a singleton that will contain all the data that the UI is to display as well as a queue of requests for the launch node to process

## Some design notes

### The Game Loop

main.cpp contains our "game loop". This is the loop responsible for drawing each frame on screen.

the game loop can be split into 2 parts

1) **The Update Part**
In the first part of the loop, we must:
- Take in the input from the user
- send the user's request to the launch node (if applicable)
- Read the data stored in the shared singleton
- Update the data/state of each display accordingly

2) **The Drawing Part**
In the second part of the loop, we must:
- create a new frame
- Draw the textures and text related with each component (instrument) onto the new frame
- Display the new frame on screen

### The Displays

Our current design was inspired by the instrument panels you'd see in aircraft

It will be made up of several "instruments" which will all have their own area onscreen, some kind of a control (like a button or a slider) and a display (possibly an indicator light or a small internal screen).
These instruments can be positioned anywhere on screen.

Some Instruments we are currently thinking about
* A simple Text display
* A Simple MFD (Multi-Function Display)
    - a display only instrument
    - It would have several buttons that would allow you to switch what information is displayed
    - example use cases:
        * A compass
        * spedometer
        * rotational data of DaVE
* MFD (Multi-Function Display)
    - like the simple MFD, it can display all sorts of stuff
    - it can interact with the display
    - example use cases:
        - Navigation -> buttons to create waypoins, zoom in/out, etc
* Warning Panel
    - would feature a bunch of warning lights that will flash for certain failures
    - example warning lights:
        * Excessive Gs
        * Low battery
        * Battery is overheating
* Terminal
    - a linux like terminal to manually call commands
* Launch menu
    - a list of all active nodes
    - buttons to allow for us to launch/kill specific nodes

## Building and Testing information

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
