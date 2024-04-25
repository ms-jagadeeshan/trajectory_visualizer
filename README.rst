
Trajectory Visualizer Documentation
===================================

Files and Functionality
------------------------

**Main Classes** 

- ``trajectory_publisher.cpp``: Publishes visual marker array from pose or by listening to tf.
- ``trajectory_saver.cpp``: Contains a service to get and save the trajectory from transform.
- ``trajectory_reader.cpp``: Contains a service to read and broadcast transform and visual marker.

**Input/Output Handling**

- An abstract class is used for modularity:
  - ``BaseIO``: Abstract base class.
  - ``CsvIO``: Handles CSV input/output.
  - ``JsonIO``: Handles JSON input/output.
  - ``YamlIO``: Handles YAML input/output.
- ``TrajectoryIO``: A facilitator class which can create instances of CsvIO, JsonIO, or YamlIO and store them as a BaseIO pointer.

**Helpers**

- ``transform_listener.cpp``: A wrapper class around tf2 transform listener to get pose.
- ``transform_broadcaster.cpp``: A wrapper class around tf2 transform broadcaster to send transform from pose.

For more details see `here <./DOCS.rst>`

Nodes
-----
- ``trajectory_saver_node``: A merge of ``TrajectorySaver`` (handled by ROS spinner) and ``TrajectoryPublisher`` (runs in the main thread) class.
  Parameters:

    - ``map_frame``: Name of the map frame.
    - ``base_frame``: Name of the base frame.
    - ``publish_rate``: Rate of publishing.

- ``trajectory_reader_visualization_node``: A merge of ``trajectory_reader`` and ``trajectory_publisher`` (sequential, after reading, the publisher will be active).
  Parameters:

    - ``map_frame``: Name of the map frame.
    - ``base_frame``: Name of the base frame.

Services
--------
- ``ReadTrajectory.srv``:

  Request:

    - ``string filename``: Name of the file to load.
    - ``int32 start_time``: Time to start from, -1 for starting from the beginning of the file.
    - ``int32 end_time``: Time to end at, -1 for ending at the end of the file.
    - ``bool loop``: Whether to loop the replay.
  Response:

    - ``bool result``: Indicates success or failure.

- ``SaveTrajectory.srv``:

  Request:

    - ``string filename``: Name of the file to save.
    - ``uint32 duration``: Duration of the trajectory.
  Response:

    - ``bool result``: Indicates success or failure.

3rd-Party Libraries
--------------------
- ``backward-cpp``: For traceback.
- ``yaml-cpp``: For YAML parsing.
- ``nlohmann/json``: For JSON parsing.

Installation
------------

Clone the repository into an existing or new ROS workspace.

.. code-block:: bash

   # Dependency
   sudo apt install libyaml-dev
   mkdir -p catkin_ws/src && cd catkin_ws/src
   git clone https://github.com/ms-jagadeeshan/trajectory_visualizer.git
   catkin init
   catkin build 


Usage
-----

Launches the trajectory saver node.

.. code-block:: bash

   roslaunch trajectory_visualizer trajectory_saver.launch
   rosservice call /save_trajectory <args>


Launches the trajectory reader node.

.. code-block:: bash

   roslaunch  trajectory_visualizer trajectory_reader.launch
   rosservice call /read_trajectory <args>
