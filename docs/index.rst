.. ROS2 Robot Navigation and Exploration System documentation master file

Welcome to ROS2 Robot Navigation and Exploration System's documentation!
======================================================================

.. toctree::
   :maxdepth: 2
   :caption: Contents:

   INSTALLATION
   TUTORIAL
   API
   videos/README

Overview
========

The ROS2 Robot Navigation and Exploration System is an autonomous exploration platform based on multiple advanced algorithms. This system provides:

* **Advanced Path Planning**: Bidirectional A* algorithm with Bezier curve smoothing
* **Intelligent Exploration**: RRT-based exploration point selection
* **Dynamic Navigation**: PID and DWA (Dynamic Window Approach) control
* **Real-time Obstacle Avoidance**: Dynamic mode switching for optimal performance
* **SLAM Integration**: Compatible with various SLAM algorithms

Key Features
============

* **Multi-algorithm Path Planning**: Combines A* path planning with Bezier curve smoothing for optimal trajectories
* **RRT Exploration Strategy**: Uses Rapidly-exploring Random Trees for intelligent exploration point selection
* **Adaptive Control**: Automatically switches between PID and DWA control based on obstacle proximity
* **Safety-first Design**: Comprehensive safety checks and emergency stop mechanisms
* **ROS2 Native**: Built specifically for ROS2 Humble with modern Python practices

Quick Start
===========

1. **Installation**: Follow the :doc:`INSTALLATION` guide to set up your environment
2. **Tutorial**: Check out the :doc:`TUTORIAL` for hands-on examples
3. **API Reference**: Browse the :doc:`API` for detailed function documentation

System Architecture
===================

.. image:: videos/main_demo.mp4
   :alt: System Demo
   :width: 100%

The system consists of several key components:

* **LidarExplorer**: Main coordination node
* **AStarPlanner**: Bidirectional A* path planning
* **DWAPlanner**: Dynamic Window Approach local planning
* **PIDNavigator**: PID motion control
* **RRTExplorationSelector**: RRT-based exploration strategy
* **BezierSmoother**: Path smoothing algorithms

Requirements
============

* **Operating System**: Ubuntu 22.04 LTS
* **ROS2**: Humble Hawksbill
* **Python**: 3.8 or higher
* **Gazebo**: Garden or newer
* **Hardware**: Intel i5/AMD Ryzen 5 or higher, 8GB RAM

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
