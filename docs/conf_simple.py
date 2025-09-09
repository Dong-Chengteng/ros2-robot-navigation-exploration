# Simplified Sphinx configuration for CI builds

import os
import sys

# Add project root to path
sys.path.insert(0, os.path.abspath('..'))

# Project information
project = 'ROS2 Robot Navigation and Exploration System'
copyright = '2024, dong'
author = 'dong'
release = '1.0.0'

# General configuration
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.intersphinx',
    'sphinx.ext.todo',
    'sphinx.ext.viewcode',
    'sphinx.ext.githubpages',
]

# Source file configuration
source_suffix = ['.rst', '.md']
master_doc = 'index'

# Exclude patterns
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# HTML output configuration
html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

# Extension configuration
intersphinx_mapping = {
    'python': ('https://docs.python.org/3/', None),
    'numpy': ('https://numpy.org/doc/stable/', None),
    'ros2': ('https://docs.ros.org/en/humble/', None),
}

# Mock imports for ROS2
autodoc_mock_imports = [
    'rclpy',
    'geometry_msgs',
    'nav_msgs',
    'nav2_msgs',
    'action_msgs',
    'builtin_interfaces',
    'sensor_msgs',
    'std_msgs',
    'cv2',
    'transforms3d',
    'plotly',
    'sklearn',
]

# Todo configuration
todo_include_todos = True
