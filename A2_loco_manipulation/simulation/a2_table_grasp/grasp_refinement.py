"""Compatibility import; the maintained ROS package owns the shared strategy."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/"src/manipulation/grasp_generation"))
from grasp_generation.refinement import *
