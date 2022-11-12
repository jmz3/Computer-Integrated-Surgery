import sys,os
sys.path.append(os.path.dirname(sys.path[0]))
from pathlib import Path
import click
from cispa.Registration import regist_matched_points
from cispa.PivotCalibration import calib_pivot_points
import cispa.DataProcess as DP
import numpy as np
import logging
from rich.logging import RichHandler
from numpy.linalg import inv

