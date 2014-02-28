__author__ = "Pleiades Robotics: Nicholas Othieno"
__email__  = "info@pleiades.ca"

import sys
if sys.version_info < (2,7,4,'final',0):
  from spiri_py27 import *
else:
  from spiri_py3x import * #Python 3.x is backported to 2.7.4. At least for all the functionality We use