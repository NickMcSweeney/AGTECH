#!/usr/bin/env python
from std_msgs.msg import Float64
import sys, os
SCRIPTS_PATH = os.path.abspath(os.path.join(sys.path[0] ,"../../../resources"))
sys.path.append(SCRIPTS_PATH)
from getter import Getter

class IrReader(Getter):
