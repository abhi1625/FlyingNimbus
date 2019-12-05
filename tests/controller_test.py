import unittest 
import sys 
import math 
import numpy as np
sys.path.append("../src/")
from Controller import Controller

  
class ControllerTest(unittest.TestCase): 
    # Returns True or False.  
    def test(self):       
        ctrl = Controller()
        out = ctrl.gen_ctrl_inputs(math.pi/6, [math.sqrt(3), 1])
        print out
        self.assertEqual(2.0, out[0]) 
  
if __name__ == '__main__': 
    unittest.main() 