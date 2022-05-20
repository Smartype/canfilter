#!/usr/bin/env python3
import sys
import time
import struct
import argparse
from panda import Panda
from canfilter import CanFilter

if __name__ == "__main__":

  p = Panda()
  p.set_safety_mode(Panda.SAFETY_ALLOUTPUT)

  while 1:
    if len(p.can_recv()) == 0:
      break

  st = CanFilter.get_state(p)
  print(st)
