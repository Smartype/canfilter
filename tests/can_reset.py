#!/usr/bin/env python3
import time
import argparse
from panda import Panda
from panda import CanHandle

if __name__ == "__main__":
  p = Panda()
  p.set_safety_mode(Panda.SAFETY_ALLOUTPUT)

  while 1:
    if len(p.can_recv()) == 0:
      break

  p.can_send(0x2A0, b"\xce\xfa\xad\xde\x1e\x0b\xb0\x01", 0)
  print("can reset done")

