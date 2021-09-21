#!/usr/bin/env python3
import time
import struct
import argparse
from panda import Panda
from panda import CanHandle


if __name__ == "__main__":

  p = Panda()
  p.set_safety_mode(Panda.SAFETY_ALLOUTPUT)

  while 1:
    if len(p.can_recv()) == 0:
      break

  ACC_CTRL      = 1
  ACC_INIT      = 2
  LOCKSPEED     = 4
  FAKELEAD      = 8
  PASSTHRU      = 16

  FEATURES = 0
  FEATURES |= ACC_CTRL
  FEATURES |= ACC_INIT
  FEATURES |= LOCKSPEED
  FEATURES |= FAKELEAD
  #FEATURES |= PASSTHRU

  SAVE = 1
  v = struct.pack('>BH', SAVE, FEATURES) 
  d = b"\xce\xfa\xad\xde\x1f" + v
  print(d.hex())
  p.can_send(0x2A0, d, 0)
  #p.can_send(0x2A0, b"\xce\xfa\xad\xde\x1f\x01\xab\xcd", 0)

