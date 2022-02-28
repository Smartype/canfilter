#!/usr/bin/env python3
import sys
import time
import struct
import argparse
from panda import Panda
from panda import CanHandle

ACC_CTRL  = 1
ACC_INIT  = 2
LOCKSPEED = 4
FAKELEAD  = 8
PASSTHRU  = 16

if __name__ == "__main__":

  if len(sys.argv) < 3:
    print(f'''usage:
    {sys.argv[0]} <features> <save-flash>

ACC_CTRL  = 1
ACC_INIT  = 2
LOCKSPEED = 4
FAKELEAD  = 8
PASSTHRU  = 16''')
    sys.exit(1)

  FEATURES = int(sys.argv[1])
  SAVE = bool(int(sys.argv[2]))

  f_str = ''
  if FEATURES & ACC_CTRL:
    f_str += "acc "
  if FEATURES & ACC_INIT:
    f_str += "init "
  if FEATURES & LOCKSPEED:
    f_str += "speed "
  if FEATURES & FAKELEAD:
    f_str += "lead "
  if FEATURES & PASSTHRU:
    f_str += "passthru"

  print(f"features: {f_str}, save: {SAVE}")

  p = Panda()
  p.set_safety_mode(Panda.SAFETY_ALLOUTPUT)

  while 1:
    if len(p.can_recv()) == 0:
      break

  v = struct.pack('>BH', SAVE, FEATURES)
  d = b"\xce\xfa\xad\xde\x1f" + v
  print(d.hex())
  p.can_send(0x2A0, d, 0)
  #p.can_send(0x2A0, b"\xce\xfa\xad\xde\x1f\x01\xab\xcd", 0)

