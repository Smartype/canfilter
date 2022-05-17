#!/usr/bin/env python3
import time
import argparse
from panda import Panda
from canfilter import CanFilter

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='Flash can-filter over can')
  parser.add_argument('--recover', action='store_true')
  parser.add_argument("fn", type=str, nargs='?', help="flash file")
  args = parser.parse_args()

  p = Panda()
  p.set_safety_mode(Panda.SAFETY_ALLOUTPUT)

  while 1:
    if len(p.can_recv()) == 0:
      break

  if args.recover:
    p.can_send(0x2A0, b"\xce\xfa\xad\xde\x1e\x0b\xb0\x02", 0)
    exit(0)
  else:
    p.can_send(0x2A0, b"\xce\xfa\xad\xde\x1e\x0b\xb0\x0a", 0)

  if args.fn:
    time.sleep(1)
    print("flashing", args.fn)
    CanFilter.flash(p, args.fn)

  print("can flash done")

