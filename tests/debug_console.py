#!/usr/bin/env python

import panda
import time
import sys

def main():
  p = panda.Panda()
  p.set_can_enable(0, True)
  p.set_safety_mode(p.SAFETY_ALLOUTPUT);
  p.set_power_save(0)

  while True:
    p.send_heartbeat()
    c = p.can_recv()
    if c:
      for m in c:
        msg_id, x, dat, bus = m
        if msg_id == 0x2A1 and dat[0] == 0x10:
          sys.stdout.write(dat[1:].decode('ascii'))

    time.sleep(0.1)

main()
