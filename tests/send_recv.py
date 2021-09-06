#!/usr/bin/env python

import panda
import time
from threading import Thread

class Receiver(Thread):
  def __init__(self, p):
    Thread.__init__(self)
    self.p = p

  def run(self):
    while True:
      self.p.send_heartbeat()
      c = self.p.can_recv()
      if c:
        print(f"can: {c}")

      l = self.p.serial_read(0)
      if l:
        print(f"log: {l}")
      time.sleep(0.2)

def main():
  p = panda.Panda()
  r = Receiver(p)
  r.start()

  for i in [0, 1]:
    p.set_can_enable(i, True)

  p.set_safety_mode(p.SAFETY_ALLOUTPUT);
  p.set_power_save(0)

  while True:
    for i in [0, 1]:
      p.can_send(100, b'\xab\x12', i)
      time.sleep(0.5)

main()
