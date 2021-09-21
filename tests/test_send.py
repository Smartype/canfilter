#!/usr/bin/env python

import panda
import time
from threading import Thread

class Receiver(Thread):
  def __init__(self, p):
    Thread.__init__(self)
    self.p = p
    self.count = { 0:0, 1:0, 128:0, 129:0 }

  def run(self):
    while True:
      self.p.send_heartbeat()
      c = self.p.can_recv()
      if c:
        #print(f"can: {c}")
        for m in c:
          addr, _, dat, bus = m
          self.count[bus] += 1
          if self.count[bus] > 1000:
            self.count[bus] = 0
            print(f"{bus}: {time.time()}")

      l = self.p.serial_read(0)
      if l:
        print(f"log: {l}")
      time.sleep(0.2)

def main():
  p = panda.Panda()
  r = Receiver(p)
  r.start()

  for i in [0, 1, 2]:
    p.set_can_enable(i, False)

  time.sleep(0.5)
  for i in [0, 1, 2]:
    p.set_can_enable(i, True)

  p.set_safety_mode(p.SAFETY_ALLOUTPUT);
  p.set_power_save(0)

  while True:
    #for i in [0, 1, 2]:
    #for i in [1]:
    for i in [0, 1]:
      #p.can_send(100, f"bus: {i}".encode('ascii'), i)
      p.can_send_many([[100, None, b'\x00\x01\x02\x03\x04\x05\x06\x07', i]] * 5)
      time.sleep(0.005)

main()
