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

def send_PCM_CRUISE_2_1(p):
  p.can_send(0x1D3, b'\x08\x28\x00\x00\x00\x00\x00\x0C', 0)

def send_PCM_CRUISE_2_3(p):
  p.can_send(0x1D3, b'\x08\x68\x00\x00\x00\x00\x00\x4C', 0)

def send_ACC_CONTROL_1(p):
  p.can_send(0x343, b'\x03\x4e\x63\xc0\x00\x00\x00\xc2', 1)

def send_ACC_CONTROL_2(p):
  p.can_send(0x343, b'\x03\x4e\xa3\xc0\x00\x00\x00\xc2', 1)

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
    #  p.can_send(100, f"bus: {i}".encode('ascii'), i)
    #send_PCM_CRUISE_2_3(p)
    send_PCM_CRUISE_2_1(p)

    send_ACC_CONTROL_1(p)
    #send_ACC_CONTROL_2(p)

    time.sleep(0.5)

main()
