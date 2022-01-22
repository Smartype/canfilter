#!/usr/bin/env python

import panda
import time
import sys
import struct

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
        if msg_id == 0x2A8:
            pass
        elif msg_id == 0x2A9:
          rx_errs, send_errs, rx_cnt, tx_cnt, txd_cnt, err_cnt, csum_err, overflow_cnt = struct.unpack('BBBBBBBB', dat)
          print(f"[ST] rx errs: {rx_errs}, send_errs: {send_errs}, rx_cnt: {rx_cnt}, tx_cnt: {tx_cnt}, txd_cnt: {txd_cnt}, err_cnt: {err_cnt}, scum_err: {csum_err}, overflow_cnt: {overflow_cnt}")
        elif msg_id == 0x2AA:
          sys.stdout.write('[LG] ' + dat.decode('ascii'))

    time.sleep(0.1)

main()
