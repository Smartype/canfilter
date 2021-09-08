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
        if msg_id == 0x2A1:
          if dat[0] & 0xF0 == 0x00:
            can1_st = (dat[0] & 0xc) >> 2
            can2_st = dat[0] & 0x3
            ver = dat[1:5].hex()
            uptime, = struct.unpack('>H', dat[5:7])
            print(f"[HB] version: {ver}, can1_status: {can1_st}, can2_status: {can2_st}, uptime: {uptime}")
        elif msg_id == 0x2A1 + 1:
          rx_errs, send_errs, rx_cnt, tx_cnt, txd_cnt, err_cnt, csum_err, overflow_cnt = struct.unpack('BBBBBBBB', dat)
          print(f"[ST] rx errs: {rx_errs}, send_errs: {send_errs}, rx_cnt: {rx_cnt}, tx_cnt: {tx_cnt}, txd_cnt: {txd_cnt}, err_cnt: {err_cnt}, scum_err: {csum_err}, overflow_cnt: {overflow_cnt}")
        elif msg_id == 0x2A1 + 2:
          sys.stdout.write('[LG] ' + dat.decode('ascii'))

    time.sleep(0.1)

main()
