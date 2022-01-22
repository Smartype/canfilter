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
          crash_state = dat[0] >> 6
          can2_st = (dat[0] >> 3) & 0x7
          can2_st = dat[0] & 0x7
          ver = dat[1:5].hex()
          uptime, = struct.unpack('>H', dat[5:7])
          acc_timeout = dat[7] & 0x1f
          aeb_to = ((dat[7] >> 5) & 1)
          pc_to = ((dat[7] >> 6) & 1)
          pc2_to = ((dat[7] >> 7) & 1)
          print(f"[HB] crash_state: {crash_state}, version: {ver}, can1_status: {can1_st}, can2_status: {can2_st}, uptime: {uptime}, aeb_to: {aeb_to}, pc_to: {pc_to}, pc2_to: {pc2_to}")

        elif msg_id == 0x2A9:
          rx_errs, send_errs, rx_cnt, tx_cnt, txd_cnt, err_cnt, csum_err, overflow_cnt = struct.unpack('BBBBBBBB', dat)
          print(f"[ST] rx errs: {rx_errs}, send_errs: {send_errs}, rx_cnt: {rx_cnt}, tx_cnt: {tx_cnt}, txd_cnt: {txd_cnt}, err_cnt: {err_cnt}, scum_err: {csum_err}, overflow_cnt: {overflow_cnt}")

        elif msg_id == 0x2AA:
          sys.stdout.write('[LG] ' + dat.decode('ascii'))

    time.sleep(0.1)

main()
