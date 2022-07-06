import signal
import struct
import sys
import os
import time
import binascii
from panda import Panda

__version__ = '0.0.1'

BASEDIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../")
DEFAULT_FW_FN = os.path.join(BASEDIR, "board", "build", "CanFilter.bin.signed")

CAN_FILTER_ISOTP_RX = 0x2A1
CAN_FILTER_ISOTP_TX = 0x2A9

CAN_FILTER_ADDR_MAGIC = 0x2A0

FEATURE_LOCKSPEED = 4
FEATURE_FAKELEAD  = 8
FEATURE_PASSTHRU  = 16
FEATURE_MIRROR_MSG = 32
FEATURE_SET_DISTANCE= 64

class BootLoaderHandle(object):
  def __init__(self, panda):
    self.panda = panda

  def transact(self, dat):
    self.panda.isotp_send(1, dat, 0, recvaddr=2)

    def _handle_timeout(signum, frame):
      # will happen on reset
      raise Exception("timeout")

    signal.signal(signal.SIGALRM, _handle_timeout)
    signal.alarm(1)
    try:
      #ret = self.panda.isotp_recv(2, 0, sendaddr=1, subaddr=None, bs=1, st=20)
      ret = self.panda.isotp_recv(2, 0, sendaddr=1)
    finally:
      signal.alarm(0)

    return ret

  def controlWrite(self, request_type, request, value, index, data, timeout=0):
    # ignore data in reply, panda doesn't use it
    return self.controlRead(request_type, request, value, index, 0, timeout)

  def controlRead(self, request_type, request, value, index, length, timeout=0):
    dat = struct.pack("HHBBHHH", 0, 0, request_type, request, value, index, length)
    return self.transact(dat)

  def bulkWrite(self, endpoint, data, timeout=0):
    if len(data) > 0x10:
      raise ValueError("Data must not be longer than 0x10")
    dat = struct.pack("HH", endpoint, len(data)) + data
    return self.transact(dat)

  def bulkRead(self, endpoint, length, timeout=0):
    dat = struct.pack("HH", endpoint, 0)
    return self.transact(dat)

class FilterHandle(object):
  def __init__(self, panda):
    self.panda = panda

  def transact(self, dat):
    self.panda.isotp_send(CAN_FILTER_ISOTP_RX, dat, 0, recvaddr=CAN_FILTER_ISOTP_TX)

    def _handle_timeout(signum, frame):
      # will happen on reset
      raise Exception("timeout")

    signal.signal(signal.SIGALRM, _handle_timeout)
    signal.alarm(1)
    try:
      #ret = self.panda.isotp_recv(CAN_FILTER_ISOTP_TX, 0, sendaddr=CAN_FILTER_ISOTP_RX, subaddr=None, bs=1, st=20)
      ret = self.panda.isotp_recv(CAN_FILTER_ISOTP_TX, 0, sendaddr=CAN_FILTER_ISOTP_RX, subaddr=None)
    finally:
      signal.alarm(0)

    return ret

class CanFilter(object):
  @staticmethod
  def reset(panda, enter_bootstub=False, enter_bootloader=False):
    print(f"reset: bootstub {enter_bootstub}, bootloader {enter_bootloader}")
    panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT, True)
    if enter_bootloader:
      panda.can_send(CAN_FILTER_ADDR_MAGIC, b"\xce\xfa\xad\xde\x1e\x0b\xb0\x02", 0)
    else:
      if enter_bootstub:
        panda.can_send(CAN_FILTER_ADDR_MAGIC, b"\xce\xfa\xad\xde\x1e\x0b\xb0\x0a", 0)
      else:
        # reset only
        panda.can_send(CAN_FILTER_ADDR_MAGIC, b"\xce\xfa\xad\xde\x1e\x0b\xb0\x01", 0)

  @staticmethod
  def flash(panda, fw=None):
    # set heartbeat_engaged=True, all controls_allowed will be set to false in 3 seconds
    panda.send_heartbeat(True)

    # set alloutput, disble heartbeat
    panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT, True)

    CanFilter.reset(panda, enter_bootstub=True)
    time.sleep(0.5)

    handle = BootLoaderHandle(panda)
    # confirm flasher is present
    print("flash: check flasher is present")
    fr = handle.controlRead(Panda.REQUEST_IN, 0xb0, 0, 0, 0xc)
    assert fr[4:8] == b"\xde\xad\xd0\x0d"

    # unlock flash
    print("flash: unlocking")
    handle.controlWrite(Panda.REQUEST_IN, 0xb1, 0, 0, b'', timeout=2)

    # erase size
    print("flash: erasing")
    if fw is None:
      fw = DEFAULT_FW_FN
    code = open(fw, "rb").read()
    handle.controlWrite(Panda.REQUEST_IN, 0xb3, len(code), 0, b'', timeout=10)

    # flash over EP2
    STEP = 0x10
    print("flash: flashing {} bytes".format(len(code)))
    for i in range(0, len(code), STEP):
      pct = 100 * i // len(code)
      sys.stdout.write(f"\rflash: {pct}% ")
      handle.bulkWrite(2, code[i:i + STEP], timeout=5)
    sys.stdout.write("\rflash: 100%\n")

    # lock flash
    print("flash: locking")
    handle.controlWrite(Panda.REQUEST_IN, 0xb4, 0, 0, b'', timeout=2)

    # reset
    print("flash: resetting")
    try:
      handle.controlWrite(Panda.REQUEST_IN, 0xd8, 0, 0, b'', timeout=2)
    except Exception:
      pass

  @staticmethod
  def get_signature_from_firmware():
    f = open(DEFAULT_FW_FN, 'rb')
    f.seek(-128, 2)  # Seek from end of file
    return f.read(128)

  @staticmethod
  def get_log(panda):
    ch = FilterHandle(panda)
    dat = b'\x00'
    panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT, True)
    return ch.transact(dat)

  @staticmethod
  def get_signature(panda):
    ch = FilterHandle(panda)
    dat = b'\x01'
    panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT, True)
    return ch.transact(dat)

  @staticmethod
  def get_state(panda):
    ch = FilterHandle(panda)
    dat = b'\x02'
    panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT, True)
    st = ch.transact(dat)
    fmt = [
      ('B', 'cfg_version'),
      ('B', 'crash_state'),
      ('H', 'features'),

      ('4s', 'git_version'),
      ('I', 'uptime_millis'),
      ('I', 'acc_ctrl_timeout'),
      ('I', 'aeb_timeout'),
      ('I', 'pre_colli_timeout'),
      ('I', 'pre_colli_2_timeout'),
      ('I', 'can_rx_errs'),
      ('I', 'can_send_errs'),
      ('I', 'can_rx_cnt'),
      ('I', 'can_tx_cnt'),
      ('I', 'can_txd_cnt'),
      ('I', 'can_err_cnt'),
      ('I', 'can_overflow_cnt'),

      ('B', 'can1_state'),
      ('B', 'can2_state'),
      ('x', 'pad'),
      ('x', 'pad'),
    ]

    # little endian with no padding
    f = '<'
    for x in fmt:
      f = f + x[0]
    r = struct.unpack(f, st)

    i = 0
    d = {}
    for x in fmt:
      if x[0] != 'x':
        if x[1] == 'git_version':
          d[x[1]] = binascii.b2a_hex(r[i]).decode()
        elif x[1] == 'features':
          fl = [
            (FEATURE_LOCKSPEED, "LOCKSPEED"),
            (FEATURE_FAKELEAD, "FAKELEAD"),
            (FEATURE_PASSTHRU, "PASSTHRU"),
            (FEATURE_MIRROR_MSG, "MIRRORMSG"),
            (FEATURE_SET_DISTANCE, "DISTANCEREQ")
          ]

          d[x[1]] = {}
          for f in fl:
            if (f[0] & r[i]) != 0:
              d[x[1]][f[1]] = True
            else:
              d[x[1]][f[1]] = False
        else:
          d[x[1]] = r[i]

        i += 1

    return d

  @staticmethod
  def set_features(panda, features, save):
    v = struct.pack('>BH', save, features)
    d = b"\xce\xfa\xad\xde\x1f" + v
    panda.can_send(CAN_FILTER_ADDR_MAGIC, d, 0)

