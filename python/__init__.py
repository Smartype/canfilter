import signal
import sys
import os
import time
from panda import Panda
from panda import CanHandle

__version__ = '0.0.1'

BASEDIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../")
DEFAULT_FW_FN = os.path.join(BASEDIR, "board", "build", "CanFilter.bin.signed")

CAN_FILTER_ISOTP_RX = 0x2A1
CAN_FILTER_ISOTP_TX = 0x2A9

CAN_FILTER_ADDR_MAGIC = 0x2A0

FEATURE_ACC_CTRL  = 1
FEATURE_ACC_INIT  = 2
FEATURE_LOCKSPEED = 4
FEATURE_FAKELEAD  = 8
FEATURE_PASSTHRU  = 16

class CanFilterHandle(object):

  def __init__(self, p, bus):
    self.p = p
    self.bus = bus

  def transact(self, dat):
    self.p.isotp_send(CAN_FILTER_ISOTP_RX, dat, self.bus, recvaddr=CAN_FILTER_ISOTP_TX)

    def _handle_timeout(signum, frame):
      # will happen on reset
      raise Exception("timeout")

    signal.signal(signal.SIGALRM, _handle_timeout)
    signal.alarm(1)
    try:
      ret = self.p.isotp_recv(CAN_FILTER_ISOTP_TX, self.bus, sendaddr=CAN_FILTER_ISOTP_RX)
    finally:
      signal.alarm(0)

    return ret

class CanFilter(object):

  @staticmethod
  def reset(panda, enter_bootstub=False, enter_bootloader=False):
    # reset
    panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
    if enter_bootloader:
      panda.can_send(CAN_FILTER_ADDR_MAGIC, b"\xce\xfa\xad\xde\x1e\x0b\xb0\x02", 0)
    else:
      if enter_bootstub:
        panda.can_send(CAN_FILTER_ADDR_MAGIC, b"\xce\xfa\xad\xde\x1e\x0b\xb0\x0a", 0)
      else:
        panda.can_send(CAN_FILTER_ADDR_MAGIC, b"\xce\xfa\xad\xde\x1e\x0b\xb0\x01", 0)

  @staticmethod
  def flash(panda):
    CanFilter.reset(panda, enter_bootstub=True)

    time.sleep(0.2)
    code = open(DEFAULT_FW_FN, "rb").read()

    handle = CanHandle(panda, 0)
    # confirm flasher is present
    print("flash: check flasher is present")
    panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
    fr = handle.controlRead(Panda.REQUEST_IN, 0xb0, 0, 0, 0xc)
    assert fr[4:8] == b"\xde\xad\xd0\x0d"
    panda.send_heartbeat()

    # unlock flash
    print("flash: unlocking")
    handle.controlWrite(Panda.REQUEST_IN, 0xb1, 0, 0, b'', timeout=2)
    panda.send_heartbeat()

    # erase size
    print("flash: erasing")
    handle.controlWrite(Panda.REQUEST_IN, 0xb3, len(code), 0, b'', timeout=10)
    panda.send_heartbeat()

    # flash over EP2
    STEP = 0x10
    print("flash: flashing {} bytes".format(len(code)))
    for i in range(0, len(code), STEP):
      pct = 100 * i // len(code)
      sys.stdout.write(f"\rflash: {pct}% ")
      handle.bulkWrite(2, code[i:i + STEP], timeout=5)
      panda.send_heartbeat()
    sys.stdout.write("\rflash: 100%\n")

    # lock flash
    print("flash: locking")
    handle.controlWrite(Panda.REQUEST_IN, 0xb4, 0, 0, b'', timeout=2)
    panda.send_heartbeat()

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
    panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
    ch = CanFilterHandle(panda, 0)
    dat = b'\x00'
    return ch.transact(dat)

  @staticmethod
  def get_signature(panda):
    panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
    ch = CanFilterHandle(panda, 0)
    dat = b'\x01'
    return ch.transact(dat)

  @staticmethod
  def get_state(panda):
    panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
    ch = CanFilterHandle(panda, 0)
    dat = b'\x02'
    st = ch.transact(dat)
    fmt = [
      ('H', 'uptime'),
      ('B', 'crash_state'),
      ('i', 'can1_state'),
      ('i', 'can2_state'),
      ('I', 'acc_ctrl_timeout'),
      ('I', 'aeb_timeout'),
      ('I', 'pre_colli_timeout'),
      ('I', 'pre_colli_2_timeout'),
      ('4s', 'git_version'),
      ('H', 'features'),
      ('I', 'can_rx_errs'),
      ('I', 'can_send_errs'),
      ('I', 'can_rx_cnt'),
      ('I', 'can_tx_cnt'),
      ('I', 'can_txd_cnt'),
      ('I', 'can_err_cnt'),
      ('I', 'can_overflow_cnt'),
    ]

    f = ''
    for x in fmt:
      f = f + x[0]
    r = struct.unpack(f, st)

    i = 0
    d = {}
    while i < len(fmt):
      d[fmt[i][1]] = r[i]
      i =+ 1

    return d

  @staticmethod
  def set_features(panda, features, save):
    panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
    v = struct.pack('>BH', save, features)
    d = b"\xce\xfa\xad\xde\x1f" + v
    p.can_send(CAN_FILTER_ADDR_MAGIC, d, 0)

