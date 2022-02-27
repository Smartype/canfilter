import signal
import os
from panda import Panda
from panda import CanHandle

__version__ = '0.0.1'

BASEDIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../")
DEFAULT_FW_FN = os.path.join(BASEDIR, "board", "build", "CanFilter.bin.signed")

class CFCanHandle(object):
  def __init__(self, p, bus):
    self.p = p
    self.bus = bus

  def transact(self, dat):
    self.p.isotp_send(0x2A9, dat, self.bus, recvaddr=0x2A1)

    def _handle_timeout(signum, frame):
      # will happen on reset
      raise Exception("timeout")

    signal.signal(signal.SIGALRM, _handle_timeout)
    signal.alarm(1)
    try:
      ret = self.p.isotp_recv(0x2A1, self.bus, sendaddr=0x2A9)
    finally:
      signal.alarm(0)

    return ret

class CanFilter(object):
  ACC_CTRL  = 1
  ACC_INIT  = 2
  LOCKSPEED = 4
  FAKELEAD  = 8
  PASSTHRU  = 16

  @staticmethod
  def reset(panda, enter_bootstub=False, enter_bootloader=False):
    # reset
    panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
    if enter_bootloader:
      panda.can_send(0x2A0, b"\xce\xfa\xad\xde\x1e\x0b\xb0\x02", 0)
    else:
      if enter_bootstub:
        panda.can_send(0x2A0, b"\xce\xfa\xad\xde\x1e\x0b\xb0\x0a", 0)
      else:
        panda.can_send(0x2A0, b"\xce\xfa\xad\xde\x1e\x0b\xb0\x01", 0)

  @staticmethod
  def flash(panda):
    panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
    panda.can_send(0x2A0, b"\xce\xfa\xad\xde\x1e\x0b\xb0\x0a", 0)
    time.sleep(0.2)
    code = open(DEFAULT_FW_FN, "rb").read()
    Panda.flash_can_static(CanHandle(p, 0), code)

  @staticmethod
  def get_signature_from_firmware():
    f = open(DEFAULT_FW_FN, 'rb')
    f.seek(-128, 2)  # Seek from end of file
    return f.read(128)

  @staticmethod
  def get_log(panda):
    panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
    ch = CFCanHandle(panda, 0)
    dat = b'\x00'
    return ch.transact(dat)

  @staticmethod
  def get_signature(panda):
    panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
    ch = CFCanHandle(panda, 0)
    dat = b'\x01'
    return ch.transact(dat)

  @staticmethod
  def get_state(panda):
    panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
    ch = CFCanHandle(panda, 0)
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
      ('4B', 'git_version'),
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
    p.can_send(0x2A0, d, 0)

