import struct
import signal


class CanHandle(object):
  def __init__(self, p, bus):
    self.p = p
    self.bus = bus

  def panda(self):
    return self.p

  def transact(self, dat, timeout):
    #print(f"isotp send {dat}")
    self.p.isotp_send(1, dat, self.bus, recvaddr=2, rate=None)

    def _handle_timeout(signum, frame):
      # will happen on reset
      raise Exception("timeout")

    signal.signal(signal.SIGALRM, _handle_timeout)
    if timeout == 0:
      timeout = 5
    signal.alarm(timeout)

    #print(f"isotp recv")
    try:
      ret = self.p.isotp_recv(2, self.bus, sendaddr=1)
      #print(f"isotp recvd")
    finally:
      signal.alarm(0)

    return ret

  def controlWrite(self, request_type, request, value, index, data, timeout=0):
    # ignore data in reply, panda doesn't use it
    return self.controlRead(request_type, request, value, index, 0, timeout)

  def controlRead(self, request_type, request, value, index, length, timeout=0):
    dat = struct.pack("HHBBHHH", 0, 0, request_type, request, value, index, length)
    return self.transact(dat, timeout)

  def bulkWrite(self, endpoint, data, timeout=0):
    if len(data) > 0x10:
      raise ValueError("Data must not be longer than 0x10")
    dat = struct.pack("HH", endpoint, len(data)) + data
    return self.transact(dat, timeout)

  def bulkRead(self, endpoint, length, timeout=0):
    dat = struct.pack("HH", endpoint, 0)
    return self.transact(dat, timeout)

