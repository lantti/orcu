import orcumaster as orcu

class lrkmaster(orcu.orcumaster):
#  def __init__(self):
#    super().__init__()
#    return
#
#  def __enter__(self):
#    super().__enter__()
#    return self
  def lrkIndexed(self, row, column, colour1, colour2):
    c1 = colour1 % 16
    c2 = colour2 % 16
    cbyte = (c1<<4) | c2
    if (self.nrfGetDynamicPayloadEnable(False) == True):
      self.nrfSetDynamicPayloadEnable(False)
    return self.ocSend(row, column, 3, [cbyte])
