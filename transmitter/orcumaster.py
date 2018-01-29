import nrf24l01p as nrf
import random
import time

random.seed()

cmdPing =          0x00  #no args, returns id
cmdLoadPage =      0x01  #takes page number, returns error code
cmdWriteBuffer =   0x02  #takes buffer offset, data length-1 and data, returns error code
cmdProgramPage =   0x03  #takes page number, returns error code
cmdProgramEeprom = 0x04  #takes eeprom offset, data length-1 and data, returns error code
cmdForceReset =    0x05  #no args, no return value
cmdInitRf =        0x06  #no args, no return value
cmdNop =           0xFF  #no args, no return value

pipeAddress = [0xE7,0xC2,0xC3,0xC4,0xC5,0xC6]

def pad():
  return random.randint(0,255)

class orcumaster(nrf.nrf24l01p):
  def __init__(self, channel):
    super().__init__()
    self.channel = channel
    return

  def __enter__(self):
    super().__enter__()
    self.nrfSetAddressWidth(3)
    self.nrfSetRadioChannel(self.channel)
    self.nrfSetDynamicPayloadEnable(True)
    self.nrfSetAckPayloadEnable(True)
    self.nrfSetRetryDelay(500)
    self.nrfSetDynamicPayloadPipe(0, True)
    self.nrfSetPowerUp(True)
    return self

  def ocNop(self, row, column):
    return self.ocSend(row, column, 0, [cmdNop])

  def ocPing(self, row, column):
    resp = self.ocSend(row, column, 0, [cmdPing])
    if (resp[0] == False):
      return resp
    resp = self.ocNop(row, column)
    if (resp[0] == False):
      return resp
    return (True, list(map(chr,resp[1])))
 
  def ocLoadPage(self, row, column, page):
    resp = self.ocSend(row, column, 0, [cmdLoadPage, page])
    if (resp[0] == False):
      return resp
    resp = self.ocNop(row, column)
    if (resp[0] == False):
      return resp
    return (True, resp[1][0])

  def ocWriteBuffer(self, row, column, offset, data):
    resp = self.ocSend(row, column, 0, [cmdWriteBuffer, offset, len(data)-1] + data)
    if (resp[0] == False):
      return resp
    resp = self.ocNop(row, column)
    if (resp[0] == False):
      return resp
    return (True, resp[1][0])

  def ocProgramPage(self, row, column, page):
    resp = self.ocSend(row, column, 0, [cmdProgramPage, page])
    if (resp[0] == False):
      return resp
    time.sleep(1)
    resp = self.ocNop(row, column)
    if (resp[0] == False):
      return resp
    return (True, resp[1][0])
    
  def ocProgramEeprom(self, row, column, offset, data):
    resp = self.ocSend(row, column, 0, [cmdProgramEeprom, offset, len(data)-1] + data)
    if (resp[0] == False):
      return resp
    time.sleep(1)    
    resp = self.ocNop(row, column)
    if (resp[0] == False):
      return resp
    return (True, resp[1][0])

  def ocForceReset(self, row, column):
    return self.ocSend(row, column, 0, [cmdForceReset])

  def ocInitRf(self, row, column):
    return self.ocSend(row, column, 0, [cmdInitRf])



  def ocFlushReturnQueue(self, row, column):
    tries = 0
    successes = 0
    while (tries < 15 and successes < 4):
      resp = self.ocNop(row, column)
      tries += 1
      if (resp[0] == True):
        successes += 1
    return (successes == 4)

  def ocSend(self, row, column, pipe, payload):
    if (pipe == 0 and self.nrfGetDynamicPayloadEnable(False) == False):
      self.nrfSetDynamicPayloadEnable(True)
    return self.nrfSend(payload + [pad()], [pipeAddress[pipe]]+[row, column]) 

  def ocChangeChannelAndAddress(self, row, column, newRow, newColumn, newChannel):
    resp = self.ocProgramEeprom(row, column, 0, [newChannel, newColumn, newRow])
    if (resp[0] == False or resp[1] != 0):
      return resp
    self.ocInitRf(row, column)
    return resp

