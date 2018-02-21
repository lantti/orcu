import spidev as spidev
import RPi.GPIO as GPIO

CE_Pin =   22
IRQ_Pin =  18
SPImaj =   0 
SPImin =   0
SPIspeed = 8000000

cmdRRegister =       0b00000000
cmdWRegister =       0b00100000
cmdRRxPayload =      0b01100001
cmdWTxPayload =      0b10100000
cmdFlushTx =         0b11100001
cmdFlushRx =         0b11100010
cmdReuseTxPl =       0b11100011
cmdRRxPlWid =        0b01100000
cmdWAckPayload =     0b10101000
cmdWTxPayloadNoAck = 0b10110000
cmdNop =             0b11111111


regConfig =     0x00
bitConfigMaskRxDr =  6
bitConfigMaskTxDs =  5
bitConfigMaskMaxRt = 4
bitConfigEnCrc =     3
bitConfigCrco =      2
bitConfigPwrUp =     1
bitConfigPrimRx =    0
initConfig = 0b00001000

regEnAa =       0x01
bitEnAaP5 = 5
bitEnAaP4 = 4
bitEnAaP3 = 3
bitEnAaP2 = 2
bitEnAaP1 = 1
bitEnAaP0 = 0
initEnAa =       0b00111111

regEnRxaddr =   0x02
bitEnRxaddrP5 = 5
bitEnRxaddrP4 = 4
bitEnRxaddrP3 = 3
bitEnRxaddrP2 = 2
bitEnRxaddrP1 = 1
bitEnRxaddrP0 = 0
initEnRxaddr =   0b00000011
	
regSetupAw =    0x03
initSetupAw =    0b00000011

regSetupRetr =  0x04
bitSetupRetrArd0 = 4
bitSetupRetrArc0 = 0
initSetupRetr =  0b00000011

regRfCh =       0x05
initRfCh =       0b00000010

regRfSetup =    0x06
bitRfSetupContWave = 7
bitRfSetupRfDrLow =  5
bitRfSetupPllLock =  4
bitRfSetupRfDrHigh = 3
bitRfSetupRfPwr0 =   1
initRfSetup =    0b00001111

regStatus =     0x07
bitStatusRxDr =   6
bitStatusTxDs =   5
bitStatusMaxRt =  4
bitStatusRxPNo0 = 1
bitStatusTxFull = 0
initStatus =     0b00001110

regObserveTx =  0x08
bitObserveTxPlosCnt0 = 4
bitObserveTxArcCnt0 =  0
initObserveTx = 0b00000000

regRpd =        0x09
bitRpdRpd = 0
initRpd = 0b00000000

regRxAddrP0 =  0x0A
initRxAddrP0 =   0xE7

regRxAddrP1 =  0x0B
initRxAddrP1 =   0xC2

regRxAddrP2 =  0x0C
initRxAddrP2 =   0xC3

regRxAddrP3 =  0x0D
initRxAddrP3 =   0xC4

regRxAddrP4 =  0x0E
initRxAddrP4 =   0xC5

regRxAddrP5 =  0x0F
initRxAddrP5 =   0xC6

regTxAddr =     0x10
initTxAddr =     0xE7

regRxPwP0 =     0x11
initRxPwP0 =     0x00

regRxPwP1 =     0x12
initRxPwP1 =     0x00

regRxPwP2 =     0x13
initRxPwP2 =     0x00

regRxPwP3 =     0x14
initRxPwP3 =     0x00

regRxPwP4 =     0x15
initRxPwP4 =     0x00

regRxPwP5 =     0x16
initRxPwP5 =     0x00

regFifoStatus = 0x17
bitFifoStatusTxReuse = 6
bitFifoStatusTxFull =  5
bitFifoStatusTxEmpty = 4
bitFifoStatusRxFull =  1
bitFifoStatusRxEmpty = 0
initFifoStatus = 0b00010001

regDynpd =      0x1C
bitDynpdP5 = 5
bitDynpdP4 = 4
bitDynpdP3 = 3
bitDynpdP2 = 2
bitDynpdP1 = 1
bitDynpdP0 = 0
initDynpd =      0b00000000

regFeature =    0x1D
bitFeatureEnDpl = 2
bitFeatureEnAckPay = 1
bitFeatureEnDynAck = 0
initFeature =    0b00000000


#TODO catch and recast exceptions for non-8bit vals
class nrf24l01p():
  def __init__(self):
    self.spi = spidev.SpiDev()
    self.__registerCache = {regConfig:0,
       regEnAa:0,
       regEnRxaddr:0,
       regSetupAw:0,
       regSetupRetr:0,
       regRfCh:0,
       regRfSetup:0,
       regStatus:0,
       regObserveTx:0,
       regRpd:0,
       regRxAddrP0:0,
       regRxAddrP1:0,
       regRxAddrP2:0,
       regRxAddrP3:0,
       regRxAddrP4:0,
       regRxAddrP5:0,
       regTxAddr:0,
       regRxPwP0:0,
       regRxPwP1:0,
       regRxPwP2:0,
       regRxPwP3:0,
       regRxPwP4:0,
       regRxPwP5:0,
       regFifoStatus:0,
       regDynpd:0,
       regFeature:0}
    return

  def __enter__(self):
    self.spi.open(SPImaj, SPImin)
    self.spi.max_speed_hz = SPIspeed
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(IRQ_Pin, GPIO.IN)
    GPIO.setup(CE_Pin, GPIO.OUT, initial=GPIO.LOW)
    self.initializeRegs()
    return self

  def __exit__(self,et,ev,tb):
    self.initializeRegs()
    GPIO.cleanup()
    self.spi.close()
    return

  def __pendingIRQ(self):
    return not(GPIO.input(IRQ_Pin))

  def __readReg(self, reg, len):
    request = [cmdRRegister|(reg&0x1F)] + [0x00] * len
    return self.spi.xfer(request)[1:]

  def __writeReg(self, reg, vals):
    self.spi.xfer([cmdWRegister|(reg&0x1F)] + list(bytes(vals)))
    return

  def __readPayload(self, len):
    request = [cmdRRxPayload] + [0x00] * len
    return self.spi.xfer(request)[1:]
    
  def __writePayload(self, vals):
    self.spi.xfer([cmdWTxPayload] + list(bytes(vals)))
    return

  def __flushTx(self):
    self.spi.xfer([cmdFlushTx])
    return

  def __flushRx(self):
    self.spi.xfer([cmdFlushRx])
    return

  def __reusePayload(self):
    self.spi.xfer([cmdReuseTxPl])
    return

  def __readPayloadWidth(self):
    return self.spi.xfer([cmdRRxPlWid, 0x00])[1]
    
  def __writeAckPayload(self, pipe, vals):
    if (0 > pipe or pipe > 5):
      raise ValueError("Available pipe numbers are 0-5")
    self.spi.xfer([cmdWAckPayload|pipe] + list(bytes(vals)))
    return

  def __writePayloadNoAck(self, vals):
    self.spi.xfer([cmdWTxPayloadNoAck] + list(bytes(vals)))
    return

  def __nop(self):
    self.spi.xfer([cmdNop])
    return

  def __status(self):
    return self.spi.xfer([cmdNop])[0]







  def initializeRegs(self):
    self.__flushTx()
    self.__flushRx()
    self.__registerCache[regFifoStatus] = initFifoStatus
    self.__writeReg(regConfig, [initConfig])
    self.__registerCache[regConfig] = initConfig
    self.__writeReg(regEnAa, [initEnAa])
    self.__registerCache[regEnAa] = initEnAa
    self.__writeReg(regEnRxaddr, [initEnRxaddr])
    self.__registerCache[regEnRxaddr] = initEnRxaddr
    self.__writeReg(regSetupAw, [initSetupAw])
    self.__registerCache[regSetupAw] = initSetupAw
    self.__writeReg(regSetupRetr, [initSetupRetr])
    self.__registerCache[regSetupRetr] = initSetupRetr
    self.__writeReg(regRfCh, [initRfCh])
    self.__registerCache[regRfCh] = initRfCh
    self.__registerCache[regObserveTx] = initObserveTx
    self.__writeReg(regRfSetup, [initRfSetup])
    self.__registerCache[regRfSetup] = initRfSetup
    self.nrfClearIRQs(True,True,True)
    self.__registerCache[regStatus] = initStatus
    self.__registerCache[regRpd] = initRpd
    self.__writeReg(regRxAddrP0, [initRxAddrP0] * 5)
    self.__registerCache[regRxAddrP0] = [initRxAddrP0] * 5
    self.__writeReg(regRxAddrP1, [initRxAddrP1] * 5)
    self.__registerCache[regRxAddrP1] = [initRxAddrP1] * 5
    self.__writeReg(regRxAddrP2, [initRxAddrP2])
    self.__registerCache[regRxAddrP2] = [initRxAddrP2]
    self.__writeReg(regRxAddrP3, [initRxAddrP3])
    self.__registerCache[regRxAddrP3] = [initRxAddrP3]
    self.__writeReg(regRxAddrP4, [initRxAddrP4])
    self.__registerCache[regRxAddrP4] = [initRxAddrP4]
    self.__writeReg(regRxAddrP5, [initRxAddrP5])
    self.__registerCache[regRxAddrP5] = [initRxAddrP5]
    self.__writeReg(regTxAddr, [initTxAddr] * 5) 
    self.__registerCache[regTxAddr] = initTxAddr
    self.__writeReg(regRxPwP0, [initRxPwP0])
    self.__registerCache[regRxPwP0] = initRxPwP0
    self.__writeReg(regRxPwP1, [initRxPwP1])
    self.__registerCache[regRxPwP1] = initRxPwP1
    self.__writeReg(regRxPwP2, [initRxPwP2])
    self.__registerCache[regRxPwP2] = initRxPwP2
    self.__writeReg(regRxPwP3, [initRxPwP3])
    self.__registerCache[regRxPwP3] = initRxPwP3
    self.__writeReg(regRxPwP4, [initRxPwP4])
    self.__registerCache[regRxPwP4] = initRxPwP4
    self.__writeReg(regRxPwP5, [initRxPwP5])
    self.__registerCache[regRxPwP5] = initRxPwP5
    self.__writeReg(regDynpd, [initDynpd])
    self.__registerCache[regDynpd] = initDynpd
    self.__writeReg(regFeature, [initFeature])
    self.__registerCache[regFeature] = initFeature
    return

  def refreshRegisterCache(self):
    self.__registerCache[regConfig] = self.__readReg(regConfig, 1)[0]
    self.__registerCache[regEnAa] = self.__readReg(regEnAa, 1)[0]
    self.__registerCache[regEnRxaddr] = self.__readReg(regEnRxaddr, 1)[0]
    self.__registerCache[regSetupAw] = self.__readReg(regSetupAw, 1)[0]
    self.__registerCache[regSetupRetr] = self.__readReg(regSetupRetr, 1)[0]
    self.__registerCache[regRfCh] = self.__readReg(regRfCh, 1)[0]
    self.__registerCache[regRfSetup] = self.__readReg(regRfSetup, 1)[0]
    self.__registerCache[regStatus] = self.__readReg(regStatus, 1)[0]
    self.__registerCache[regObserveTx] = self.__readReg(regObserveTx, 1)[0]
    self.__registerCache[regRpd] = self.__readReg(regRpd, 1)[0]
    self.__registerCache[regRxAddrP0] = self.__readReg(regRxAddrP0, 5)
    self.__registerCache[regRxAddrP1] = self.__readReg(regRxAddrP1, 5)
    self.__registerCache[regRxAddrP2] = self.__readReg(regRxAddrP2, 1)
    self.__registerCache[regRxAddrP3] = self.__readReg(regRxAddrP3, 1)
    self.__registerCache[regRxAddrP4] = self.__readReg(regRxAddrP4, 1)
    self.__registerCache[regRxAddrP5] = self.__readReg(regRxAddrP5, 1)
    self.__registerCache[regTxAddr] = self.__readReg(regTxAddr, 5)
    self.__registerCache[regRxPwP0] = self.__readReg(regRxPwP0, 1)[0]
    self.__registerCache[regRxPwP1] = self.__readReg(regRxPwP1, 1)[0]
    self.__registerCache[regRxPwP2] = self.__readReg(regRxPwP2, 1)[0]
    self.__registerCache[regRxPwP3] = self.__readReg(regRxPwP3, 1)[0]
    self.__registerCache[regRxPwP4] = self.__readReg(regRxPwP4, 1)[0]
    self.__registerCache[regRxPwP5] = self.__readReg(regRxPwP5, 1)[0]
    self.__registerCache[regFifoStatus] = self.__readReg(regFifoStatus, 1)[0]
    self.__registerCache[regDynpd] = self.__readReg(regDynpd, 1)[0]
    self.__registerCache[regFeature] = self.__readReg(regFeature, 1)[0]

  def nrfClearIRQs(self, rx, tx, rt):
    rxbit = int(rx) % 2
    txbit = int(tx) % 2
    rtbit = int(rt) % 2
    self.__writeReg(regStatus,[(rxbit<<bitStatusRxDr)|(txbit<<bitStatusTxDs)|(rtbit<<bitStatusMaxRt)])
    return

  def nrfGetIrqMask(self, fetch = True):
    if (fetch == True):
      self.__registerCache[regConfig] = self.__readReg(regConfig, 1)[0]
    c = self.__registerCache[regConfig]
    rxbit = bool(c & (1<<bitConfigMaskRxDr))
    txbit = bool(c & (1<<bitConfigMaskTxDs))
    rtbit = bool(c & (1<<bitConfigMaskMaxRt))
    return (rxbit,txbit,rtbit)
    

  def nrfSetIrqMask(self, maskRx, maskTx, maskRt):
    rxbit = int(maskRx) % 2
    txbit = int(maskTx) % 2
    rtbit = int(maskRt) % 2
    emptyMask = 0xFF&~(1<<bitConfigMaskRxDr)&~(1<<bitConfigMaskTxDs)&~(1<<bitConfigMaskMaxRt)
    newMask = (rxbit<<bitConfigMaskRxDr)|(txbit<<bitConfigMaskTxDs)|(rtbit<<bitConfigMaskMaxRt)
    self.__registerCache[regConfig] &= emptyMask 
    self.__registerCache[regConfig] |= newMask 
    self.__writeReg(regConfig,[self.__registerCache[regConfig]])
    return

  def nrfGetCrcMode(self, fetch = True):
    if (fetch == True):
      self.__registerCache[regConfig] = self.__readReg(regConfig, 1)[0]
    c = self.__registerCache[regConfig]
    enbit = bool(c & (1<<bitConfigEnCrc))
    scbit = int(bool(c & (1<<bitConfigCrco)))
    return (enbit,scbit)

  def nrfSetCrcMode(self, enable, scheme):
    enbit = int(enable) % 2
    scbit = int(scheme) % 2
    emptyBits = 0xFF&~(1<<bitConfigEnCrc)&~(1<<bitConfigCrco)
    newBits = (enbit<<bitConfigEnCrc)|(scbit<<bitConfigCrco)
    self.__registerCache[regConfig] &= emptyBits 
    self.__registerCache[regConfig] |= newBits 
    self.__writeReg(regConfig,[self.__registerCache[regConfig]])
    self.__registerCache[regConfig] = self.__readReg(regConfig, 1)[0]
    return    

  def nrfGetPowerUp(self, fetch = True):
    if (fetch == True):
      self.__registerCache[regConfig] = self.__readReg(regConfig, 1)[0]
    c = self.__registerCache[regConfig]
    pwrbit = bool(c & (1<<bitConfigPwrUp))
    return pwrbit

  def nrfSetPowerUp(self, powerUp):
    pwrbit = int(powerUp) % 2
    self.__registerCache[regConfig] &= ~(1<<bitConfigPwrUp) 
    self.__registerCache[regConfig] |= (pwrbit<<bitConfigPwrUp)
    self.__writeReg(regConfig,[self.__registerCache[regConfig]])
    return    

  def nrfGetPrimRxMode(self, fetch = True):
    if (fetch == True):
      self.__registerCache[regConfig] = self.__readReg(regConfig, 1)[0]
    c = self.__registerCache[regConfig]
    primbit = bool(c & (1<<bitConfigPrimRx))
    return primbit

  def nrfSetPrimRxMode(self, primRx):
    primbit = int(primRx) % 2
    self.__registerCache[regConfig] &= ~(1<<bitConfigPrimRx) 
    self.__registerCache[regConfig] |= (primbit<<bitConfigPrimRx)
    self.__writeReg(regConfig,[self.__registerCache[regConfig]])
    return    


  def nrfGetAutoAckModes(self, fetch = True):
    if (fetch == True):
      self.__registerCache[regEnAa] = self.__readReg(regEnAa, 1)[0]
    c = self.__registerCache[regEnAa]
    enaa5 = bool(c & (1<<bitEnAaP5))
    enaa4 = bool(c & (1<<bitEnAaP4))
    enaa3 = bool(c & (1<<bitEnAaP3))
    enaa2 = bool(c & (1<<bitEnAaP2))
    enaa1 = bool(c & (1<<bitEnAaP1))
    enaa0 = bool(c & (1<<bitEnAaP0))
    return (enaa5,enaa4,enaa3,enaa2,enaa1,enaa0)

  def nrfSetAutoAckModes(self, p5, p4, p3, p2, p1, p0):
    p5bit = int(p5) % 2
    p4bit = int(p4) % 2
    p3bit = int(p3) % 2
    p2bit = int(p2) % 2
    p1bit = int(p1) % 2
    p0bit = int(p0) % 2
    emptyBits = 0xFF&~(1<<bitEnAaP5)&~(1<<bitEnAaP4)&~(1<<bitEnAaP3)&~(1<<bitEnAaP2)&~(1<<bitEnAaP1)&~(1<<bitEnAaP0)
    enableBits = (p5bit<<bitEnAaP5)|(p4bit<<bitEnAaP4)|(p3bit<<bitEnAaP3)|(p2bit<<bitEnAaP2)|(p1bit<<bitEnAaP1)|(p0bit<<bitEnAaP0)
    self.__registerCache[regEnAa] &= emptyBits 
    self.__registerCache[regEnAa] |= enableBits 
    self.__writeReg(regEnAa,[self.__registerCache[regEnAa]])
    self.__registerCache[regConfig] = self.__readReg(regConfig, 1)[0] 
    return

  def nrfGetRxEnables(self, fetch = True):
    if (fetch == True):
      self.__registerCache[regEnRxaddr] = self.__readReg(regEnRxaddr, 1)[0]
    c = self.__registerCache[regEnRxaddr]
    enrx5 = bool(c & (1<<bitEnRxaddrP5))
    enrx4 = bool(c & (1<<bitEnRxaddrP4))
    enrx3 = bool(c & (1<<bitEnRxaddrP3))
    enrx2 = bool(c & (1<<bitEnRxaddrP2))
    enrx1 = bool(c & (1<<bitEnRxaddrP1))
    enrx0 = bool(c & (1<<bitEnRxaddrP0))
    return (enrx5,enrx4,enrx3,enrx2,enrx1,enrx0)

  def nrfSetRxEnables(self, p5, p4, p3, p2, p1, p0):
    p5bit = int(p5) % 2
    p4bit = int(p4) % 2
    p3bit = int(p3) % 2
    p2bit = int(p2) % 2
    p1bit = int(p1) % 2
    p0bit = int(p0) % 2
    emptyBits = 0xFF&~(1<<bitEnRxaddrP5)&~(1<<bitEnRxaddrP4)&~(1<<bitEnRxaddrP3)&~(1<<bitEnRxaddrP2)&~(1<<bitEnRxaddrP1)&~(1<<bitEnRxaddrP0)
    enableBits = (p5bit<<bitEnRxaddrP5)|(p4bit<<bitEnRxaddrP4)|(p3bit<<bitEnRxaddrP3)|(p2bit<<bitEnRxaddrP2)|(p1bit<<bitEnRxaddrP1)|(p0bit<<bitEnRxaddrP0)
    self.__registerCache[regEnRxaddr] &= emptyBits 
    self.__registerCache[regEnRxaddr] |= enableBits 
    self.__writeReg(regEnRxaddr,[self.__registerCache[regEnRxaddr]])
    return

  def nrfGetAddressWidth(self, fetch = True):
    if (fetch == True):
      self.__registerCache[regSetupAw] = self.__readReg(regSetupAw, 1)[0]
    c = self.__registerCache[regSetupAw]
    c &= 0x03
    if (c == 0):
      return 0
    else:
      return (c + 2)
    
  def nrfSetAddressWidth(self, width):
    if (3 > width or width > 5):
      raise ValueError("Address Width must be 3, 4 or 5 bytes")
    self.__registerCache[regSetupAw] = width - 2
    self.__writeReg(regSetupAw,[self.__registerCache[regSetupAw]])
    return     

  def nrfGetRetryDelay(self, fetch = True):
    if (fetch == True):
      self.__registerCache[regSetupRetr] = self.__readReg(regSetupRetr, 1)[0]
    c = self.__registerCache[regSetupRetr]
    return ((1 + (c >> bitSetupRetrArd0)) * 250)

  def nrfSetRetryDelay(self, delay):
    if (250 > delay or delay > 4000):
      raise ValueError("Automatic Retransmission Delay must be within 250 and 4000us")
    emptyBits = 0xFF >> bitSetupRetrArd0
    delayBits = (round(delay / 250) - 1) << bitSetupRetrArd0
    self.__registerCache[regSetupRetr] &= emptyBits 
    self.__registerCache[regSetupRetr] |= delayBits 
    self.__writeReg(regSetupRetr,[self.__registerCache[regSetupRetr]])
    return    

  def nrfGetMaxRetryCount(self, fetch = True):
    if (fetch == True):
      self.__registerCache[regSetupRetr] = self.__readReg(regSetupRetr, 1)[0]
    c = self.__registerCache[regSetupRetr]
    return (c & (0xFF >> bitSetupRetrArd0))

  def nrfSetMaxRetryCount(self, count):
    if (0 > count or count > 15):
      raise ValueError("Maximum Automatic Retransmission Count must be within 0 and 15")
    emptyBits = 0xFF << bitSetupRetrArd0
    countBits = count & (0xFF >> bitSetupRetrArd0)
    self.__registerCache[regSetupRetr] &= emptyBits 
    self.__registerCache[regSetupRetr] |= countBits 
    self.__writeReg(regSetupRetr,[self.__registerCache[regSetupRetr]])
    return    

  def nrfGetRadioChannel(self, fetch = True):
    if (fetch == True):
      self.__registerCache[regRfCh] = self.__readReg(regRfCh, 1)[0]
    c = self.__registerCache[regRfCh]
    return c
    
  def nrfSetRadioChannel(self, channel):
    if (0 > channel or channel > 127):
      raise ValueError("Radio channel must be between 0 and 127")
    self.__registerCache[regRfCh] = channel & 0x7F
    self.__writeReg(regRfCh,[self.__registerCache[regRfCh]])
    return     

  def nrfGetContinuousCarrierMode(self, fetch = True):
    if (fetch == True):
      self.__registerCache[regRfSetup] = self.__readReg(regRfSetup, 1)[0]
    c = self.__registerCache[regRfSetup]
    ccbit = bool(c & (1<<bitRfSetupContWave))
    return ccbit

  def nrfSetContinuousCarrierMode(self, ccmode):
    ccbit = int(ccmode) % 2
    self.__registerCache[regRfSetup] &= ~(1<<bitRfSetupContWave) 
    self.__registerCache[regRfSetup] |= (ccbit<<bitRfSetupContWave)
    self.__writeReg(regRfSetup,[self.__registerCache[regRfSetup]])
    return    

  def nrfGetRfDataRate(self, fetch = True):
    if (fetch == True):
      self.__registerCache[regRfSetup] = self.__readReg(regRfSetup, 1)[0]
    c = self.__registerCache[regRfSetup]
    drhighbit = bool(c & (1<<bitRfSetupRfDrHigh))
    drlowbit = bool(c & (1<<bitRfSetupRfDrLow))
    drbits = drlowbit | (drhighbit << 1)
    if (drbits == 0):
      dr = 1000
    elif (drbits == 1):
      dr = 250
    elif (drbits == 2):
      dr = 2000
    else:
      dr = 0
    return dr

  def nrfSetRfDataRate(self, dr):
    if (dr == 250):
      drhighbit = 0 
      drlowbit = 1
    elif (dr == 1000):
      drhighbit = 0
      drlowbit = 0
    elif (dr == 2000):
      drhighbit = 1
      drlowbit = 0
    else:
      raise ValueError("RF Data Rate must be one of 250kbps, 1000kbps or 2000kbps")
    emptyBits = 0xFF&~(1<<bitRfSetupRfDrHigh)&~(1<<bitRfSetupRfDrLow)
    drBits = (drhighbit<<bitRfSetupRfDrHigh)|(drlowbit<<bitRfSetupRfDrLow)
    self.__registerCache[regRfSetup] &= emptyBits 
    self.__registerCache[regRfSetup] |= drBits 
    self.__writeReg(regRfSetup,[self.__registerCache[regRfSetup]])
    return

  def nrfGetTransmitPower(self, fetch = True):
    if (fetch == True):
      self.__registerCache[regRfSetup] = self.__readReg(regRfSetup, 1)[0]
    c = self.__registerCache[regRfSetup]
    pwrbits = (~(0xFF << (bitRfSetupRfPwr0 + 2)) & c) >> bitRfSetupRfPwr0
    if (pwrbits == 0):
      pwr = -18
    elif (pwrbits == 1):
      pwr = -12
    elif (pwrbits == 2):
      pwr = -6
    else:
      pwr = 0
    return pwr

  def nrfSetTransmitPower(self, pwr):
    if (pwr == -18):
      pwrbits = 0
    elif (pwr == -12):
      pwrbits = 1
    elif (pwr == -6):
      pwrbits = 2
    elif (pwr == 0):
      pwrbits = 3
    else:
      raise ValueError("RF Transmit Power must be one of -18dBm, -12dBm, -6dBm or 0dBm")
    emptyBits = ~(0x03<<bitRfSetupRfPwr0)
    pwrbitsBits = pwrbits<<bitRfSetupRfPwr0
    self.__registerCache[regRfSetup] &= emptyBits 
    self.__registerCache[regRfSetup] |= pwrbitsBits 
    self.__writeReg(regRfSetup,[self.__registerCache[regRfSetup]])
    return

  def nrfGetRxInterrupt(self):
    self.__registerCache[regStatus] = self.__status()
    s = self.__registerCache[regStatus]
    return bool(s & (1<<bitStatusRxDr))

  def nrfGetTxInterrupt(self):
    self.__registerCache[regStatus] = self.__status()
    s = self.__registerCache[regStatus]
    return bool(s & (1<<bitStatusTxDs))

  def nrfGetMaxRtInterrupt(self):
    self.__registerCache[regStatus] = self.__status()
    s = self.__registerCache[regStatus]
    return bool(s & (1<<bitStatusMaxRt))

  def nrfGetReceivePipe(self):
    self.__registerCache[regStatus] = self.__status()
    s = self.__registerCache[regStatus]
    pipe = (~(0xFF << (bitStatusRxPNo0+3)) & s) >> bitStatusRxPNo0
    if (pipe == 7):
      return (False, pipe)
    else:
      return (True, pipe)

  def nrfGetTxFullFlag(self):
    self.__registerCache[regStatus] = self.__status()
    s = self.__registerCache[regStatus]
    return bool(s & (1<<bitStatusTxFull))

  def nrfGetPacketLoss(self, fetch = True):
    if (fetch == True):
      self.__registerCache[regObserveTx] = self.__readReg(regObserveTx, 1)[0]
    c = self.__registerCache[regObserveTx]
    return (c >> bitObserveTxPlosCnt0)

  def nrfGetRetryCount(self, fetch = True):
    if (fetch == True):
      self.__registerCache[regObserveTx] = self.__readReg(regObserveTx, 1)[0]
    c = self.__registerCache[regObserveTx]
    return (c & ~(0xFF << bitObserveTxPlosCnt0))

  def nrfGetReceivedPowerFlag(self,  fetch = True):
    if (fetch == True):
      self.__registerCache[regRpd] = self.__readReg(regRpd, 1)[0]
    c = self.__registerCache[regRpd]
    return bool(c & (1<<bitRpdRpd))

  def nrfGetRxPipeAddress(self, pipe, fetch = True):
    if (5 < pipe or pipe < 0):
      raise ValueError("Available receive pipe numbers are 0-5")
    addrReg = regRxAddrP0 + pipe
    if (fetch == True):
      if (pipe > 1):
        self.__registerCache[addrReg] = self.__readReg(addrReg, 1)
      else:
        aw = self.nrfGetAddressWidth(True)
        self.__registerCache[addrReg] = self.__readReg(addrReg, aw)
    addr = self.__registerCache[addrReg]
    return addr

  def nrfSetRxPipeAddress(self, pipe, address, dirty = False):
    if (5 < pipe or pipe < 0):
      raise ValueError("Available receive pipe numbers are 0-5")
    if (pipe > 1):
      if (len(address) > 1):
        raise ValueError("Pipes 2-5 can only have one own address byte")
    else:
      aw = self.nrfGetAddressWidth(~dirty)
      if (len(address) != aw):
        raise ValueError("Addresses for pipes 0 and 1 must agree with the Address Width set previously") 
    addrReg = regRxAddrP0 + pipe
    self.__registerCache[addrReg] = address
    self.__writeReg(addrReg, self.__registerCache[addrReg])
    return

  def nrfGetTxAddress(self, fetch = True):
    if (fetch == True):
      aw = self.nrfGetAddressWidth(True)
      self.__registerCache[regTxAddr] = self.__readReg(regTxAddr, aw)
    addr = self.__registerCache[regTxAddr]
    return addr

  def nrfSetTxAddress(self, address, dirty = False):
    aw = self.nrfGetAddressWidth(~dirty)
    if (len(address) != aw):
      raise ValueError("Transmit Address must agree with the Address Width set previously") 
    self.__registerCache[regTxAddr] = address
    self.__writeReg(regTxAddr, self.__registerCache[regTxAddr])
    return

  def nrfGetRxPipeWidth(self, pipe, fetch = True):
    if (5 < pipe or pipe < 0):
      raise ValueError("Available receive pipe numbers are 0-5")
    pwReg = regRxPwP0 + pipe
    if (fetch == True):
      self.__registerCache[pwReg] = self.__readReg(pwReg, 1)[0]
    return self.__registerCache[pwReg]

  def nrfSetRxPipeWidth(self, pipe, width):
    if (5 < pipe or pipe < 0):
      raise ValueError("Available receive pipe numbers are 0-5")
    if (32 < pipe or pipe < 0):
      raise ValueError("Receive Pipe Width must be between 0 and 32")
    pwReg = regRxPwP0 + pipe
    self.__registerCache[pwReg] = width
    self.__writeReg(pwReg,[self.__registerCache[pwReg]])
    return

  def nrfGetFifoStatus(self, fetch = True):
    if (fetch == True):
      self.__registerCache[regFifoStatus] = self.__readReg(regFifoStatus, 1)[0]
    c = self.__registerCache[regFifoStatus]
    txrbit = bool(c & (1<<bitFifoStatusTxReuse))
    txfbit = bool(c & (1<<bitFifoStatusTxFull))
    txebit = bool(c & (1<<bitFifoStatusTxEmpty))
    rxfbit = bool(c & (1<<bitFifoStatusRxFull))
    rxebit = bool(c & (1<<bitFifoStatusRxEmpty))
    return (txrbit,txfbit,txebit,rxfbit,rxebit)

  def nrfGetDynamicPayloadPipes(self, fetch = True):
    if (fetch == True):
      self.__registerCache[regDynpd] = self.__readReg(regDynpd, 1)[0]
    c = self.__registerCache[regDynpd]
    dp5 = bool(c & (1<<bitDynpdP5))
    dp4 = bool(c & (1<<bitDynpdP4))
    dp3 = bool(c & (1<<bitDynpdP3))
    dp2 = bool(c & (1<<bitDynpdP2))
    dp1 = bool(c & (1<<bitDynpdP1))
    dp0 = bool(c & (1<<bitDynpdP0))
    return (dp5,dp4,dp3,dp2,dp1,dp0)

  def nrfSetDynamicPayloadPipe(self, pipe, enable):
    if (5 < pipe or pipe < 0):
      raise ValueError("Available receive pipe numbers are 0-5")
    bitpos = bitDynpdP0 + pipe
    enbit = int(enable) % 2
    self.__registerCache[regDynpd] &= ~(1<<bitpos) 
    self.__registerCache[regDynpd] |= (enbit << bitpos)
    self.__writeReg(regDynpd,[self.__registerCache[regDynpd]])
    return

  def nrfGetDynamicPayloadEnable(self, fetch = True):
    if (fetch == True):
      self.__registerCache[regFeature] = self.__readReg(regFeature, 1)[0]
    c = self.__registerCache[regFeature]
    return bool(c & (1<<bitFeatureEnDpl))

  def nrfSetDynamicPayloadEnable(self, enable):
    enbit = int(enable) % 2
    self.__registerCache[regFeature] &= ~(1<<bitFeatureEnDpl) 
    self.__registerCache[regFeature] |= (enbit<<bitFeatureEnDpl)
    self.__writeReg(regFeature,[self.__registerCache[regFeature]])
    return

  def nrfGetAckPayloadEnable(self, fetch = True):
    if (fetch == True):
      self.__registerCache[regFeature] = self.__readReg(regFeature, 1)[0]
    c = self.__registerCache[regFeature]
    return bool(c & (1<<bitFeatureEnAckPay))

  def nrfSetAckPayloadEnable(self, enable):
    enbit = int(enable) % 2
    self.__registerCache[regFeature] &= ~(1<<bitFeatureEnAckPay) 
    self.__registerCache[regFeature] |= (enbit<<bitFeatureEnAckPay)
    self.__writeReg(regFeature,[self.__registerCache[regFeature]])
    return

  def nrfGetNoAckEnable(self, fetch = True):
    if (fetch == True):
      self.__registerCache[regFeature] = self.__readReg(regFeature, 1)[0]
    c = self.__registerCache[regFeature]
    return bool(c & (1<<bitFeatureEnDynAck))

  def nrfSetNoAckEnable(self, enable):
    enbit = int(enable) % 2
    self.__registerCache[regFeature] &= ~(1<<bitFeatureEnDynAck) 
    self.__registerCache[regFeature] |= (enbit<<bitFeatureEnDynAck)
    self.__writeReg(regFeature,[self.__registerCache[regFeature]])
    return

  def nrfPrepareAckPayload(self, pipe, payload):
    self.__writeAckPayload(pipe, payload)
    return


  def nrfSend(self, payload = [], address = [], ack = True):
    if (len(address) > 0):
      self.nrfSetTxAddress(address)
      self.nrfSetRxPipeAddress(0, address)
    if (len(payload) > 0):
      if (ack == True):
        self.__writePayload(payload)
      else:
        self.__writePayloadNoAck(payload)
    else:
      self.__reusePayload()
    GPIO.output(CE_Pin, True)
    GPIO.output(CE_Pin, False)
    GPIO.wait_for_edge(IRQ_Pin, GPIO.FALLING, timeout=100)
    s = self.__status()
    if (s & (1<<bitStatusMaxRt)):
      self.__flushTx()
      self.__flushRx()
      self.nrfClearIRQs(True, True, True)
      return (False, [])
    elif (s & (1<<bitStatusTxDs)):
      if (s & (1<<bitStatusRxDr)):
        ackPlLen = self.__readPayloadWidth()
        if ackPlLen > 32:
          self.__flushTx()
          self.__flushRx()
          self.nrfClearIRQs(True, True, True)
          return (False, []) 
        ackPl = self.__readPayload(ackPlLen)
      else:
        ackPl = []
      self.nrfClearIRQs(True, True, True)
      return (True, ackPl)
    else:
      raise RuntimeError("RF module did not respond to send, something went wrong, sorry...")
