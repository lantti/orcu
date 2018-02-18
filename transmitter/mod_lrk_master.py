import orcumaster
import orcuui
import sys
import csv

class lrkmaster(orcumaster.orcumaster):
  rawPipe = 1
  gbrPipe = 2
  indexedPipe = 3
  confPipe = 4

#  def __init__(self):
#    super().__init__()
#    return
#
#  def __enter__(self):
#    super().__enter__()
#    return self
  def lrkIndexed(self, row, column, colour1, colour2):
    if colour1 > 15 or colour2 > 15 or colour1 < 0 or colour2 < 0:
      raise ValueError('Invalid colour index')
    cbyte = (colour2<<4) | colour1
    if (self.nrfGetDynamicPayloadEnable(False) == True):
      self.nrfSetDynamicPayloadEnable(False)
    return self.ocSend(row, column, self.indexedPipe, [cbyte])

  def lrkGbr(self, row, column, g1, b1, r1, g2, b2, r2):
    clist = [g1, g2, b1, b2, r1, r2]
    if (self.nrfGetDynamicPayloadEnable(False) == True):
      self.nrfSetDynamicPayloadEnable(False)
    return self.ocSend(row, column, self.gbrPipe, clist)

  def lrkPalette(self, row, column, palette):
    resp = self.ocProgramEeprom(row, column, 3, palette[:24])
    if resp[0]:
      return self.ocProgramEeprom(row, column, 27, palette[24:])
    else:
      return resp

manualText = ['Lrk2', '', 'Commands:', 'q[uit]', 's[elect] <row>', 
              'u[nselect] <row>', 'p[ing]', 'i[ndexed] <index> <index>',
              'g[br] <g> <b> <r> <g> <b> <r>', 'l[oad] <filename> [nocal]', 'pa[lette]']

channel = int(sys.argv[1])
palette = []

print("Starting, please wait...")

with orcuui.OrcuUI(['q','quit']) as ui, lrkmaster(channel) as lrk:
  def pingCb(ui, cmdline, selected):
    for u in selected:
      row = u[0]
      column = u[1]
      ui.writeResult('Pinging ' + str((row,column)))
      ui.unmarkUnit(row, column)
      ping = lrk.ocPing(row, column)
      if ping[0]:
        splitPing = ''.join(ping[1]).split('\x00')
        ui.markUnit(row, column, splitPing[1] + '/' + splitPing[0])
        ui.writeResult('Pinging ' + str((row,column)) + '...Ok!')
      else:
        ui.writeResult('Pinging ' + str((row,column)) + '...Fail!')


  def rowSelectCb(ui, cmdline, selected):
    try:
      row = int(cmdline.split()[1])
      if cmdline[0] == 's':
        f = ui.selectUnit
      else:
        f = ui.unselectUnit
      for column in range(0,256):
        f(row,column)
      ui.writeResult('')
    except ValueError:
      ui.writeResult('Please supply a valid row number')

  def loadCb(ui, cmdline, selected):
    global palette
    cb = [x**2/600 for x in range(0,256)]
    cg = [x**2/400 for x in range(0,256)]
    cr = [x**2/255 for x in range(0,256)]
    cmdwords = cmdline.split()
    if len(cmdwords) > 2 and cmdwords[2] == 'nocal':
      calibrate = False
    else:
      calibrate = True 
    try:
      filename = cmdwords[1]
      with open(filename, newline='') as f:
        r = csv.reader(f)
        gbr = [[int(c[1], 0),int(c[2], 0),int(c[0], 0)] for c in r if c]
      if calibrate:
        gbr = [[cg[c[0]],cb[c[1]],cr[c[2]]] for c in gbr]
      palette = list(bytes([round(i) for c in gbr for i in c]))
      ui.writeResult('Loaded ' + filename)
    except (IndexError, ValueError, FileNotFoundError):
      ui.writeResult('Please supply a valid palette file')

  def paletteCb(ui, cmdline, selected):
    for u in selected:
      row = u[0]
      column = u[1]
      ui.writeResult('Setting palette for ' + str((row,column)))
      (success,_) = lrk.lrkPalette(row, column, palette)
      if success:
        ui.writeResult('Setting palette for ' + str((row,column)) + '...Ok!')
      else:
        ui.writeResult('Setting palette for ' + str((row,column)) + '...Fail!')


  def indexedCb(ui, cmdline, selected):
    cmdwords = cmdline.split()
    try:
      c1 = int(cmdwords[1])
      c2 = int(cmdwords[2])
      for u in selected:
        row = u[0]
        column = u[1]
        ui.writeResult('Setting colour index ' + str((c1,c2)) + ' for ' + str((row,column)))
        (success,_) = lrk.lrkIndexed(row, column, c1, c2)
        if success:
          ui.writeResult('Setting colour index ' + str((c1,c2)) + ' for ' + str((row,column)) + '...Ok!')
        else:
          ui.writeResult('Setting colour index ' + str((c1,c2)) + ' for ' + str((row,column)) + '...Fail!')
    except (ValueError, IndexError):
      ui.writeResult('Please give two valid colour indices')

  def gbrCb(ui, cmdline, selected):
    cmdwords = cmdline.split()
    try:
      g1 = int(cmdwords[1])
      b1 = int(cmdwords[2])
      r1 = int(cmdwords[3])
      g2 = int(cmdwords[4])
      b2 = int(cmdwords[5])
      r2 = int(cmdwords[6])
      for u in selected:
        row = u[0]
        column = u[1]
        ui.writeResult('Setting gbr colour ' + str((g1,b1,r1,g2,b2,r2)) + ' for ' + str((row,column)))
        (success,_) = lrk.lrkGbr(row, column, g1, b1, r1, g2, b2, r2)
        if success:
          ui.writeResult('Setting gbr colour ' + str((g1,b1,r1,g2,b2,r2)) + ' for ' + str((row,column)) + '...Ok!')
        else:
          ui.writeResult('Setting gbr colour ' + str((g1,b1,r1,g2,b2,r2)) + ' for ' + str((row,column)) + '...Fail!')
    except (ValueError, IndexError):
      ui.writeResult('Please give six valid component values')


  ui.registerCallback('ping', pingCb)
  ui.registerCallback('p', pingCb)
  ui.registerCallback('select', rowSelectCb)
  ui.registerCallback('s', rowSelectCb)
  ui.registerCallback('unselect', rowSelectCb)
  ui.registerCallback('u', rowSelectCb)
  ui.registerCallback('indexed', indexedCb)
  ui.registerCallback('i', indexedCb)
  ui.registerCallback('gbr', gbrCb)
  ui.registerCallback('g', gbrCb)
  ui.registerCallback('load', loadCb)
  ui.registerCallback('l', loadCb)
  ui.registerCallback('palette', paletteCb)
  ui.registerCallback('pa', paletteCb)
  ui.setManualText(manualText)
  ui.start()

print("Bye!")





#def opFlash(row, column, hexfilename)
#  with orcumaster.orcumaster(channel) as orcu, open(hexfilename, 'r') as hexfile:
#    ih = IntelHex(hexfile)

