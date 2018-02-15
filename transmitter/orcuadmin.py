import orcumaster
import orcuui
import sys
import intelhex


PAGECOUNT = 64
PAGESIZE = 64
MAXPAYLOAD = 28
manualText = ['Orcu Admin', '', 'Commands:', 'q[uit]', 's[elect] <row>', 
              'u[nselect] <row>', 'p[ing]',
              'a[ddress] <row> <column> <channel>',
              'l[oad] <filename>', 'f[lash]']

channel = int(sys.argv[1])
ih = intelhex.IntelHex()

def splitSegments(segList):
  pageLists = [[0] for _ in range(PAGECOUNT)] 
  for s in segList:
    sBeg = s[0]
    sEnd = s[1]
    while (sBeg < sEnd):
      page = sBeg // PAGESIZE
      pageEnd = (page + 1) * PAGESIZE
      plEnd = sBeg + MAXPAYLOAD
      newEnd = min(pageEnd, plEnd, sEnd)
      pageLists[page][0] += newEnd - sBeg
      pageLists[page].append((sBeg,newEnd))
      sBeg = newEnd
  return pageLists

print("Starting, please wait...")

with orcuui.OrcuUI(['q','quit']) as ui, orcumaster.orcumaster(channel) as orcu:
  def echoCb(ui, cmdline, selected):
    ui.writeResult('Echo: ' + cmdline + ' ' + str(selected))

  def loadCb(ui, cmdline, selected):
    global ih
    try:
      filename = cmdline.split()[1]
      ih = intelhex.IntelHex()
      ih.loadhex(filename)
      ui.writeResult('Loaded ' + filename)
    except (IndexError, FileNotFoundError, intelhex.IntelHexError):
      ui.writeResult('Please supply a valid Intel Hex file')

  def pingCb(ui, cmdline, selected):
    for u in selected:
      row = u[0]
      column = u[1]
      ui.writeResult('Pinging (' + str(row) + ',' + str(column) + ')')
      ui.unmarkUnit(row, column)
      ping = orcu.ocPing(row, column)
      if ping[0]:
        splitPing = ''.join(ping[1]).split('\x00')
        ui.markUnit(row, column, splitPing[1] + '/' + splitPing[0])
        ui.writeResult('Pinging (' + str(row) + ',' + str(column) + ')...Ok!')
      else:
        ui.writeResult('Pinging (' + str(row) + ',' + str(column) + ')...Fail!')


  def rowSelectCb(ui, cmdline, selected):
    row = int(cmdline.split()[1])
    if cmdline[0] == 's':
      f = ui.selectUnit
    else:
      f = ui.unselectUnit
    for column in range(0,256):
      f(row,column)
    ui.writeResult('')

  def flashCb(ui, cmdline, selected):
    pages = splitSegments(ih.segments())
    for u in selected:
      row = u[0]
      column = u[1]
      fail = False
      wrote = False
      ui.unmarkUnit(row, column)
      ui.writeResult('Flashing (' + str(row) + ',' + str(column) + ')')
      for p in range(PAGECOUNT):
        if (pages[p][0] != 0):
          if (pages[p][0] != PAGESIZE):
            (success, _) = orcu.ocLoadPage(row, column, p)
            fail = fail or not success
          for s in pages[p][1:]:
            (success, _) = orcu.ocWriteBuffer(row, column, s[0]-p*PAGESIZE, list(ih[s[0]:s[1]].tobinarray()))
            fail = fail or not success
          (success, _) = orcu.ocProgramPage(row, column, p)
          fail = fail or not success
          wrote = True
      orcu.ocForceReset(row, column)
      if fail:
        if wrote:
          ui.writeResult('Flashing (' + str(row) + ',' + str(column) + ')...Fail! (Bricked)')
        else:
          ui.writeResult('Flashing (' + str(row) + ',' + str(column) + ')...Fail!')
      else:
        ui.writeResult('Flashing (' + str(row) + ',' + str(column) + ')...Ok!')

  ui.registerCallback('echo', echoCb)
  ui.registerCallback('e', echoCb)
  ui.registerCallback('ping', pingCb)
  ui.registerCallback('p', pingCb)
  ui.registerCallback('select', rowSelectCb)
  ui.registerCallback('s', rowSelectCb)
  ui.registerCallback('unselect', rowSelectCb)
  ui.registerCallback('u', rowSelectCb)
  ui.registerCallback('load', loadCb)
  ui.registerCallback('l', loadCb)
  ui.registerCallback('flash', flashCb)
  ui.registerCallback('f', flashCb)
  ui.setManualText(['Orcu Admin', '', 'Commands:', 'q[uit]', 's[elect] <row>', 'u[nselect] <row>', 'p[ing]', 'l[oad] <filename>', 'f[lash]'])
  ui.start()

print("Bye!")





#def opFlash(row, column, hexfilename)
#  with orcumaster.orcumaster(channel) as orcu, open(hexfilename, 'r') as hexfile:
#    ih = IntelHex(hexfile)

