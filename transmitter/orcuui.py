import curses

class OrcuUI():
  selectX = 64
  linesPerRow = -(-256 // selectX) + 1
  resultY = 1
  promptY = 1
  defaultIcon = '.'
  def __init__(self, quitCommands):
    self.__mainWindow = None
    self.__manualWindow = None
    self.__legendWindow = None
    self.__selectPad = None
    self.__resultWindow = None
    self.__promptWindow = None
    self.__selectTLY = 0
    self.__selectTLX = 0
    self.__selectBRY = 0
    self.__selectBRX = 0
    self.__selectView = 0
    self.__quitCommands = quitCommands
    self.__commandCallbacks = {}
    self.__sCursorRow = 0
    self.__sCursorColumn = 0
    self.__editline = ''
    self.__unitStatus = {}
    self.__unitMarks = []
    self.__manualLines = []

  def __enter__(self):
    self.__mainWindow = curses.initscr()
    curses.noecho()
    curses.cbreak()
    self.__makeLayout()
    return self

  def __exit__(self,et,ev,tb):
    curses.nocbreak()
    curses.echo()
    curses.endwin()
    return

  def __makeLayout(self):
    (mainY, mainX) = self.__mainWindow.getmaxyx()
    manualY = (mainY - self.resultY - self.promptY) // 2
    manualX = mainX - self.selectX - 1
    legendY = mainY - manualY - self.resultY - self.promptY - 1
    legendX = manualX
    selectY = mainY - self.resultY - self.promptY - 1
    resultX = mainX
    promptX = mainX
    self.__selectTLY = 0
    self.__selectTLX = manualX + 1
    self.__selectBRY = selectY - 1
    self.__selectBRX = manualX + self.selectX
    self.__manualWindow = curses.newwin(manualY, manualX, 0, 0)
    self.__legendWindow = curses.newwin(legendY, legendX, manualY, 0)
    self.__vSepWindow = curses.newwin(selectY, 1, 0, manualX)
    self.__selectPad = curses.newpad(1320, self.selectX)
    self.__hSepWindow = curses.newwin(1, mainX, selectY, 0)
    self.__resultWindow = curses.newwin(self.resultY, resultX, selectY + 1, 0)
    self.__promptWindow = curses.newwin(self.promptY, promptX, selectY + 1 + self.resultY, 0)

  def __drawAll(self):
    self.__drawManual()
    self.__drawLegend()
    self.__drawPrompt()
    self.__resultWindow.erase()
    self.__resultWindow.noutrefresh()
    self.__drawSelect()
    self.__drawSeparators()

  def __drawManual(self):
    (maxy,maxx) = self.__manualWindow.getmaxyx()
    self.__manualWindow.erase()
    for y in range(0, min(len(self.__manualLines), maxy)):
      self.__manualWindow.addnstr(y,0,self.__manualLines[y], maxx-1)
    self.__manualWindow.noutrefresh()

  def __drawLegend(self):
    (maxy,maxx) = self.__legendWindow.getmaxyx()
    self.__legendWindow.erase()
    for y in range(0, min(len(self.__unitMarks), maxy)):
      self.__legendWindow.addnstr(y,0,str(y+1) + '-' + self.__unitMarks[y], maxx-1)
    self.__legendWindow.noutrefresh()

  def __drawPrompt(self):
    self.__promptWindow.erase()
    self.__promptWindow.addstr(0,0,'(' + str(self.__sCursorRow) + ',' + str(self.__sCursorColumn) + ')>' + self.__editline)

  def __drawUnit(self, row, column, refresh = True):
    status = self.__unitStatus.get((row,column), [False, 0])      
    (coordY,coordX) = self.__addrToCoords(row, column)
    if status[1] > 0:
      self.__selectPad.addch(coordY, coordX, ord(hex(status[1])[-1]))
    else:
      self.__selectPad.addch(coordY, coordX, ord(self.defaultIcon))
    if (self.__sCursorRow,self.__sCursorColumn) == (row,column):
      self.__selectPad.chgat(coordY, coordX, 1, curses.A_STANDOUT)
    elif not status[0]:
      self.__selectPad.chgat(coordY, coordX, 1, curses.A_DIM)
    if refresh:
      self.__refreshSelect()

  def __drawSeparators(self):
    (my,mx) = self.__manualWindow.getmaxyx()
    self.__vSepWindow.bkgd(curses.ACS_VLINE)
    self.__hSepWindow.bkgd(curses.ACS_HLINE)
    self.__vSepWindow.erase()
    self.__hSepWindow.erase()
    self.__hSepWindow.addch(0, mx, curses.ACS_BTEE)
    self.__vSepWindow.noutrefresh()
    self.__hSepWindow.noutrefresh()
    
  def __drawSelect(self):
    self.__selectPad.bkgdset(self.defaultIcon, curses.A_DIM)
    self.__selectPad.erase()
    self.__selectPad.bkgdset(' ')
    for r in range(0,256):
      tag = '_______.o(' + str(r) + ')o._______'
      leadingWs = ' ' * ((self.selectX // 2) - (len(tag) // 2))
      trailingWs = ' ' * (self.selectX - len(tag) - len(leadingWs))
      self.__selectPad.addstr(r*self.linesPerRow, 0, leadingWs + tag + trailingWs, curses.A_NORMAL)
    for ((row,column),_) in self.__unitStatus.items():    
      self.__drawUnit(row, column, False)
    self.__drawUnit(self.__sCursorRow, self.__sCursorColumn, False)
    self.__refreshSelect()

  def __refreshSelect(self):
    self.__selectPad.refresh(self.__selectView * self.linesPerRow, 0, self.__selectTLY, self.__selectTLX, self.__selectBRY, self.__selectBRX)


  def __editSelection(self, key):
    def cUp():
      self.__sCursorColumn -= self.selectX
      if self.__sCursorColumn < 0:
        if self.__sCursorRow > 0:
          self.__sCursorColumn = 256 + self.__sCursorColumn
          self.__sCursorRow = max(self.__sCursorRow - 1, 0)
          if self.__sCursorRow < self.__selectView:
            self.__selectView = self.__sCursorRow
        else:
          self.__sCursorColumn += self.selectX

    def cDown():
      self.__sCursorColumn += self.selectX
      if self.__sCursorColumn > 255:
        if self.__sCursorRow < 255:
          self.__sCursorColumn = self.__sCursorColumn - 256
          self.__sCursorRow = min(self.__sCursorRow + 1, 255)
          rowsPerScreen = (self.__selectBRY - self.__selectTLY + 1) // self.linesPerRow
          if self.__sCursorRow >= self.__selectView + rowsPerScreen:
            self.__selectView = self.__sCursorRow - rowsPerScreen + 1
        else:
          self.__sCursorColumn -= self.selectX

    def cLeft():
      self.__sCursorColumn -= 1
      if self.__sCursorColumn < 0:
        if self.__sCursorRow > 0:
          self.__sCursorColumn = 255
          self.__sCursorRow = max(self.__sCursorRow - 1, 0)
          if self.__sCursorRow < self.__selectView:
            self.__selectView = self.__sCursorRow
        else:
          self.__sCursorColumn += 1
          

    def cRight():
      self.__sCursorColumn += 1
      if self.__sCursorColumn > 255:
        self.__sCursorColumn = 0
        self.__sCursorRow = min(self.__sCursorRow + 1, 255)
        rowsPerScreen = (self.__selectBRY - self.__selectTLY + 1) // self.linesPerRow
        if self.__sCursorRow >= self.__selectView + rowsPerScreen:
          self.__selectView = self.__sCursorRow - rowsPerScreen + 1

    def toggleUnit(row, column):
      status = self.__unitStatus.get((row,column))
      if status and status[0]:
        self.unselectUnit(row, column)
      else:
        self.selectUnit(row,column)

    oldscRow = self.__sCursorRow
    oldscColumn = self.__sCursorColumn
    if key == 'KEY_DOWN':
      cDown()
    elif key == 'KEY_UP':
      cUp()
    elif key == 'KEY_LEFT':
      cLeft()
    elif key == 'KEY_RIGHT':
      cRight()
    elif key == 'KEY_SF':
      toggleUnit(self.__sCursorRow, self.__sCursorColumn)
      cDown()
    elif key == 'KEY_SR':
      toggleUnit(self.__sCursorRow, self.__sCursorColumn)
      cUp()
    elif key == 'KEY_SLEFT':
      toggleUnit(self.__sCursorRow, self.__sCursorColumn)
      cLeft()
    elif key == 'KEY_SRIGHT':
      toggleUnit(self.__sCursorRow, self.__sCursorColumn)
      cRight()
    elif key == 'KEY_PPAGE':
      self.__selectView = max(self.__selectView - 1, 0)
      self.__sCursorRow = max(self.__sCursorRow - 1, 0)
    elif key == 'KEY_NPAGE':
      rowsPerScreen = (self.__selectBRY - self.__selectTLY + 1) // self.linesPerRow
      self.__selectView = min(self.__selectView + 1, 255 - rowsPerScreen + 1)
      self.__sCursorRow = min(self.__sCursorRow + 1, 255)
    else:
      pass
    self.__drawUnit(oldscRow, oldscColumn, False)
    self.__drawUnit(self.__sCursorRow, self.__sCursorColumn, False)
    self.__refreshSelect()

  def __sendCommand(self, command):
    if command in self.__commandCallbacks:
      selected = []
      for (addr,status) in self.__unitStatus.items():
        if status[0]:
          selected.append(addr)
      self.__commandCallbacks[command](self, self.__editline, selected)
    else:
      self.writeResult('Sorry?')
    self.__editline = ''

  def __addrToCoords(self, row, column):
    if row > 255 or column > 255 or row < 0 or column < 0:
      raise ValueError('Unit Address out of bounds')
    coordY = 1 + row * self.linesPerRow + column // self.selectX
    coordX = column % self.selectX
    return (coordY, coordX)

  def __getKey(self):
    self.__drawPrompt()
    return self.__promptWindow.getkey()

  def setManualText(self, lineslist):
    self.__manualLines = lineslist
    self.__drawManual()

  def registerCallback(self, command, callback):
    self.__commandCallbacks[command] = callback

  def unregisterCallback(self, command):
    del self.__commandCallbacks[command]

  def writeResult(self, result):
    self.__resultWindow.erase()
    self.__resultWindow.addstr(0,0,result)
    self.__resultWindow.refresh()

  def selectUnit(self, row, column):
    self.__unitStatus.setdefault((row,column), [False, 0])
    self.__unitStatus[(row,column)][0] = True
    self.__drawUnit(row,column)

  def unselectUnit(self, row, column):
    status = self.__unitStatus.get((row,column))
    if status != None:
      if status[1] != 0:
        self.__unitStatus[(row,column)][0] = False
      else:
        del self.__unitStatus[(row,column)]
      self.__drawUnit(row,column)

  def markUnit(self, row, column, mark):
    if mark not in self.__unitMarks:
      self.__unitMarks.append(mark)
      self.__drawLegend()
    i = self.__unitMarks.index(mark) + 1
    self.__unitStatus.setdefault((row,column), [False, 0])
    self.__unitStatus[(row,column)][1] = i
    self.__drawUnit(row,column)

  def unmarkUnit(self, row, column):
    status = self.__unitStatus.get((row,column))
    if status != None:
      if status[0]:
        self.__unitStatus[(row,column)][1] = 0
      else:
        del self.__unitStatus[(row,column)]
      self.__drawUnit(row,column)

  def start(self):
    self.__drawAll()
    self.__promptWindow.keypad(True)
    while True:
      key = self.__getKey()
      if key in ['\t','\r','\x0b','\x0c']:
        pass
      elif key == 'KEY_RESIZE':
        self.__makeLayout()
        self.__drawAll()
        self.__promptWindow.keypad(True)
      elif key in ['KEY_ENTER','\n']:
        command = self.__editline.split()[0]
        if command in self.__quitCommands:
          break
        self.__sendCommand(command)
      elif len(key) == 1 and key.isprintable():
        self.__editline = self.__editline + key
      elif key in ['KEY_BACKSPACE','KEY_DC']:
        self.__editline = self.__editline[:-1]
      elif key in ['KEY_DOWN','KEY_UP','KEY_LEFT','KEY_RIGHT','KEY_SF','KEY_SR','KEY_SLEFT','KEY_SRIGHT','KEY_PPAGE','KEY_NPAGE']:
        self.__editSelection(key)
      else:
        pass
    return

