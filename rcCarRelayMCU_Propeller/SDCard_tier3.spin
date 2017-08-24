CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 6_250_000

  CS  = 8       ' Propeller Pin 3 - Set up these pins to match the Parallax Micro SD Card adapter connections.
  DI  = 11       ' Propeller Pin 2 - For additional information, download and refer to the Parallax PDF file for the Micro SD Adapter.                        
  CLK = 13       ' Propeller Pin 1 - The pins shown here are the correct pin numbers for my Micro SD Card adapter from Parallax                               
  D0  = 12       ' Propeller Pin 0 - In addition to these pins, make the power connections as shown in the following comment block.
Var

  long clock, clockLim, base, dt, flag

  long SDCmd, SDLock, insert_card

  long SDStatus

OBJ
  sd      : "SDCard_tier2"
  'debug   : "fullDuplexSerial4Port_tier2.spin"   ' If you don't already have it in your working directory, you can also download this object from OBEX.

PUB main


PUB getSDStatus

  return SDStatus

PUB openSD      
  insert_card := \sd.mount_explicit(D0, CLK, DI, CS)
  sd.popen(string("output.txt"), "w")
  if insert_card < 0 
    SDStatus := -1
  else
    SDStatus := 1


PUB closeSD
  if SDStatus >0
    sd.pclose                            
    sd.unmount
    SDStatus := 0
    
PUB writeSD(var1, var2, var3)

  if SDStatus >0
    sd.dec(var1)
    sd.str(String(" "))
    sd.dec(var2)
    sd.str(String(" "))
    sd.dec(var3)
    sd.str(String(" "))
    SDStatus := 2 
     
PUB writeSD_startLine(var1)

  if SDStatus>0 
    sd.dec(var1)
    sd.str(String(" "))
    SDStatus := 2
     
PUB writeSD_endLine
  if SDStatus >0 
    sd.newline 
    SDStatus := 2


   