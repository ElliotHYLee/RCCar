CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 6_250_000

  db = 0
  ULTRASONIC_SENSOR_PIN = 12
  
OBJ
  
  debug          : "fullDuplexSerial4port_tier2.spin"
  DCM            : "DCM_Tier3_Euler.spin" 
  ping           : "Ping.spin"

VAR

  'attitude variables
  long eulerPtr[3], gyroPtr[3], accPtr[3], magPtr[3]
  'debug
  long debuggerStack[100] 
PUB main  

  startSensor
  cognew(runDebug, @debuggerStack)

PUB runDebug

  debug.quickStartDebug
  repeat
    debug.decLn(db, long(eulerPtr))
  


PRI startSensor 

  DCM.masterKey_tier3
  eulerPtr := DCM.getEulerPtr
  gyroPtr  := DCM.getGyroPtr
  accPtr   := DCM.getAccPtr
  magPtr   := DCM.getMagPtr

     