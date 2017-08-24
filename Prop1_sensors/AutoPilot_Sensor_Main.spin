CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 6_250_000

  db = 0
  ULTRASONIC_SENSOR_PIN = 12
  
OBJ
  commToCtrl     : "Serial_mirror_full_duplex.spin"
  commToReporter : "Serial_mirror_full_duplex.spin"  
  debug          : "fullDuplexSerial4port_tier2.spin"
  DCM            : "DCM_Tier3_Euler.spin" 
  ping           : "Ping.spin"

VAR

  'attitude variables
  long eulerPtr[3], gyroPtr[3], accPtr[3], magPtr[3], distGround, distFiltered
  long memTx1_init, A[16]
  long memRx1_init, A1[16]
  long updaterStack[100]

  'attitude vars for direct to reporter
  long memTx2_init, data[14]
  long memRx2_init, data1[14]

  
  'debug
  long base[5], dt[5], debuggerStack[100] 
PUB main  

  startSensor


PRI startSensor 

  DCM.masterKey_tier3
  eulerPtr := DCM.getEulerPtr
  gyroPtr  := DCM.getGyroPtr
  accPtr   := DCM.getAccPtr
  magPtr   := DCM.getMagPtr

     