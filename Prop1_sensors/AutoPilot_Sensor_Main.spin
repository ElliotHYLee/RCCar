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

  cognew(runUpdate, @updaterStack)

  startComm2
  startComm1  

PUB startComm1
  memTx1_init := cnt
  commToCtrl.TX_start(@memTx1_init,@memRx1_init, 17, 17,  6,7,8,-1)

PUB startComm2
  memTx2_init := cnt
  commToReporter.TX_start(@memTx2_init, @memRx2_init, 15, 15,  9,10,11,-1)

PUB runDebug | i
  
  debug.quickStartDebug
  repeat
    debug.clear(db)
    repeat i from 0 to 4
      debug.dec(db, memTx1_init[i])
      debug.str(db, String("    "))
      debug.dec(db, memRx1_init[i])
      debug.newline(db)
    debug.newline(db)

    repeat i from 0 to 14
      debug.dec(db, memTx2_init[i])
      debug.str(db, String("    "))
      debug.dec(db, memRx2_init[i])
      debug.newline(db)

      
    waitcnt(cnt + clkfreq/10)


PRI runUpdate | i

  base[0] := cnt
  base[1] := cnt
 
  repeat
    base[0] := cnt

    'update distance sensor
    if ((cnt - base[1] => clkfreq/160 ))   ' udate at ~160 Hz
      'distGround := getDistance_Ground
      base[1] := cnt
    
    memTx1_init[1] := long[eulerPtr][0]  
    memTx1_init[2] := long[eulerPtr][1]
    memTx1_init[3] := long[eulerPtr][2]
    memTx1_init[4] := distGround
    memTx1_init[5] := 1  
    memTx1_init[6] := 2
    memTx1_init[7] := 3
    memTx1_init[8] := 4
    memTx1_init[9] := 5  
    memTx1_init[10] := 6
    memTx1_init[11] := 7
    memTx1_init[12] := 8
    memTx1_init[13] := 9
    memTx1_init[14] := long[gyroPtr][0]
    memTx1_init[15] := long[gyroPtr][1]
    memTx1_init[16] := long[gyroPtr][2]                
    
    memTx2_init[1] := long[eulerPtr][0]  
    memTx2_init[2] := long[eulerPtr][1]
    memTx2_init[3] := long[eulerPtr][2]
    memTx2_init[4] := long[gyroPtr][0]
    memTx2_init[5] := long[gyroPtr][1]
    memTx2_init[6] := long[gyroPtr][2]
    memTx2_init[7] := long[accPtr][0]
    memTx2_init[8] := long[accPtr][1]
    memTx2_init[9] := long[accPtr][2]
    memTx2_init[10] := long[magPtr][0]
    memTx2_init[11] := long[magPtr][1]
    memTx2_init[12] := long[magPtr][2]
    memTx2_init[13] := distGround
          
    dt[0] := cnt - base[0]
    
PRI getDistance_Ground

  distFiltered := distFiltered*80/100 + ping.Millimeters(ULTRASONIC_SENSOR_PIN)*20/100

  return distFiltered
  
'===================================================================================================
'===================== ATTITUDE SENSOR PART ==================================================================
'===================================================================================================

'-----------------------------------------------------------------
'ATTITUDE SENSOR REGION                                          |
'  Number of cog used : 4                                        |
'  Sensors            : MPU9150, AK8                             |
'  Cog usage          : Reading Sensor Values                    |
'                       Calculating Complementary Filter         |
'  Functions:         : stopSensor                               |
'                       startSensor (call startSensor)           |
'                       runSensor (start MPU, AK8, MPl sensor)   |
'-----------------------------------------------------------------
  
PRI startSensor 

  DCM.masterKey_tier3
  eulerPtr := DCM.getEulerPtr
  gyroPtr  := DCM.getGyroPtr
  accPtr   := DCM.getAccPtr
  magPtr   := DCM.getMagPtr

     