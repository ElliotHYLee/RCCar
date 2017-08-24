CON
_clkmode = xtal1 + pll16x
_xinfreq = 6_250_000
  xb = 1

VAR

  ' state vars
  long eAnglePtr[3], gyroPtr[3], accPtr[3], magPtr[3]
  long dist_ground_ptr, lcPtr[3] 'lc = local coordinate pointer   

  ' actuator vars
  long pulsePtr[6], throttlePtr

  ' control references
  long refAttPtr[3], refNavPtr[3]
  
  ' pid vars
  long attPidPtr[22], navPidPtr[22]
  
  ' parsing
  byte GCSByte


  ' cmd for ctrl vars
  long attPidConstPtr[10], attPidOnOffPtr[2], attPidCtrlRefPtr[4]
  long navPidConstPtr[10], navPidOnOffPtr[2], navPidCtrlRefPtr[4]
  long motor_inputPtr[7], throttle_inputPtr[2], sys_modePtr[2]


  'cmd variable from GCS, and for ACK purpose
  long cmdGCS, cmdAck, infoChoice, chkList[32]

  'sd card variable
  long SDStatus, SDCogStack[500], SDCmd

  'voltage monitor
  long ping[6], cell[6], pin[3], lp[6]


  long sysModeInternalPtr
  
OBJ
  com :  "fullDuplexSerial4port_tier2"
  sd  :  "SDCard_tier3"

PUB main | isReceived, c, localCoordinate[3]



  lcPtr[0] := @localCoordinate[0] 
  lcPtr[1] := @localCoordinate[1]
  lcPtr[2] := @localCoordinate[2]  

  initialize

  repeat
    if com.rxIsIn(xb)
      c := com.charIn(xb)
      com.char(xb, c)
      com.newline(xb)

PUB getinfoChoice
  return infoChoice

PUB initialize
  com.initialize

  cmdAck := false
  infoChoice := 255
  updateInfoSelection(infoChoice)

  
  pin[0] := 13
  pin[1] := 12
  pin[2] := 11
  ' SD Card Part
  
  cognew(driveSDCog, @SDCogStack)  
  
PUB setCmdAck(val)

  cmdAck := val

pub setSysModeInternal(val)
  sysModeInternalPtr := val
  
PUB setLocalCoordinate(val)
  lcPtr := val

PUB setDistPtr(val)
  dist_ground_ptr := val

PUB setAttPtr(euler, gyro, acc, mag, distGround)
  eAnglePtr := euler
  gyroPtr   := gyro
  accPtr    := acc
  magPtr    := mag
  dist_ground_ptr := distGround

PUB setThrottle(valuePtr)
  throttlePtr := valuePtr
    
PUB setMotPtr(pwmPtr) | i
  repeat i from 0 to 5
    pulsePtr[i] := pwmPtr[i]
    i++           

PUB setTargetAttitude(valuePtr)
  refAttPtr := valuePtr
    
PUB setAttPid(val)
  attPidPtr := val

PUB setTargetLocalCoordinate(valuePtr)
  refNavPtr := valuePtr
  
PUB setNavPid(val)
  navPidPtr := val

PUB setCmdInput(val1, val2, val3, val4, val5, val6, val7, val8, val9)

'(@attPidConst, @attPidOnOff, @attPidCtrlRef,@navPidConst, @navPidOnOff, @navPidCtrlRef, @throttle_input, @sys_mode) 
  attPidConstPtr := val1
  attPidOnOffPtr := val2
  attPidCtrlRefPtr := val3
   
  navPidConstPtr := val4
  navPidOnOffPtr := val5
  navPidCtrlRefPtr := val6
   
  motor_inputPtr := val7
  throttle_inputPtr := val8
  sys_modePtr := val9


'=================================
' Main Loop
'=================================         
PUB communicate | base , c, turnCase, msgElementCnt
  base := cnt
  turnCase := 0
  msgElementCnt := 0
  repeat
    if com.rxIsIn(xb) AND (msgElementCnt =< 10 )
      readCharArray_xb
      msgElementCnt++
      
    else
      msgElementCnt := 0
      'base := cnt 
      'sendSDMsg
      if (cnt > (base + clkfreq))  'send voltage message once a second
        sendVoltMsg
        base := cnt
    
      case turnCase
        0 : sendEulerMsg
        1 : sendGyroMsg
        2 : sendAccMsg
        3 : sendMagMsg
        4 : sendMotorMsg
        5 : sendThrottleMsg
        6 : sendDistGrdMsg
        7 : sendLocalCoordinate(xb)
        8 : sendAttPidCalc
        9 : sendAttCtrlRef
        10: sendNavPidCalc
        11: sendNavCtrlRef
        12: sendSysMode   
      turnCase++

      if (turnCase => 13)
        turnCase := 0       
           
      ' seldomly sended by condition
      if (cmdAck == 1)
        sendPidAttOnOff
        cmdAck := 0
       
      if (cmdAck == 2)
        repeat 2
          sendAttPidConst
        cmdAck := 0
       
      if (cmdAck == 3)
        sendPidNavOnOff
        cmdAck := 0
       
      if (cmdGCS == 1)
        sendInfoChoice
        cmdGCS := 0  
       
      if (cmdGCS == 2)
        sendNavPidConst
        cmdGCS := 0 
        
      if (cmdGCS == 3)
        sendPidNavOnOff
        cmdGCS := 0  
       
      if (cmdGCS == 12)
        sendPidAttOnOff
        cmdGCS := 0
       
      if (cmdGCS == 11)
        repeat 2
          sendAttPidConst
        cmdGCS := 0
       
      'if ( ||(cnt - base) >  clkfreq/100)
           
                           
            

PUB readCharArray_xb        

  'com.char(xb, com.charIn(xb))

  case consume
    "M" : getMotors
    "D" : getSysMode
    "T" : getThrottle
    "o" : getPIDAttOnOff
    "O" : getPIDNavOnOff
    "p" : getPIDAttConst
    "P" : getPIDNavConst
    "I" : getInfoSelection
    "R" : getGCSCommand
    "C" : getAttCtrlRef
    "N" : getNavCtrlRef
    "B" : getSDCommand 
     
     
PUB consume  | tick
  result := GCSByte
  GCSByte := com.CharIn(xb)
  tick := cnt
  repeat while(cnt =< tick + clkfreq/1000*20)
  'It needs to pause for a while in case of xbee is slow and next buffer can be temporarily empty
  ' , which doesn't mean the end of data

PUB getAttCtrlRef| theLetter
  theLetter := consume
  long[attPidCtrlRefPtr][theLetter - "A" + 1] := getDec   
  long[attPidCtrlRefPtr][0] := 1        'producer flag up: ready to be comsumed

PUB getNavCtrlRef| theLetter, index
  theLetter := consume
  index :=  theLetter - "A" + 1
  if (index==1)
    long[navPidCtrlRefPtr][index] := 1500 #> getDec <#3000
    long[navPidCtrlRefPtr][0] := 1        'producer flag up: ready to be comsumed 
  elseif (index==2)
    long[navPidCtrlRefPtr][index] := -500 #> getDec <#500  
    long[navPidCtrlRefPtr][0] := 2        'producer flag up: ready to be comsumed 
  elseif (index==3)
    long[navPidCtrlRefPtr][index] := -500 #> getDec <#500  
    long[navPidCtrlRefPtr][0] := 3        'producer flag up: ready to be comsumed

      
PUB getInfoSelection  
  infoChoice := getDec
  updateInfoSelection(infoChoice)
  
PUB updateInfoSelection(val) | i, temp
  temp := val
  repeat i from 0 to 31
    chkList[i] := temp & %1
    temp := temp >> 1

PUB getSDCommand

  SDCmd := getDec

PUB getGCSCommand 
  cmdGCS := getDec

PRI getPIDAttConst | theLetter
  theLetter := consume
  long[attPidConstPtr][theLetter - "A" + 1] := getDec  
  long[attPidConstPtr][0] := 1 
  
PRI getPIDAttOnOff 

  long[attPidOnOffPtr][1] := getDec
  long[attPidOnOffPtr][0] := 1
  
PRI getPIDNavConst | theLetter 
  theLetter := consume
  long[navPidConstPtr][theLetter - "A" + 1] := getDec
  long[navPidConstPtr][0] := 1 
  
PRI getPIDNavOnOff 

  long[navPidOnOffPtr][1] := getDec
  long[navPidOnOffPtr][0] := 1


PRI getSysMode
  long[sys_modePtr][1] := getDec
  long[sys_modePtr][0] := 1        'raise producer flag   
  long[throttle_inputPtr][0] := 0  ' make sure throttle is NOT to be updated
  long[motor_inputPtr][0] := 0     ' make sure mottor is NOT to be updated 

  if long[sys_modePtr][1] == 0    ' if idle mode
    long[attPidOnOffPtr][1] := 0  ' all att pid=off
  else
    long[attPidOnOffPtr][1] := 7  ' else all att pid=on

  cmdAck := 1
  
  
  'long[attPidOnOffPtr][0] := 1
    ' don't turn on pidonOffheader sys_mode should take care of it.
  ' just update pidOnOffstatus.. 
  
PRI getThrottle | theLetter
  theLetter := consume
  if (theLetter == "H")
    long[throttle_inputPtr][1] := getDec
    long[throttle_inputPtr][0] := 1
  
PRI getMotors | theLetter, i, temp
  theLetter := consume
  temp := getDec 
  if (theLetter == "g")
    repeat i from 0 to 5
      long[motor_inputPtr][i+1] := 1000 #> long[pulsePtr][i] + temp <#2100
  else
    long[motor_inputPtr][theLetter - "a" + 1] := 1000 #> long[pulsePtr][theLetter - "a" ] + temp  <#2100 
    
  long[motor_inputPtr][0] := 1
  
PRI getDec | value, sign, answer

  value := 0
  if (GCSByte == "-")
    sign := -1
    consume
  elseif ((GCSByte => "0") and (GCSByte =< "9"))
    sign := 1
    value := consume - "0"

  repeat 
    if ((GCSByte => "0") and (GCSByte =< "9"))
      value *= 10
      value += consume - "0"
    else
      quit
  answer := sign*value
  return answer

PRI sendSysMode
if (chkList[12]) 
  com.str(xb, string("[SM"))
  com.dec(xb, long[sysModeInternalPtr])
  com.str(xb, string("]"))
  
PUB sendPidAttOnOff

  com.str(xb, string("[o"))
  com.dec(xb, long[attPidPtr][0])
  com.str(xb, string("]"))

PUB sendLocalCoordinate(port)

if (chkList[6]) 
  com.str(port, string("[lx"))
  com.dec(port, long[lcPtr][0])
  com.str(port, string("]"))

  com.str(port, string("[ly"))
  com.dec(port, long[lcPtr][1])
  com.str(port, string("]"))

  com.str(port, string("[lz"))
  com.dec(port, long[lcPtr][2])
  com.str(port, string("]"))


PRI sendVoltMsg | i
   
  repeat i from 0 to 5
   'motor write
    com.str(xb, String("[v"))
    com.Dec(xb, i+1)
    com.Dec(xb, cell[i])
    com.str(xb, String("]"))

PRI sendThrottleMsg

if (chkList[5]==1)
  com.str(xb, String("[t"))
  com.dec(xb, 0)
  com.dec(xb, long[throttlePtr])
  com.str(xb, String("]"))

PRI sendMotorMsg | i
if (chkList[4]==1)
  repeat i from 0 to 5
    'motor write
    com.str(xb, String("[m"))
    com.Dec(xb, i+1)
    com.Dec(xb, long[pulsePtr][i])
    com.str(xb, String("]"))

PRI sendDistGrdMsg
if (chkList[7]==1)
  com.str(xb, String("[dg"))
  com.dec(xb, long[dist_ground_ptr])
  com.str(xb, String("]"))         

PRI sendEulerMsg | i
if (chkList[0]==1)
  repeat i from 0 to 2
    com.str(xb, String("[c"))
    case i
      0: com.str(xb, String("x"))
      1: com.str(xb, String("y"))
      2: com.str(xb, String("z"))
    com.dec(xb, long[eAnglePtr][i])
    com.str(xb, String("]"))

PRI sendAccMsg | i
if (chkList[2]==1)
  repeat i from 0 to 2
    com.str(xb, String("[a"))
    case i
      0: com.str(xb, String("x"))
      1: com.str(xb, String("y"))
      2: com.str(xb, String("z"))
    com.dec(xb, long[accPtr][i])
    com.str(xb, String("]"))

PRI sendGyroMsg | i
if (chkList[1]==1)
  repeat i from 0 to 2 
    com.str(xb,String("[g"))
    case i
      0: com.str(xb,String("x"))
      1: com.str(xb,String("y"))
      2: com.str(xb,String("z")) 
    com.dec(xb,long[gyroPtr][i])
    com.str(xb,String("]"))

PRI sendMagMsg | i
if (chkList[3]==1)
  repeat i from 0 to 2
    com.str(xb,String("[q"))
    case i
      0: com.str(xb,String("x"))
      1: com.str(xb,String("y"))
      2: com.str(xb,String("z")) 
    com.dec(xb,long[magPtr][i])
    com.str(xb,String("]"))
                
PRI sendInfoChoice

  com.str(xb, string("[i"))
  com.dec(xb, infoChoice)
  com.str(xb, string("]"))

PRI sendAttCtrlRef | i

if (chkList[10]==1)
  repeat i from 0 to 2
    com.str(xb,String("[s"))
    case i
      0: com.str(xb,String("x"))
      1: com.str(xb,String("y"))
      2: com.str(xb,String("z")) 
    com.dec(xb,long[refAttPtr][i])
    com.str(xb,String("]"))

PRI sendNavCtrlRef | i

if (chkList[11]==1)
  repeat i from 0 to 2
    com.str(xb,String("[n"))
    case i
      0: com.str(xb,String("x"))
      1: com.str(xb,String("y"))
      2: com.str(xb,String("z")) 
    com.dec(xb,long[refNavPtr][i])
    com.str(xb,String("]"))


PRI sendAttPidConst 

  'p0 = Kp of x axis
  'p1 = Ki of x axis
  'p3 = Kd of x axis
  'p4 = Kp of y axis

  com.str(xb, String("[p0"))
  com.dec(xb, long[attPidPtr][1])
  com.str(xb, String("]"))
  com.str(xb, String("[p1"))
  com.dec(xb, long[attPidPtr][2])
  com.str(xb, String("]"))
  com.str(xb, String("[p2"))
  com.dec(xb, long[attPidPtr][3])
  com.str(xb, String("]"))
 
  com.str(xb, String("[p3"))
  com.dec(xb, long[attPidPtr][8])
  com.str(xb, String("]"))
  com.str(xb, String("[p4"))
  com.dec(xb, long[attPidPtr][9])
  com.str(xb, String("]"))
  com.str(xb, String("[p5"))
  com.dec(xb, long[attPidPtr][10])
  com.str(xb, String("]"))  

  com.str(xb, String("[p6"))
  com.dec(xb, long[attPidPtr][15])
  com.str(xb, String("]"))
  com.str(xb, String("[p7"))
  com.dec(xb, long[attPidPtr][16])
  com.str(xb, String("]"))
  com.str(xb, String("[p8"))
  com.dec(xb, long[attPidPtr][17])
  com.str(xb, String("]"))     

PRI sendAttPidCalc
if (chkList[8]==1)
  com.str(xb, String("[k0"))
  com.dec(xb, long[attPidPtr][4])
  com.str(xb, String("]"))
  com.str(xb, String("[k1"))
  com.dec(xb, long[attPidPtr][5])
  com.str(xb, String("]"))
  com.str(xb, String("[k2"))
  com.dec(xb, long[attPidPtr][6])
  com.str(xb, String("]"))
  com.str(xb, String("[k3"))
  com.dec(xb, long[attPidPtr][7])
  com.str(xb, String("]"))

  com.str(xb, String("[k4"))
  com.dec(xb, long[attPidPtr][11])
  com.str(xb, String("]"))
  com.str(xb, String("[k5"))
  com.dec(xb, long[attPidPtr][12])
  com.str(xb, String("]"))
  com.str(xb, String("[k6"))
  com.dec(xb, long[attPidPtr][13])
  com.str(xb, String("]"))
  com.str(xb, String("[k7"))
  com.dec(xb, long[attPidPtr][14])
  com.str(xb, String("]"))  

  com.str(xb, String("[k8"))
  com.dec(xb, long[attPidPtr][18])
  com.str(xb, String("]"))
  com.str(xb, String("[k9"))
  com.dec(xb, long[attPidPtr][19])
  com.str(xb, String("]"))
  com.str(xb, String("[kA"))
  com.dec(xb, long[attPidPtr][20])
  com.str(xb, String("]"))
  com.str(xb, String("[kB"))
  com.dec(xb, long[attPidPtr][21])
  com.str(xb, String("]"))


PUB sendPidNavOnOff

  com.str(xb, string("[O"))
  com.dec(xb, long[navPidPtr][0])
  com.str(xb, string("]"))    

PRI sendNavPidConst 

  'p0 = Kp of x axis
  'p1 = Ki of x axis
  'p3 = Kd of x axis
  'p4 = Kp of y axis

  com.str(xb, String("[P0"))
  com.dec(xb, long[navPidPtr][1])
  com.str(xb, String("]"))
  com.str(xb, String("[P1"))
  com.dec(xb, long[navPidPtr][2])
  com.str(xb, String("]"))
  com.str(xb, String("[P2"))
  com.dec(xb, long[navPidPtr][3])
  com.str(xb, String("]"))
 
  com.str(xb, String("[P3"))
  com.dec(xb, long[navPidPtr][8])
  com.str(xb, String("]"))
  com.str(xb, String("[P4"))
  com.dec(xb, long[navPidPtr][9])
  com.str(xb, String("]"))
  com.str(xb, String("[P5"))
  com.dec(xb, long[navPidPtr][10])
  com.str(xb, String("]"))  

  com.str(xb, String("[P6"))
  com.dec(xb, long[navPidPtr][15])
  com.str(xb, String("]"))
  com.str(xb, String("[P7"))
  com.dec(xb, long[navPidPtr][16])
  com.str(xb, String("]"))
  com.str(xb, String("[P8"))
  com.dec(xb, long[navPidPtr][17])
  com.str(xb, String("]")) 
  com.str(xb, String("[P9"))
  com.dec(xb, long[navPidPtr][18])
  com.str(xb, String("]"))
  com.str(xb, String("[PA"))
  com.dec(xb, long[navPidPtr][19])
  com.str(xb, String("]")) 

  
PRI sendNavPidCalc
if (chkList[9]==1)
  com.str(xb, String("[K0"))
  com.dec(xb, long[navPidPtr][4])
  com.str(xb, String("]"))
  com.str(xb, String("[K1"))
  com.dec(xb, long[navPidPtr][5])
  com.str(xb, String("]"))
  com.str(xb, String("[K2"))
  com.dec(xb, long[navPidPtr][6])
  com.str(xb, String("]"))
  com.str(xb, String("[K3"))
  com.dec(xb, long[navPidPtr][7])
  com.str(xb, String("]"))

  com.str(xb, String("[K4"))
  com.dec(xb, long[navPidPtr][11])
  com.str(xb, String("]"))
  com.str(xb, String("[K5"))
  com.dec(xb, long[navPidPtr][12])
  com.str(xb, String("]"))
  com.str(xb, String("[K6"))
  com.dec(xb, long[navPidPtr][13])
  com.str(xb, String("]"))
  com.str(xb, String("[K7"))
  com.dec(xb, long[navPidPtr][14])
  com.str(xb, String("]"))  

  com.str(xb, String("[K8"))
  com.dec(xb, long[navPidPtr][20])
  com.str(xb, String("]"))
  com.str(xb, String("[K9"))
  com.dec(xb, long[navPidPtr][21])
  com.str(xb, String("]"))
  com.str(xb, String("[KA"))
  com.dec(xb, long[navPidPtr][22])
  com.str(xb, String("]"))
  com.str(xb, String("[KB"))
  com.dec(xb, long[navPidPtr][23])
  com.str(xb, String("]"))


PUB sendSDMsg

  com.str(xb, String("sdcmd: "))
  com.dec(xb, SDcmd)
  com.str(xb, String("    "))
  com.str(xb, String("sdStatus: "))
  com.dec(xb, SDStatus)
  com.str(xb, String("    ")) 

  
'=====================================================================================================  
'=====================================================================================================
'==================         SDCard Region        =====================================================
'=====================================================================================================
'=====================================================================================================
PUB driveSDCog 
  repeat
    'updateVoltage       
    if SDcmd == 1 AND SDStatus < 1
      sd.openSD
      repeat while SDStatus < 1
        SDStatus := sd.getSDStatus
    if SDcmd == 1 AND SDStatus > 0
      writeData
    if SDcmd == 0 AND SDStatus == 1
      sd.closeSD
      SDStatus := sd.getSDStatus


PUB writeData
  sd.writeSD_startLine(cnt)

  'IMU vars
  sd.writeSD(long[gyroPtr][0], long[gyroPtr][1], long[gyroPtr][2])
  sd.writeSD(long[accPtr][0],  long[accPtr][1],  long[accPtr][2])
  sd.writeSD(long[magPtr][0],  long[magPtr][1],  long[magPtr][2])
  sd.writeSD(long[eAnglePtr][0],  long[eAnglePtr][1],  long[eAnglePtr][2])                                                                 

  ' pos-ctrl vars
  sd.writeSD(long[lcPtr][0],  long[lcPtr][1],  long[lcPtr][2])
  sd.writeSD(long[refNavPtr][0],  long[refNavPtr][1],  long[refNavPtr][2])

  
  'ctrl vars
  sd.writeSD(long[attPidCtrlRefPtr][1],  long[attPidCtrlRefPtr][2],  long[attPidCtrlRefPtr][3])
  sd.writeSD(long[refAttPtr][0],  long[refAttPtr][1],  long[refAttPtr][2])

  'motors
  sd.writeSD(long[pulsePtr][0],  long[pulsePtr][1],  long[pulsePtr][2])
  sd.writeSD(long[pulsePtr][3],  long[pulsePtr][4],  long[pulsePtr][5])
  'sd.writeSD(long[throttlePtr])
   
  sd.writeSD_endLine


pri updateVoltage

  ping[0] := pulse_in(pin[0])
  ping[1] := pulse_in(pin[1])
  if ping[0] > 7 AND ping[0] < 13
    cell[0] := lowPass(0, ping[1])
  if ping[0] > 17 AND ping[0] < 23
    cell[1] := lowPass(1, ping[1]) 
  if ping[0] > 27 AND ping[0] < 33
    cell[2] := lowPass(2, ping[1]) 
  if ping[0] > 37 AND ping[0] < 43
    cell[3] := lowPass(3, ping[1]) 
  if ping[0] > 47 AND ping[0] < 53
    cell[4] := lowPass(4, ping[1]) 
  if ping[0] > 57 AND ping[0] < 63
    cell[5] := lowPass(5, ping[1])  


pri lowPass(index, val) | ratio

  ratio := 70
  lp[index] := (ratio*lp[index]+50)/100 + ((100-ratio)*val+50)/100 
  return lp[index]            

pri pulse_in(pinNumber) | mask, input

  mask := 1 << pinNumber                                        ' mask for pin
  frqa := 1                                                     ' set for high-resolution measurement
  ctra := (%01000 << 26) | pinNumber                            ' set ctra to POS detect on pin   
  waitpne(mask, mask, 0)                                        ' wait for pin to be low
  phsa := 0                                                     ' clear accumulator
  waitpeq(mask, mask, 0)                                        ' wait for pin to go high
  waitpne(mask, mask, 0)                                        ' wait for pin to return low
  input := phsa / (clkfreq / 100_000)                           ' convert ticks to percentage
  return  input          










       