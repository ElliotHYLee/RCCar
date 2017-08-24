CON
  _clkmode = xtal1 + pll16x                                                    
  _xinfreq = 6_250_000
  db = 0
OBJ

  rawAccMag   : "DCM_Tier1_Acc_Mag.spin"
  rawGyro     : "DCM_Tier1_gyro.spin"
  debug       : "fullDuplexSerial4port_tier2.spin"
  tr          : "TRIG.spin"
VAR
  long accMagStack[50], gyroStack[50], simStack[50]
  long mag[3], acc[3], gyro[3], magEarth[3], euler[3], heading, fastHeading, magIntrim[3]
  long base[5], dt[5]
  long magLowPassed[3], accLowPassed[3], gyroLowPassed[3]
  
PUB main

  debug.quickStartDebug

  cognew(runAccMag_Cog, @accMagStack)
  cognew(runGyro_Cog,   @gyroStack)
  cognew(simTier3, @simStack)
  
  repeat
    debug.clear(db)
    debug.str(db,String("gyroX "))
    debug.decLn(db,gyroLowPassed[0])
    debug.str(db,String("gyroY "))
    debug.decLn(db,gyroLowPassed[1])
    debug.str(db,String("gyroZ "))
    debug.decLn(db,gyroLowPassed[2])
    debug.newline(db)
    
    debug.str(db,String("accX "))
    debug.dec(db,accLowPassed[0])
    debug.str(db,String("   accY "))
    debug.dec(db,accLowPassed[1])
    debug.str(db,String("   accZ "))
    debug.decLn(db,accLowPassed[2])
    debug.str(db,String("eulerX "))
    debug.dec(db,euler[0])
    debug.str(db,String("   eulerY "))
    debug.dec(db,euler[1])
    debug.str(db,String("   eulerZ "))
    debug.decLn(db,euler[2])
     
    debug.str(db,String("norm acc: ")) 
    debug.decLn(db,^^(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]))
    debug.newline(db)
    debug.str(db,String("magX "))
    debug.dec(db,mag[0]*100/555)
    debug.str(db,String(" magY "))
    debug.dec(db,mag[1]*100/555)
    debug.str(db,String(" magZ "))
    debug.decLn(db,mag[2]*100/555)
    debug.str(db,String(" |mag| "))
    debug.decLn(db,^^(magLowPassed[0]*magLowPassed[0] +magLowPassed[1]*magLowPassed[1] +magLowPassed[2]*magLowPassed[2] ))
    debug.newline(db)
    debug.newline(db)
    debug.newline(db)
    debug.newline(db)
    debug.str(db,String("magIntrimX "))
    debug.dec(db,magLowPassed[0])
    debug.str(db,String(" magIntrimY "))
    debug.dec(db,magLowPassed[1])
    debug.str(db,String(" magIntrimZ "))
    debug.decLn(db,magLowPassed[2])
     debug.newline(db)
    debug.newline(db)
    debug.newline(db)
    debug.newline(db)
    debug.str(db,String("euler[0] "))
    debug.dec(db,euler[0])
    debug.str(db,String(" euler[1] "))
    debug.dec(db,euler[1])
     debug.newline(db)
    debug.newline(db)
    
    debug.str(db,String("heading: "))
    debug.dec(db, fastHeading) 
    debug.newline(db)
    debug.newline(db)
    debug.newline(db)

    debug.str(db,String("acc dt[0] = "))
    debug.decLn(db,dt[0])
    debug.str(db,String("acc freq[0] = "))
    debug.dec(db,clkfreq/dt[0])
    debug.strLn(db,String("Hz"))

    debug.newline(db)
    debug.str(db,String("mag dt[1] = "))
    debug.decLn(db,dt[1])
    debug.str(db,String("mag freq[1] = "))
    debug.dec(db,clkfreq/dt[1])
    debug.strLn(db,String("Hz"))

    debug.newline(db)
    debug.str(db,String("gyro dt[1] = "))
    debug.decLn(db,dt[2])
    debug.str(db,String("gyro freq[1] = "))
    debug.dec(db,clkfreq/dt[2])
    debug.strLn(db,String("Hz"))
    
    waitcnt(cnt + clkfreq/5)


PUB getGyro_t2(gyroPtr)|i
  getGyro
  repeat i from 0 to 2
    long[gyroPtr][i] := gyroLowPassed[i]

PUB getAcc_t2(accPtr)|i
  getAcc
  repeat i from 0 to 2
    long[accPtr][i] := accLowPassed[i]

PUB getMag_t2(magPtr)|i
  getMag
  repeat i from 0 to 2
    long[magPtr][i] := magLowPassed[i]

PRI simTier3 
  base[0] := cnt
  base[1] := cnt
  base[2] := cnt 
  repeat
    base[3] := cnt
    
    if (cnt - base[0]) > clkfreq/135  '~130 Hz 
      getAcc
      dt[0] := cnt - base[0] 
      base[0] := cnt
    
    if (cnt - base[1]) > clkfreq/11  '~10 Hz   
      getMag
      dt[1] := cnt - base[1]
      base[1] := cnt 
    
    if (cnt - base[2]) > clkfreq/185  '~175 Hz  
      getGyro
      dt[2] := cnt - base[2]
      base[2] := cnt
      
    dt[3] := cnt - base[3]

   


PRI getAcc  'need to be called at 130 Hz
  rawAccMag.getAcc(@acc)
  getAccLowPass
  
PRI getMag  'need to be called at 10 Hz
  rawAccMag.getMag(@mag)
  getMagLowPass
  
PUB runAccMag_Cog
  rawAccMag.runAccMag

PRI getGyro  'need to be called at 150 Hz
  rawGyro.getGyro(@gyro)
  getGyroLowPass
  
PUB runGyro_cog
  rawGyro.runGyro

PRI getMagLowPass | c, t, i, accNorm

 ' acc2Ang
  
  'mag[i]*100/max_axis  = scale mag values at -100 to 100 
  magIntrim[0] := (mag[0]*100/575) '- euler[0]*13/1000
  magIntrim[1] := (mag[1]*100/570) '+ euler[1]*13/1000    
  magIntrim[2] := mag[2]*100/560 

  t := 10
  c := 10
  repeat i from 0 to 2
    magLowPassed[i] := ( c*(magIntrim[i])/t + (t-c)*magLowPassed[i]/t)
  {
  if (magLowPassed[0] =< -30 )  ' North
    fastHeading :=  magLowPassed[1]*90/60
  else 'South
    if (magLowPassed[1] =< 0) 
      fastHeading :=  -180 - magLowPassed[1]*90/60       'SW             
    else
      fastHeading :=  180 - magLowPassed[1]*90/60        'SE
          }
PRI getAccLowPass | c, t, i
  t := 100
  c := 80

  repeat i from 0 to 2
    accLowPassed[i] := c*acc[i]/t + (t-c)*accLowPassed[i]/t
   
PRI getGyroLowPass | c, t, i
  t := 100
  c := 80

  repeat i from 0 to 2
    gyroLowPassed[i] := c*gyro[i]/t + (t-c)*gyroLowPassed[i]/t

PUB acc2ang | x, y, temp

  temp := acc[2] * acc[2] + acc[1] * acc[1]
  x := ^^(temp)
  y := acc[0]

  euler[0] := tr.atan2(x, y)  ' theta
  euler[1] := tr.atan2(-acc[2], -acc[1])  
  euler[2] := 0

 