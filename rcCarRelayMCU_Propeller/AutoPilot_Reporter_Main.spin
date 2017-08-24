CON             
  _clkmode = xtal1 + pll16x     
  _xinfreq = 5_000_000

  db = 0
  xb = 1
  motorPin = 0
  servoPin = 1
  
VAR
  'debug
  long debuggerStack[100], readPulseStack[100], pwmStack[100], pwmServoStack[100]

  long high, period, pwmOut
  long high_servo, period_servo, pwmOut_servo

OBJ
  debug         : "fullDuplexSerial4port_tier2.spin"

  
PUB Main | i


  period_servo := 9000
  period := 10_000
  high := 1490
  
  'cognew(runDebug, @debuggerStack)
  cognew(readPulse, @readPulseStack)
  cognew(writePWM, @pwmStack)
  cognew(writeServoPWM, @pwmServoStack)
  



PUB writeServoPWM

  dira[servoPin] := 1 
  repeat
    pwmOut_servo := high_servo
    if pwmOut_servo > 900
      outa[servoPin]:=1
      waitcnt(cnt + clkfreq/1_000_000*pwmOut_servo)
      outa[servoPin]:=0
      waitcnt(cnt + clkfreq/1_000_000*(period - high_servo))
    else
      outa[servoPin]:=1
      waitcnt(cnt + clkfreq/1_000_000*1500)
      outa[servoPin]:=0
      waitcnt(cnt + clkfreq/1_000_000*(period - 1500))   
  




PUB writePWM

  dira[motorPin] := 1 
  repeat
    pwmOut := high
    if pwmOut > 900
      outa[motorPin]:=1
      waitcnt(cnt + clkfreq/1_000_000*pwmOut)
      outa[motorPin]:=0
      waitcnt(cnt + clkfreq/1_000_000*(period - high))
    else
      outa[motorPin]:=1
      waitcnt(cnt + clkfreq/1_000_000*1500)
      outa[motorPin]:=0
      waitcnt(cnt + clkfreq/1_000_000*(period - 1500))   
  
PUB runDebug | i

  debug.quickStartDebug 
  repeat
    debug.clear(db)
    'debug.newline(db)

    debug.str(db, String("high: "))
    debug.decln(db, high_servo)
    debug.str(db, String("period: "))
    debug.decln(db, period)
    debug.str(db, String("out: "))
    debug.decln(db, pwmOut_servo)    
    waitcnt(cnt + clkfreq/3)

PUB readPulse

  repeat
    pulse_in(3)
    'lowTime(2)
    pulse_in_servo(2)
    
pri pulse_in(pinNumber) | mask

  mask := 1 << pinNumber                                        ' mask for pin
  frqa := 1                                                     ' set for high-resolution measurement
  ctra := (%01000 << 26) | pinNumber                            ' set ctra to POS detect on pin   
  waitpne(mask, mask, 0)                                        ' wait for pin to be low
  phsa := 0                                                     ' clear accumulator
  waitpeq(mask, mask, 0)                                        ' wait for pin to go high
  waitpne(mask, mask, 0)                                        ' wait for pin to return low
  high := phsa / (clkfreq / 1_000_000)                         ' convert ticks to micro sec
           
pri lowTime(pinNumber) | t1, t2

  waitpeq(0, |< pinNumber, 0)         ' wait for sigin pin to be low
  waitpeq(|< pinNumber, |< pinNumber, 0)  ' wait for sigin pin to be high
  t1 := cnt


  waitpeq(0, |< pinNumber, 0)         ' wait for sigin pin to be low
  waitpeq(|< pinNumber, |< pinNumber, 0)  ' wait for sigin pin to be high
  t2 := cnt                       ' capture system clock at rising edge
  period := (t2 - t1)*1000/clkfreq*1000


pri pulse_in_servo(pinNumber) | mask

  mask := 1 << pinNumber                                        ' mask for pin
  frqa := 1                                                     ' set for high-resolution measurement
  ctra := (%01000 << 26) | pinNumber                            ' set ctra to POS detect on pin   
  waitpne(mask, mask, 0)                                        ' wait for pin to be low
  phsa := 0                                                     ' clear accumulator
  waitpeq(mask, mask, 0)                                        ' wait for pin to go high
  waitpne(mask, mask, 0)                                        ' wait for pin to return low
  high_servo := phsa / (clkfreq / 1_000_000)                         ' convert ticks to micro sec
  









  