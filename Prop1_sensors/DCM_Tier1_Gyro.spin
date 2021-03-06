{{
  Modified by Elliot Lee 5.31.2016
  ┌──────────────────────────────────────────┐
  │ MPU-9150 demo using my I2C driver        │
  │ Author: Chris Gadd                       │
  │ Copyright (c) 2014 Chris Gadd            │
  │ See end of file for terms of use.        │
  └──────────────────────────────────────────┘
}}

CON
  _clkmode = xtal1 + pll16x                                                    
  _xinfreq = 6_250_000

  scl = 0'15
  sda = 1'14
  mpuAdd = $68
  db = 0
  
VAR
  '1st-tier data
  long gyro[3], bias[3]

  'intermediate data
  long stack[128]

  'raw data
  byte gyroArr[6]
  long prev, dt

OBJ
  I2C    : "I2C Spin driver v1.3"
  debug  : "fullDuplexSerial4Port_tier2.spin"

PUB Main 

  debug.quickStartDebug
  cognew(runGyro, @stack)

  repeat
    debug.clear(db)
    debug.str(db,String("dt = "))
    debug.decLn(db,dt)
    debug.str(db,String("freq = "))
    debug.dec(db,clkfreq/dt)
    debug.strLn(db,String("Hz"))

    debug.str(db,String("gyroX "))
    debug.decLn(db,gyro[0])
    debug.str(db,String("gyroY "))
    debug.decLn(db,gyro[1])
    debug.str(db,String("gyroZ "))
    debug.decLn(db,gyro[2])
    debug.newline(db)

    waitcnt(cnt + clkfreq/10)

PUB getGyro(gyroPtr) 'need to be called at 150 Hz

  long[gyroPtr][0] := gyro[1]
  long[gyroPtr][1] := gyro[0]
  long[gyroPtr][2] := -gyro[2]
     
PUB runGyro 
  initSensor(SCL, SDA)
  calcBias
    
  repeat
    prev := cnt
    updateGyro
    dt := cnt - prev


PRI calcBias | coef

  I2C.write(mpuAdd,$6B,$00)                                                        ' take out of sleep and use gyro-x as clock source
  I2C.write(mpuAdd,$19,$01)                                                        ' Sample rate divider (divide gyro_rate by 1 + x)
  I2C.write(mpuAdd,$1A,%00_000_110)                                                ' Digital low-pass filtering (0 = 8KHz gyro_rate, !0 = 1KHz gyro_rate)
  I2C.write(mpuAdd,$1B,%000_00_000)                                                ' Accelerometer sensitivity ±250°/s
  I2C.write(mpuAdd,$1C,%000_00_000)        

  coef := 200
  repeat coef
    repeat until I2C.read(mpuAdd,$3A) & $01
    I2C.read_page(mpuAdd,$43,@gyroArr,6)
    bias[0] += ~gyroArr[0] << 8 | gyroArr[1]
    bias[1] += ~gyroArr[2] << 8 | gyroArr[3]
    bias[2] += ~gyroArr[4] << 8 | gyroArr[5]

  bias[0] /= coef
  bias[1] /= coef
  bias[2] /= coef  

PRI initSensor(sc, sd)
  I2C.start(sc,sd)      

PRI updateGyro

  if I2C.read(mpuAdd,$3A) & $01 
    I2C.read_page(mpuAdd,$43,@gyroArr,6)  
    gyro[0] := ~gyroArr[0] << 8 | gyroArr[1] - bias[0] 
    gyro[1] := ~gyroArr[2] << 8 | gyroArr[3] - bias[1]     
    gyro[2] := ~gyroArr[4] << 8 | gyroArr[5] - bias[2]


{{
┌──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┐
│                                                   TERMS OF USE: MIT License                                                  │                                                            
├──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
│Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation    │ 
│files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,    │
│modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software│
│is furnished to do so, subject to the following conditions:                                                                   │
│                                                                                                                              │
│The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.│
│                                                                                                                              │
│THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE          │
│WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR         │
│COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,   │
│ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                         │
└──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
}}                                