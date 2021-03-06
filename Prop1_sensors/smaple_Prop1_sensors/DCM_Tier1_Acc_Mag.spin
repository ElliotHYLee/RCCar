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

  sclAcc = 2
  sdaAcc = 3

  sclMag = 4
  sdaMag = 5

  accXOffset = 150'1230
  accYOffset = -275
  accZOffset = 0'-800
  
  mpuAdd = $68

  db = 0

  MODE           = $02
  READ_DATA      = $3D 'Used to perform a Read operation
  WRITE_DATA     = $3C
  OUTPUT_X_MSB   = $03

VAR
 
  '1st-tier data
  long acc[3], mag[3],rawMag[3], counter

  long stack[128]

  'raw data
  byte accArr[6], magArr[6]
  long prev1, dt1, prev2, dt2
  byte  asax, asay, asaz

    
OBJ
  I2C : "I2C Spin driver v1.3"
  debug : "fullDuplexSerial4Port_tier2.spin"

PUB Main 

  debug.quickStartDebug
  cognew(runAccMag, @stack)

  repeat
    debug.clear(db)
    debug.str(db,String("dt1 = "))
    debug.decLn(db,dt1)
    debug.str(db,String("freq1 = "))
    debug.dec(db,clkfreq/dt1)
    debug.strLn(db,String("Hz"))
    
    debug.str(db,String("accX "))
    debug.dec(db,acc[0])
    debug.str(db,String("   accY "))
    debug.dec(db,acc[1])
    debug.str(db,String("   accZ "))
    debug.decLn(db,acc[2])

    debug.str(db,String("norm acc: ")) 
    debug.decLn(db,^^(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]))
    debug.newline(db)


    debug.str(db,String("dt2 = "))
    debug.decLn(db,dt2)
    debug.str(db,String("freq2 = "))
    debug.dec(db,clkfreq/dt2)
    debug.strLn(db,String("Hz"))

    
    debug.str(db,String("magX "))
    debug.dec(db,mag[0])
    debug.str(db,String(" magY "))
    debug.dec(db,mag[1])
    debug.str(db,String(" magZ "))
    debug.decLn(db,mag[2])
    debug.newline(db)

    waitcnt(cnt + clkfreq/10)

PUB getAcc(accPtr)

  long[accPtr][0] := acc[1]
  long[accPtr][1] := acc[0]
  long[accPtr][2] := -acc[2]
  
PUB getMag(magPtr) | i
  long[magPtr][0] := mag[1]
  long[magPtr][1] := mag[0]
  long[magPtr][2] := mag[2]
    
PUB runAccMag | il
  initSensor(sclAcc, sdaAcc)  
  setUpMpu(%000_00_00, %000_00_000)
  counter := 0
  repeat
    prev1 := cnt
    updateAcc
    il++   
    waitcnt(cnt + (clkfreq/130 - (cnt-prev1)))  ' 130Hz for Accelerometer
    dt1 := cnt - prev1
    if (il => 12) ' 10 Hz for Magnetometer
      dt2 := cnt - prev2
      prev2 := cnt
      updateMag
      il := 0

PRI initSensor(sc, sd)
  I2C.start(sc,sd)      

PRI setUpMPU(gyroSense, accSense)
  I2C.write(mpuAdd,$6B,$00)                                                        ' take out of sleep and use gyro-x as clock source
  I2C.write(mpuAdd,$37,$02)                                                        ' enable I2C bypass in order to communicate with the magnetometer at address $0C
  
  I2C.write(mpuAdd,$19,$01)                                                        ' Sample rate divider (divide gyro_rate by 1 + x)
  I2C.write(mpuAdd,$1A,%00_000_110)                                                ' Digital low-pass filtering (0 = 8KHz gyro_rate, !0 = 1KHz gyro_rate)
  I2C.write(mpuAdd,$1B,gyroSense)                                                ' Accelerometer sensitivity ±250°/s
  I2C.write(mpuAdd,$1C,accSense)                                                ' Accelerometer sensitivity ±2g



PRI updateAcc

  if I2C.read(mpuAdd,$3A) & $01 
    I2C.read_page(mpuAdd,$3B,@accArr,6)
    I2C.read(mpuAdd,$3B)  
    acc[0] := (~accArr[0] << 8 | accArr[1] )   - accXOffset 
    acc[1] := (~accArr[2] << 8 | accArr[3] )   - accYOffset    
    acc[2] := (~accArr[4] << 8 | accArr[5] )   - accZOffset 
  
PRI updateMag

  setCont
  setPointer(OUTPUT_X_MSB)
  getRaw


PRI SetCont

{{ Sets the Compass to Continuous output mode.}}
 
  start
  send(WRITE_DATA)
  send(MODE)
  send($00)
  stop

PRI SetPointer(Register)

{{ Start pointer at user specified Register. }}

  start
  send(WRITE_DATA)
  send(Register)
  stop

PRI GetRaw 

{{ Get raw data from continous output.}}

  start
  send(READ_DATA)
  rawMag[0] := ((receive(true) << 8) | receive(true))            'RegisterA and RegisterB
  rawMag[2] := ((receive(true) << 8) | receive(true))
  rawMag[1] := ((receive(true) << 8) | receive(false))
  stop
  ~~rawMag[0]
  ~~rawMag[1]
  ~~rawMag[2]
  mag[0] := rawMag[0] - (-22)'(5)      
  mag[1] := rawMag[1] - (25) '(-179)    
  mag[2] := rawMag[2] - (-67)'(23)         

   
PRI send(value) ' I²C Send data - 4 Stack Longs

  value := ((!value) >< 8)

  repeat 8
    dira[sdaMag]  := value
    dira[sclMag] := false
    dira[sclMag] := true
    value >>= 1

  dira[sdaMag]  := false
  dira[sclMag] := false
  result         := !(ina[sdaMag])
  dira[sclMag] := true
  dira[sdaMag]  := true

PRI receive(aknowledge) ' I²C receive data - 4 Stack Longs

  dira[sdaMag] := false

  repeat 8
    result <<= 1
    dira[sclMag] := false
    result         |= ina[sdaMag]
    dira[sclMag] := true

  dira[sdaMag]  := aknowledge
  dira[sclMag] := false
  dira[sclMag] := true
  dira[sdaMag]  := true

PRI start ' 3 Stack Longs

  outa[sdaMag]  := false
  outa[sclMag] := false
  dira[sdaMag]  := true
  dira[sclMag] := true

PRI stop ' 3 Stack Longs

  dira[sclMag] := false
  dira[sdaMag]  := false
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