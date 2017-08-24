CON


  FF                            = 12                    ' form feed
  CR                            = 16                    ' carriage return
  NL                            = 13


  debug =  0
  debugRx = 31
  debugTx = 30
  debugBaud = 115200
  
OBJ
  com :  "fullDuplexSerial4port_tier1.spin"

PUB quickStartDebug

   com.AddPort(debug, debugRx, debugTx,-1,-1,0,0, debugBaud)   

   result := com.start

PUB clear(port)

  char(port, CR)

PUB rxFlush(port)

  com.rxFlush(port)

PUB rxIsIn(port) : rxbyte

  rxbyte := com.rxIsIn(port)

PUB RxCount(port) : count

  count := com.RxCount(port)

PUB RxHowFull(port)

  return com.RxHowFull(port)  
PUB strLn(port,stringptr)
'' Send string                    
 str(port, stringPtr)
 newline(port)
                           
PUB str(port,stringptr)
'' Send string                    
  repeat strsize(stringptr)
    com.tx(port,byte[stringptr++])

PUB newline(port)
  char(port,NL)

PUB char(port,txbyte)
'' Send a byte to the terminal
  com.tx(port,txbyte)

PUB charIn(port)
'' Get a character
'' -- will not block if nothing in uart buffer
   return com.rxcheck(port)
'  return rx

pub decLn(port,value)

  dec(port, value)
  newline(port)

PUB dec(port,value) | i
'' Print a decimal number
  decl(port,value,10,0)

PUB decl(port,value,digits,flag) | i, x
  digits := 1 #> digits <# 10
  x := value == NEGX                                                            'Check for max negative
  if value < 0
    value := ||(value+x)                                                        'If negative, make positive; adjust for max negative
    com.tx(port,"-")

  i := 1_000_000_000
  if flag & 3
    if digits < 10                                      ' less than 10 digits?
      repeat (10 - digits)                              '   yes, adjust divisor
        i /= 10
  repeat digits
    if value => i
      com.tx(port,value / i + "0")
      value //= i
      result~~
    elseif (i == 1) OR result OR (flag & 2)
      com.tx(port,"0")
    elseif flag & 1
      com.tx(port," ")
    i /= 10