'Data transfer between propellers
'#ifndef _clkmode
  _clkmode = c#_clkmode
  _xinfreq = c#_xinfreq
'#endif

''Theory of operation:
''This code is designed to transfer an array of sequential longs between two propeller chips. In this capacity, one chip acts as a master, and the other a slave device.
''Three wires are required for operation, and chip select (cs), clock (clk), and data line. An additional LED line is available to visually indicate the status of the
''transfer. Transfers are initiated by the master, which writes a specified number of longs to the receiver, and then clocks data from the slave. After the transfer, the
''master initiates a synchronization pulse to ensure the transfer windows are aligned. This synchronization window can realign in the case of an error, or resync in the case
''of a hard reset of either propeller chip.
''
''Sync pulse is as follows:
''The master holds the CS line high and sets the data line as an input. It then waits for the slave to strobe the data input line. After the strobe pulse is detected, the CS
''line is brought low by the master and the transmit routine is initiated. This is immediately followed by the receive routine, then looping back to the synchronization pulse.
''If the CS line is brought high by the master, the slave will exit its loop and resync based on a timeout counter. This effectively prevents the slave from locking up while
''waiting for a complete read/write from the master.  

'' NOTE: tx_count and rx_count are inverted depending on whether you are starting the master or slave. If the master transmits 20 longs, the slave must receive 20 longs... 

VAR

long    loop_cnt,error_count
byte    cog

OBJ

c :     "constants"

PUB TX_Start(tx_add,rx_add,tx_cnt,rx_cnt,CS,CLK,D_PIN,LED)
'Master system
{{
  Starts transmission from master to slave. 

  parameters:  tx_add - Address of data to transmit from main memory
               rx_add - Address to write received data from slave propeller.
               tx_count - Number of longs to transmit
               rx_count - Number of longs to receive
               CS - chip select pin
               CLK - clock pin
               D_PIN - data pin
               LED - LED pin                                                     
  return:      none

  example usage: TX_Start(@tx_buffer,@rx_buffer,20,30,1,2,3,4)
  expected results: Starts master cog and returns cog ID if successful
}}
mode := 0
return Start(tx_add,rx_add,tx_cnt,rx_cnt,CS,CLK,D_PIN,LED)

PUB RX_Start(tx_add,rx_add,tx_cnt,rx_cnt,CS,CLK,D_PIN,LED)
'Slave system
{{
  Starts receive from slave to master. 

  parameters:  tx_add - Address of data to transmit from main memory
               rx_add - Address to write received data from slave propeller.
               tx_count - Number of longs to transmit
               rx_count - Number of longs to receive
               CS - chip select pin
               CLK - clock pin
               D_PIN - data pin
               LED - LED pin                                                     
  return:      none

  example usage: RX_Start(@tx_buffer,@rx_buffer,30,20,1,2,3,4)
  expected results: Starts slave cog and returns cog ID if successful
}}
mode := 1
return Start(tx_add,rx_add,tx_cnt,rx_cnt,CS,CLK,D_PIN,LED)

PUB Start(AD,AD1,tx_cnt,rx_cnt,CS,CLK,D_PIN,LED)

if cog 
  return
  
cs_pin   := |<CS
clk_pin  := |<CLK
data_pin := |<D_PIN

if led <> -1
  led_pin  := |<LED
else
  led_pin := LED  

tx_count := tx_cnt           
rx_count := rx_cnt
tx_address := AD
rx_address := AD1

return cog := cognew(@DATA_RX_START,@loop_cnt)

PUB Get_Errors
'Returns errors detected by slave

return error_count

PUB Get_Loop_Count
'Returns number of cycles

return loop_cnt  
  
PUB Stop

if cog
  cogstop(cog~)

DAT  'Slave Receive

org 0

'***********************************************************************************************
DATA_RX_START
                        tjz     mode,#DATA_TRANSFER_START                       'If transmit prop, jump to tx routines
                        
                        andn    dira,cs_pin             'Set cs,clk,data pins as outputs
                        andn    dira,clk_pin
                        andn    dira,data_pin
             '           cmp     led_pin,#0      wc
             ' if_nc     or      dira,led_pin
                        call    #DATA_RX_LOOP
                        
DATA_RX_START_RET       ret

'***********************************************************************************************
DATA_ERROR
                        cmp     timeout,#0      wz      'Log errors on receiver side
              if_z      add     error,#1
                        mov     p1,par
                        add     p1,#4                   'Offset to second long           
                        wrlong  error,p1

DATA_ERROR_RET          ret

'***********************************************************************************************
DATA_RX_LOOP
             '           cmp     led_pin,#0      wc
            '  if_nc     xor     outa,LED_PIN            'Strobe LED to indicate sync
                        mov     p1,par                  'Loop counter for update flag
                        add     loop_count,#1
                        wrlong  loop_count,p1

                        waitpeq cs_pin,cs_pin           'Wait for cs_pin to go high
                        or      dira,data_pin           'Make data pin output
                        or      outa,data_pin           'Strobe data pin
                        waitpne cs_pin,cs_pin           'Wait for CS to go low, indicates start condition
                        andn    dira,data_pin           'Make data pin input

                        'Setup for receive from master
                        mov     p1,rx_address           'Set by AD1 variable
                        mov     buff_size,rx_count  wz  'Set by rx_cnt input variable
                        andn    dira,data_pin           'Set data pin as input                        
         :rx_loop                         
              if_nz     call    #READ_32_R                                          
              if_nz     wrlong  data_value,p1
              if_nz     add     p1,#4
              if_nz     djnz    buff_size,#:RX_LOOP                        

                        'Setup for transfer to master
                        mov     p1,tx_address
                        mov     buff_size,tx_count  wz                        
                        or      dira,data_pin
         :tx_loop
              if_nz     rdlong  data_value,p1
              if_nz     add     p1,#4                     
              if_nz     call    #WRITE_32_R
              if_nz     djnz    buff_size,#:TX_LOOP

                        call    #DATA_RX_LOOP                                  

DATA_RX_LOOP_RET        ret

'***********************************************************************************************
WRITE_32_R
                        mov     counter,#32
                        mov     timeout,timeout_cnt  
                        ror     data_value,#1 wc                        
        :write_loop
                        test    clk_pin,ina     wz
               if_z     djnz    timeout,#:WRITE_LOOP
                        tjz     timeout,#DATA_ERROR
                        muxc    outa,data_pin           'Write data bit
                        ror     data_value,#1 wc        'Rotate data value for next loop  
                        waitpne clk_pin,clk_pin         'Wait for clock to go low               
                        djnz    counter,#:WRITE_LOOP   'if not 32 bits, keep going
                        
WRITE_32_R_RET          ret

'***********************************************************************************************
READ_32_R
                        mov     counter,#32
                        mov     timeout,timeout_cnt
                        
        :read_loop
                        test    clk_pin,ina     wz
               if_z     djnz    timeout,#:READ_LOOP
                        tjz     timeout,#DATA_ERROR
                        test    data_pin,ina wc
                        rcr     data_value,#1
                        waitpne clk_pin,clk_pin                        
                        djnz    counter,#:READ_LOOP   'if not 32 bits, keep going
                        
READ_32_R_RET           ret

DAT  'Master Transmit 

'***********************************************************************************************
DATA_TRANSFER_START
                        mov     p1,tx_address
                          
                        or      dira,cs_pin             'Set cs,clk,led, and data pins as outputs
                        or      dira,clk_pin
                        or      dira,data_pin
             '           cmp     led_pin,#0      wc      'If led pin active (not -1), set as output
             ' if_nc     or      dira,led_pin

                        or      outa,cs_pin             'Drive cs pin high to indicate start condition                        
                        andn    outa,clk_pin            'Drive clk and data low
                        andn    outa,data_pin
                        
                        call    #DATA_TX_LOOP

                        
DATA_TRANSFER_START_RET ret

'***********************************************************************************************
DATA_TX_LOOP
                        'cmp     led_pin,#0      wc
              'if_nc     xor     outa,LED_PIN            'Strobe LED to indicate sync
                        mov     p1,par                  'Loop counter for update flag
                        add     loop_count,#1
                        wrlong  loop_count,p1
              

                        or      outa,cs_pin             'Set CS pin high for sync
                        andn    dira,data_pin           'Make data pin input for strobe pulse
                        waitpeq data_pin,data_pin       'Wait for receive propeller to strobe data pin
                        andn    outa,cs_pin             'drive cs pin low
                        or      dira,data_pin           'Make data pin an output for write operation

                        'Setup for trasmit to slave
                        mov     p1,tx_address           'Transmit buffer location in main memory
                        mov     buff_size,tx_count  wz  'Set by TX_cnt input variable
                                                 
        :tx_loop        
              if_nz     rdlong  data_value,p1           'Get value to write
              if_nz     add     p1,#4                   'Increment to next long
              if_nz     call    #WRITE_32_T                             
              if_nz     djnz    buff_size,#:TX_LOOP     

                        'Setup for receive from slave
                        mov     p1,rx_address           'Set pointer to receive address
                        mov     buff_size,rx_count  wz  'Set counter for receive size, set by rx_cnt variable
                        andn    dira,data_pin           'Make data pin input for read operation                        
       :rx_loop
                        long 0[12]                      'Read overhead for synchronization
              if_nz     call    #READ_32_T
              if_nz     wrlong  data_value,p1
              if_nz     add     p1,#4
              if_nz     djnz    buff_size,#:RX_LOOP

                        call   #DATA_TX_LOOP            'Main repeat loop
                        
DATA_TX_LOOP_RET        ret

'***********************************************************************************************
WRITE_32_T
                        mov     counter,#32
                        ror     data_value,#1 wc                                                                  
        :tx_loop
                        or      outa,clk_pin            'Set clock high                        
                        muxc    outa,data_pin           'Write c value to data pin
                        ror     data_value,#1 wc        'Increment data bit
                        nop                             'mismatched clocks on propeller chips...
                        nop
                        andn    outa,clk_pin            'Drive clock pin low
                        nop
                        nop                                                       
                        djnz    counter,#:tx_loop
                        
WRITE_32_T_RET          ret

'***********************************************************************************************
READ_32_T
                        mov     counter,#32
                        andn    dira,data_pin                     
        :read_loop
                        or      outa,clk_pin            'Set clock pin high                        
                        long 0[7]                       'Waitpeq overhead before read       
                        test    data_pin,ina wc         'Read data pin
                        andn    outa,clk_pin            'Set clock pin low for waitpne overhead                                  
                        rcr     data_value,#1 wc        'Accumulate c flag into data
                        nop
                        djnz    counter,#:READ_LOOP
                          
READ_32_T_RET           ret

timeout_cnt   long      50 
error         long      0       'Error flag for incorrect syncronization or timeout
loop_count    long      0
mode          byte      0       'Master or slave declaration

cs_pin        long      0       'Chip select pin
clk_pin       long      0       'Clock pin, controlled by master
data_pin      long      0       'Data transfer pin
led_pin       long      0       'Optional LED pin

tx_count      long      0       'Number of longs to transmit
rx_count      long      0       'Number of longs to receive 
tx_address    long      0       'Location of transmit buffer in main memory
rx_address    long      0       'Location of receive buffer in main memory 

timeout       res       1       'Timeout counter
buff_size     res       1       'Transmit and receive buffer size counter
data_value    res       1       'Scratch pad long value
p1            res       1       'Address pointer
counter       res       1       'Bit counter for transmit and receive operations

fit 496              

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