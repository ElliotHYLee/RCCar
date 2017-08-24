'===============================================================================
'MyMath.Spin :Matrix representation follows Matlab's matrix repesentation
'Made by : Elliot Lee
'Date    : Sep/5/2015
'===============================================================================

CON
  _clkmode = xtal1 + pll16x                                                    
  _xinfreq = 6_250_000
  
CMNSACLE = 10_000
ObJ

 ' fds : "FullDuplexSerial"
  tr     : "TRIG.spin"


VAR

  long R[9], testAcc[3], temp3x3[9]

{
PUB main

  fds.quickStart

  testAcc[0] := -1
  testAcc[1] := 2
  testAcc[2] := -3
  
  repeat
    
    fds.clear
   ' fds.decLn(getFirstDCM(@R, @testAcc))
    waitcnt(cnt + clkfreq/10)
}
PUB sqrt(value)| x, i

  x := value

  repeat i from 0 to 20
    x := (value/x + x) /2

  return x

PUB getIdentityMatrix(EPtr) { 10^4 = unity to represent 1.xxxx}

  long[EPtr][0] := 10000
  long[EPtr][1] := 0
  long[EPtr][2] := 0

  long[EPtr][3] := 0
  long[EPtr][4] := 10000
  long[EPtr][5] := 0

  long[EPtr][6] := 0
  long[EPtr][7] := 0
  long[EPtr][8] := 10000  
  
PUB getSign(value)

  if value >= 0
    result := 1
  else
    result := -1

PUB getAbs(value)
  if value > 0
    result := value
  else
    result := -value
'==============================================================================================================  
'==============================================================================================================
'==============================================================================================================
'==============================================================================================================
'==============================================================================================================  
'==============================================================================================================
'==============================================================================================================
'==============================================================================================================
'==============================================================================================================  
'==============================================================================================================
'==============================================================================================================
'==============================================================================================================
' 2 by 2 matrix (vector) operations & special operations
PUB detOp22(matPtr)

  return long[matPtr][0]*long[matPtr][3] - long[matPtr][1]*long[matPtr][2]

{
{==================================================================
colOf33: choose a column of 3 by 3 matrix

    @matPtr33 : matrix A
    @col : column number. starts from left and 1
    
    @updates : desVec
==================================================================}
PUB colOf33(matPtr33, col, desVec)

  long[desVec][0] := long[matPtr33][col-1 + 0]
  long[desVec][1] := long[matPtr33][col-1 + 3]
  long[desVec][2] := long[matPtr33][col-1 + 6]

PUB glueCol31(col1, col2, col3, desMat)

  long[desMat][0] := long[col1][0]
  long[desMat][1] := long[col2][0]
  long[desMat][2] := long[col3][0]
  
  long[desMat][3] := long[col1][1]
  long[desMat][4] := long[col2][1]
  long[desMat][5] := long[col3][1]

  long[desMat][6] := long[col1][2]
  long[desMat][7] := long[col2][2]
  long[desMat][8] := long[col3][2]

}
'==============================================================================================================  
'==============================================================================================================
'==============================================================================================================
'==============================================================================================================
'==============================================================================================================  
'==============================================================================================================
'==============================================================================================================
'==============================================================================================================
'==============================================================================================================  
'==============================================================================================================
'==============================================================================================================
'==============================================================================================================
' 3 by 1 matrix (vector) operations




{==================================================================
addOp31: 3by1 matrix addition

    @matPtr1 : matrix A
    @matPtr2 : matrix B
    
    @updates : matPtr3
==================================================================}
pub addOp31(matPtr1, matPtr2, matPtr3) | i
  repeat i from 0 to 2
    long[matPtr3][i] := long[matPtr1][i] + long[matPtr2][i]

{==================================================================
subOp31: 3by1 matrix subtraction

    @matPtr1 : matrix A
    @matPtr2 : matrix B
    
    @updates : matPtr3
==================================================================}
pub subOp31(matPtr1, matPtr2, matPtr3) | i
  repeat i from 0 to 2
    long[matPtr3][i] := long[matPtr1][i] - long[matPtr2][i]
{==================================================================
scalarMult31: 3by1 matrix multiplication of scalar

    @matAPtr : matrix A
    @k : scalar value to muliply

    @updates : matAPtr
==================================================================}
pub scalarMult31(matPtr, k) | i
  repeat i from 0 to 2
    long[matPtr][i] *= k 

{==================================================================
dot31: 3by1 matrix dot product

    @matPtr1 : matrix A
    @matPtr2 : matrix B

    @returns : answer = dot(A,B)
==================================================================}
pub dot31(matPtr1, matPtr2) | i , answer
  answer := 0
  repeat i from 0 to 2
    answer += long[matPtr1][i]*long[matPtr2][i]

  return answer

{==================================================================
cross31: 3by1 matrix cross product

    @matPtr1 : matrix A
    @matPtr2 : matrix B

    @updates : matPtr3
==================================================================}
pub cross31(matPtr1, matPtr2, matPtr3) 


  long[matPtr3][0] := long[matPtr1][1]*long[matPtr2][2] - long[matPtr1][2]*long[matPtr2][1]
  long[matPtr3][1] := long[matPtr1][2]*long[matPtr2][0] - long[matPtr1][0]*long[matPtr2][2]
  long[matPtr3][2] := long[matPtr1][0]*long[matPtr2][1] - long[matPtr1][1]*long[matPtr2][0]   
                                                                                          
  
'==============================================================================================================  
'==============================================================================================================
'==============================================================================================================
'==============================================================================================================
'==============================================================================================================  
'==============================================================================================================
'==============================================================================================================
'==============================================================================================================
'==============================================================================================================  
'==============================================================================================================
'==============================================================================================================
'==============================================================================================================
'      3 by 3 matrix operations
{==================================================================
scalarMultOp33: 3by3 matrix multiplication of scalar

    @matAPtr : matrix A
    @k : scalar value to muliply

    @updates : matAPtr
==================================================================}
PUB scalarMultOp33(matPtr, k) | i

  repeat i from 0 to 9
    long[matPtr][i] *= k
    
{==================================================================
scalarDivOp33: 3by3 matrix division of scalar

    @matAPtr : matrix A
    @k : scalar value to muliply

    @updates : matAPtr
==================================================================}
PUB scalarDivOp33(matPtr, k) | i, sign
  sign := getSign(long[matPtr][i])*getSign(k)
  repeat i from 0 to 9
    sign := getSign(long[matPtr][i])*getSign(k)  
    long[matPtr][i] := (long[matPtr][i] /k)'+ sign*k/2) / k <-faster

{==================================================================
multOp33: 3by3 matrix multiplication

    @matAPtr : matrix A
    @matBPtr : matrix B
    @matCPtr : result matrix C
               C = A * B

    @updates : matCPtr
==================================================================}        
PUB multOp33(matAPtr, matBPtr, matCPtr)| i,j, rowCheck, colCheck

  repeat i from 0 to 8
    long[matCPtr][i] := 0

  rowCheck := 0
  colCheck := 0

  i:= 0
  j:= 0
  
  repeat 3
    repeat 3  
      repeat 3
        long[matCPtr][i] += long[matAPtr][rowCheck+j]*long[matBPtr][colCheck + 3*j] 
        j++
      i++
      j:=0        
      colCheck++  
    rowCheck+=3 
    colCheck :=0

{==================================================================
addOp33: 3by3 matrix addition

    @matAPtr : matrix A
    @matBPtr : matrix B
    @matCPtr : result matrix C
               C = A + B

    @updates : matCPtr
==================================================================} 
PUB addOp33(matAPtr, matBPtr, matCPtr) | i

  i := 0
  repeat i from 0 to 8
    long[matCPtr][i] := long[matAPtr][i] + long[matBPtr][i]


{==================================================================
subOp33: 3by3 matrix subtraction

    @matAPtr : matrix A
    @matBPtr : matrix B
    @matCPtr : result matrix C
               C = A - B

    @updates : matCPtr
==================================================================}             
PUB subOp33(matAPtr, matBPtr, matCPtr) | i

  i := 0
  repeat i from 0 to 8
    long[matCPtr][i] := long[matAPtr][i] - long[matBPtr][i]


{==================================================================
transposeOp: calculates transpose of 3by3 matrix

   @matResultPtr: destination matrix
   
   @updates: matResultPtr
==================================================================}
PUB transposeOp(matAPtr, matResultPtr) | i, j,k

  i :=0
  j :=0
  k :=0

  repeat i from 0 to 8
    long[matResultPtr][i] := long[matAPtr][3*k + j]
    k++
    if ((i+1)//3 ==0)
      j++
      k :=0

{==================================================================
detOp: calculates determinant of 3by3 matrix

   @matAPtr : array, size of 9 -> represents 3by3 matrix

   @returns: determinant of matAPtr
==================================================================}
PUB detOp33(matAPtr) : det

  det := long[matAPtr][0]*(long[matAPtr][4]*long[matAPtr][8]-long[matAPtr][7]*long[matAPtr][5])-long[matAPtr][1]*(long[matAPtr][3]*long[matAPtr][8]-long[matAPtr][6]*long[matAPtr][5]) + long[matAPtr][2]*(long[matAPtr][3]*long[matAPtr][7]-long[matAPtr][6]*long[matAPtr][4])

{==================================================================
invOp: calculates inverse of 3by3 matrix

   @matAPtr : array, size of 9 -> represents 3by3 matrix
   @matResultPtr: destination matrix
   
   @updates: matResultPtr
   @returns: -1 only when no determinant exists   
==================================================================}
PUB invOp(matAPtr, matResultPtr)| det, i

  det := detOp33(matAPtr)
  if (det ==0)
    return -1

  long[matResultPtr][0] := long[matAPtr][4]*long[matAPtr][8]-long[matAPtr][5]*long[matAPtr][7]
  long[matResultPtr][1] := long[matAPtr][2]*long[matAPtr][7]-long[matAPtr][1]*long[matAPtr][8] 
  long[matResultPtr][2] := long[matAPtr][1]*long[matAPtr][5]-long[matAPtr][2]*long[matAPtr][4] 
  
  long[matResultPtr][3] := long[matAPtr][5]*long[matAPtr][6]-long[matAPtr][3]*long[matAPtr][8]
  long[matResultPtr][4] := long[matAPtr][0]*long[matAPtr][8]-long[matAPtr][2]*long[matAPtr][6]
  long[matResultPtr][5] := long[matAPtr][2]*long[matAPtr][3]-long[matAPtr][0]*long[matAPtr][5]
  
  long[matResultPtr][6] := long[matAPtr][3]*long[matAPtr][7]-long[matAPtr][4]*long[matAPtr][6]
  long[matResultPtr][7] := long[matAPtr][1]*long[matAPtr][6]-long[matAPtr][0]*long[matAPtr][7]
  long[matResultPtr][8] := long[matAPtr][0]*long[matAPtr][4]-long[matAPtr][1]*long[matAPtr][3]

  repeat i from 0 to 8 'rounding up for final result
   ' if long[matResultPtr][i] > 0
      long[matResultPtr][i] := (long[matResultPtr][i] /det)'+ getAbs(det)/2) / det
    'else
     ' long[matResultPtr][i] := (long[matResultPtr][i] - getAbs(det)/2) /det

    
'==============================================================================================================  
'==============================================================================================================
'==============================================================================================================
'==============================================================================================================
'==============================================================================================================  
'==============================================================================================================
'==============================================================================================================
'==============================================================================================================
'==============================================================================================================  
'==============================================================================================================
'==============================================================================================================
'==============================================================================================================
' DCM supporter function codes start from here
  
PUB skew(Dptr, a, b, c)

  long[Dptr][1] := -c
  long[Dptr][2] := b
  long[Dptr][3] := c
  long[Dptr][5] := -a
  long[Dptr][6] := -b  
  long[Dptr][7] := a

{ ================================================= 
  a2d : getting DCM from euler angles
  @Rptr in value * 10_000
  @eulerPtr in degree*100 , theta, phi, psi <- always this order

  @updates Rptr in 10_000
=================================================== }
PUB a2d(RPtr, eulerPtr)| th, ph, ps, temp, reg1, reg2

  th := long[eulerPtr][0]
  ph := long[eulerPtr][1]
  ps := long[eulerPtr][2]


  reg1 := conv(tr.cosine(th)) * conv(tr.cosine(ps))                
  long[RPtr][0] := (reg1 + getSign(reg1)*CMNSACLE/2) / CMNSACLE 

  reg1 := conv(tr.sine(ph))*conv(tr.sine(th))
  reg2 := (reg1 + getSign(reg1)*CMNSACLE/2)/CMNSACLE * conv( tr.cosine(ps))  
  temp := (reg2+ getSign(reg2)*CMNSACLE/2 )/CMNSACLE
  long[RPtr][1] :=  temp - (conv(tr.cosine(ph))*conv(tr.sine(ps))+CMNSACLE/2)/CMNSACLE


  reg1 := conv(tr.cosine(ph))*conv(tr.sine(th))
  reg2 := (reg1 + getSign(reg1)*CMNSACLE/2)/CMNSACLE*conv(tr.cosine(ps))
  temp := (reg2 + getSign(reg2)*CMNSACLE/2)/CMNSACLE
  long[RPtr][2] :=  temp + (conv(tr.sine(ph))*conv(tr.sine(ps))+CMNSACLE/2)/CMNSACLE

  reg1 := conv(tr.cosine(th))*conv(tr.sine(ps))
  long[RPtr][3] := (reg1+getSign(reg1)*CMNSACLE/2)/CMNSACLE


  reg1 := conv(tr.sine(ph))*conv(tr.sine(th))
  reg2 := (reg1 + getSign(reg1)*CMNSACLE/2)/CMNSACLE*conv(tr.sine(ps))
  temp := (reg2 + getSign(reg2)*CMNSACLE/2)/CMNSACLE
  long[RPtr][4] := temp + (conv(tr.cosine(ph))*conv(tr.cosine(ps))+CMNSACLE/2)/CMNSACLE   

  reg1 := conv(tr.cosine(ph))*conv(tr.sine(th))
  reg2 := (reg1 + getSign(reg1)*CMNSACLE/2)/CMNSACLE* conv(tr.sine(ps))
  temp :=(reg2 + getSign(reg2)*CMNSACLE/2)/CMNSACLE
  reg1 := conv(tr.sine(ph))*conv(tr.cosine(ps))
  long[RPtr][5] := temp - (reg1+ getSign(reg1)*CMNSACLE/2)/CMNSACLE

  long[RPtr][6] := -conv(tr.sine(th))

  reg1 := conv(tr.sine(ph))*conv(tr.cosine(th))
  long[RPtr][7] := (reg1 + getSign(reg1)*CMNSACLE/2)/CMNSACLE

  reg1 := conv(tr.cosine(ph))*conv(tr.cosine(th))
  long[RPtr][8] := (reg1 + getSign(reg1)* CMNSACLE/2)/CMNSACLE
{ ================================================= 
  conv : convert 65536 to 10_000 
  @value: it is in 65536 : 1

  @returns value in 10_000
=================================================== }
PUB conv(value)

  result := (value*10_000 + 65536/2)/65536

{
d2a  : direction cosince matrix to angle
@ad  : DCM pointer (in value*10_000), matlab convention of data representation
@out : Euler angle pointers in degree*100
}
PUB d2a(RPtr,outPtr) | counter

  'convert 10_000*value to rad*32768
  'origanl, destination, DCM factor, scale factor
  copy_scale(RPtr, @temp3x3, 10_000, 32768) 

'  LONG[out][0] := -tr.asin(LONG[temp][2])                   ' p, roll, psi
'  LONG[out][1] := tr.atan2(LONG[temp][8], LONG[temp][5]) ' q, pitch, theta
'  LONG[out][2] := tr.atan2(LONG[temp][0], LONG[temp][1]) ' r, yaw, phi

                                                    
  LONG[outPtr][0] := -tr.asin(temp3x3[6]*2)           ' q, pitch, theta
  LONG[outPtr][1] := tr.atan2(temp3x3[8], temp3x3[7]) ' p, roll, psi  
  LONG[outPtr][2] := tr.atan2(temp3x3[0], temp3x3[3]) ' r, yaw, phi



PRI copy_scale(oriPtr, desPtr, convention, scale) | counter, reg

  repeat counter from 0 to 8
    'reg := long[oriPtr][counter]
    long[desPtr][counter] := (long[oriPtr][counter] * scale / convention)' + getSign(reg)*convention/2 ) / convention


PUB copy(oriPtr, desPtr) | counter


  repeat counter from 0 to 8
    long[desPtr][counter] := long[oriPtr][counter]

{ ================================================= 
  acc2ang: calcualtes euler angle from accelerometer raw

  @accPtr: accelerometer pointer
  @eulerPtr: euler angle pointer

  @updates eulerPtr in degree*100
=================================================== }
PUB acc2ang(accPtr, eulerPtr) | x, y, temp

  temp := long[accPtr][2] * long[accPtr][2]+long[accPtr][1] * long[accPtr][1]
  x := sqrt(temp)
  y := long[accPtr][0] 

  long[eulerPtr][0] := tr.atan2(x, y)  ' theta
  long[eulerPtr][1] := tr.atan2(-long[accPtr][2], -long[accPtr][1])
  long[eulerPtr][2] := 0