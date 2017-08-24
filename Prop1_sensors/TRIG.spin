'Returns sine and cosine from built in table. Format is in 90.00 ~> 9000

'+/- 180 angle reference

'Matt McCrink
'7/2014

CON

K1 = 5701
K2 = -1645
K3 = 446

PUB SINE(ANGLE) | ng
'Return sine of standard 0 referenced angle +/- 180
'ANGLE input should be scaled by 100, aka 90.01 degrees should be 9001
'Output is in range +/- 65536 (2^16)

    if ANGLE < 0
      ||ANGLE
      ng := true
    else
      ng := false
       
    if ANGLE => 0 AND ANGLE =< 90_00
      ANGLE := word[((ANGLE<<12)/90_00)+$E000]
    else
      ANGLE := word[(((180_00-ANGLE)<<12)/90_00)+$E000]

    if ng
      -ANGLE

    return  ANGLE
    
PUB COSINE(ANGLE)
'Return cosine of standard 0 referenced angle +/- 180
'ANGLE input should be scaled by 100, aka 90.01 degrees should be 9001
'Output is in range +/- 65536 (2^16)

    ||ANGLE
    
    if ((ANGLE => 0) AND (ANGLE =< 90_00))
      ANGLE += 90_00
    else
      ANGLE -=  270_00

    return SINE(ANGLE)

PUB TANGENT(ANGLE)
'Return tangent of standard 0 referenced angle +/- 180
'ANGLE input should be scaled by 100, aka 90.01 degrees should be 9001
'Output is in range +/- 65536 (2^16)

if ANGLE == 90_00 OR ANGLE == -90_00
  return 0
return ((SINE(ANGLE)*32768)/(COSINE(ANGLE)))<<1

PUB SINE_360(ANGLE)
' Return sine of North referenced 360 degree angle
' ANGLE input should be scaled by 100
'Output is in range +/- 65536 (2^16)

return(-SINE(ANGLE-180_00))

PUB COSINE_360(ANGLE)
' Return sine of North referenced 360 degree angle
' ANGLE input should be scaled by 100
'Output is in range +/- 65536 (2^16)

return(-COSINE(ANGLE - 180_00))


{ ================================================= 
ASIN : calculate value to degree*100, centi degree 
@ix: value in +/- 66536 ~ +/-1
@returns angle in degree*100
=================================================== }
PUB ASIN(ix):pivot | tem,ng,pivot_axis
'Expects scaled value between +/- 65536 (-1 and 1). Output range limited to 0 to 90 degrees in degrees*100

ng~

if ix < 0
  ng := true
  ||ix

pivot := 2048
pivot_axis := constant(2048>>1)

repeat 11
  tem := word[$E000+pivot]
  if tem > ix
    pivot -= pivot_axis
  elseif tem < ix
    pivot += pivot_axis
  else
    quit
  pivot_axis >>= 1

pivot := ((pivot-1)*439 +((ix-word[$DFFE+pivot])*constant(439*4))/(word[$E002+pivot]-word[$DFFE+pivot]))**21496311 - 4'1001/2000/100-439/100 ''1001 is a boost factor to account for numerical truncation

if ng
  -pivot

PUB ACOS(ix)
'Expects scaled value between +/- 16384 (-1 and 1). Output range limited to 0 to 90 degrees in degrees*100

return (90_00-ASIN(ix))

PUB ATAN(ix,iy) | iAngle,iRatio,iTmp

'calculates 100*atan(iy/ix) range 0 to 9000 for all ix, iy positive in range 0 to 32767
if ((ix == 0) AND (iy == 0))
  return (0)
if ((ix == 0) AND (iy <> 0))
  return (9000)

'check for non-pathological cases
if (iy < ix)
  iRatio := (iy<<15)/ix '/* return a fraction in range 0. to 32767 = 0. to 1.
else
  iRatio := (ix<<15)/iy '/* return a fraction in range 0. to 32767 = 0. to 1.
 
'first, third and fifth order polynomial approximation
iAngle := K1 * iRatio
iTmp := ((iRatio * iRatio) >> 10) * (iRatio >> 5)
iAngle += (iTmp >> 15) * K2
iTmp := (iTmp >> 20) * (iRatio >> 5) * (iRatio >> 5)
iAngle += (iTmp >> 15) * K3
iAngle := iAngle >> 15

'check if above 45 degrees */
if (iy > ix)
  iAngle := (9000 - iAngle)

return (iAngle)

PUB  ATAN2 (ix,iy) : iResult
'calculates 100*atan2(iy/ix)=100*atan2(iy,ix) in deg for ix, iy in range -32768 to 32767
'check for -32768 which is not handled correctly

'same arguemnts as Matlab's atan2(x,y). But arguements's order is opposite.

repeat while ((||ix) > 32768) OR ((||iy) > 32768) 'if not in proper range scale accordingly 
  ix ~>= 1
  iy ~>= 1  

if (ix == -32768)
  ix++
if (iy == -32768)
  iy++
  
'check for quadrants
if ((ix => 0) AND (iy => 0)) 'range 0 to 90 degrees
  iResult := ATAN(ix, iy)
elseif ((ix =< 0) AND (iy => 0))  'range 90 to 180 degrees
  iResult := 180_00 - (ATAN( ||ix,iy))
elseif ((ix =< 0) AND (iy =< 0))  'range -180 to -90 degrees
  iResult := -180_00 + (ATAN(||ix, ||iy))
else 'ix => 0 and iy =< 0 'giving range -90 to 0 degrees
  iResult := ( -(ATAN(ix,||iy)))