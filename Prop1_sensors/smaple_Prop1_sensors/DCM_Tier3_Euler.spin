CON
  _clkmode = xtal1 + pll16x                                                    
  _xinfreq = 6_250_000

  CMNSCALE = 10_000
  db = 0

  
OBJ

  t2_sensor  : "DCM_Tier2_LP.spin"
  debug      : "fullDuplexSerial4Port_tier2.spin"
  tr         : "TRIG"
  math       : "MyMath.spin"
  
VAR

  'Reporter Pointers
  long eulerPtr[3], gyroPtr[3], accPtr[3], magPtr[3]


  'MPU varaibles
  long t3_acc[3], t3_gyro[3], t3_mag[3]
  long t3_mpuCogId1, t3_mpuStack1[50],t3_mpuCogId2, t3_mpuStack2[50],t3_mpuCogId3, t3_mpuStack3[50]
  long t3_dt_mpu, t3_prev_mpu, t3_freq_mpu
  byte gyroIsReady, accIsReady, magIsReady
  
  'DCM variables
  long isStabilized
  long t3_dcmCogId, t3_dcmStack[200]
  long t3_dcm[9], t3_eye[9], t3_imdt[9], t3_imdt2[9], t3_imdt3[9]
  long t3_omega[3], t3_euler[3], t3_acc_body[3], t3_acc_earth[3], t3_mag_earth[3]
  long t3_err_acc_earth[3], t3_err_mag_earth[3], t3_err_body[3], t3_err_earth[3], t3_I[3],t3_intrmdtI[3]  
  long t3_dt_dcm, t3_prev_dcm, t3_freq_dcm, t3_prev_compensation
  long t3_avgAcc[3], t3_prevAccX[20], t3_prevAccY[20], t3_prevAccZ[20], t3_avgAccInter[20], t3_avgAccCnt
  
  'first values for dcm
  long t3_first_mag[3], t3_first_euler_in[3],t3_first_euler_out[3]
  long t3_first_dcm[9]

  'general
  long t3_counter, base[10], dt[10]
  
  'DCM anomaly checker
  byte t3_an_skew_omega

  'Debug Window
  long t3_dt_debug, t3_prev_debug
  long t3_acc_d[3], t3_gyro_d[3], t3_mag_d[3]
  long t3_dcm_d[9], t3_eye_d[9], t3_imdt_d[9],t3_imdt2_d[9], t3_imdt3_d[9], t3_first_dcm_d[9]
  long t3_omega_d[3], t3_euler_d[3]
  long t3_matrix_monitor1[9], t3_matrix_monitor2[9], t3_matrix_monitor3[9]   
  long t3_matrix_monitor1_d[9], t3_matrix_monitor2_d[9], t3_matrix_monitor3_d[9]   
  long t3_avg_acc_d[3], t3_first_mag_d[3], t3_first_euler_in_d[3],t3_first_euler_out_d[3] 
  
PUB main
  debug.quickStartDebug  

  masterKey_tier3

  repeat
    freezResult      
    'debug.clear(db)
    {
    printDt
    printGyro
    printAcc
    printCurrentMag

    printFirstEulerInput
    printFisrtDCM
    printFirstEulerOutput
    
    printOmega
    printDCM     }
    
    printEuler

   ' printMatrixMonitor
    waitcnt(cnt + clkfreq/50)

{
PUB getEulerAngles(xPtr)
  eulerPtr := xPtr
PUB getAcc(xPtr)
  accPtr := xPtr
PUB getGyro(xPtr)
  gyroPtr := xPtr    
PUB getMag(xPtr)
  magPtr := xPtr
}

PUB getEulerPtr
  return @t3_euler
PUB getAccPtr
  return @t3_acc
PUB getGyroPtr
  return @t3_gyro    
PUB getMagPtr
  return @t3_mag

  
PUB masterKey_tier3

  startMpu  ' cognew is called for reading MPU
  
  startDCM  ' cognew is called for calculating DCM

PRI stopMpu
  if t3_mpuCogId1 OR t3_mpuCogId2  OR t3_mpuCogId3
    cogstop(t3_mpuCogId1 ~ -1)
    cogstop(t3_mpuCogId2 ~ -1)
    cogstop(t3_mpuCogId3 ~ -1)

PRI startMpu
  stopMpu
  t3_mpuCogId1 := cognew(runGyro,   @t3_mpuStack1) + 1
  t3_mpuCogId2 := cognew(runAccMag, @t3_mpuStack2) + 1
  t3_mpuCogId3 := cognew(readMpu,   @t3_mpuStack3) + 1

PRI runGyro
  t2_sensor.runGyro_cog

PRI runAccMag
  t2_sensor.runAccMag_cog

PRI readMpu | goCompensation
  base[0] := cnt 
  base[1] := cnt
  base[2] := cnt
  repeat
    if (cnt - base[0]) => clkfreq/138  '~130 Hz 
      t2_sensor.getAcc_t2(@t3_acc)
      accIsReady := true
      dt[0] := cnt - base[0] 
      base[0] := cnt
    
    if (cnt - base[1]) => clkfreq/11  '~10 Hz   
      t2_sensor.getMag_t2(@t3_mag)
      magIsReady := true 
      dt[1] := cnt - base[1]
      base[1] := cnt 
    
    if (cnt - base[2]) => clkfreq/188  '~175 Hz  
      t2_sensor.getGyro_t2(@t3_gyro)
      gyroIsReady := true
      dt[2] := cnt - base[2]
      base[2] := cnt    


  
PUB setUpDCM | counter

  waitcnt(cnt + clkfreq)
  t2_sensor.getAcc_t2(@t3_acc)      ' updates acc variable
  t2_sensor.getMag_t2(@t3_mag)      ' updates mag variable

  repeat counter from 0 to 2
    t3_avgAcc[counter] := t3_acc[counter]    ' updates avg. acc variable

  repeat counter from 0 to 2
    t3_I[counter] := 0                               ' initialize error vector that will be used for gyro bias compensation
    t3_euler[counter] := 0                           ' initialize eular angles => all zeros for now
    't3_first_mag[counter] := t3_mag[counter]         ' initialize heading


  acc2ang                                       ' based on first accelerometer, get first eular angles
  math.a2d(@t3_dcm,@t3_first_euler_in)          ' based on the first eular angles, set up a initial DCM
  d2a                                           ' based on the DCM, calculate euler;this result should be same as first euler

  ' calculates heading in ground frame
  't3_first_mag[0] := t3_mag[0] - t3_first_euler_in[0]*15/1000'(t3_dcm[0]*t3_first_mag[0] + t3_dcm[1]*t3_first_mag[1] + t3_dcm[2]*t3_first_mag[2])/CMNSCALE
  't3_first_mag[1] := t3_mag[1] + t3_first_euler_in[1]*15/1000'(t3_dcm[3]*t3_first_mag[0] + t3_dcm[4]*t3_first_mag[1] + t3_dcm[5]*t3_first_mag[2])/CMNSCALE
  't3_first_mag[2] := 0'(t3_dcm[6]*t3_first_mag[0] + t3_dcm[7]*t3_first_mag[1] + t3_dcm[8]*t3_first_mag[2])/CMNSCALE
  repeat t3_counter from 0 to 2
     t3_matrix_monitor1[t3_counter] := t3_first_mag[t3_counter]
  t3_first_euler_in[2] := calcHeading(t3_first_euler_in[0],t3_first_euler_in[1],t3_mag[0], t3_mag[1]) ' based on first accelerometer, get first eular angles
  math.a2d(@t3_dcm,@t3_first_euler_in)          ' based on the first eular angles, set up a initial DCM
  d2a


  't3_dcm and t3_euler are used through out the algorithm
  ' below assignments are for debugging purpose
  repeat counter from 0 to 8
    t3_first_dcm[counter] := t3_dcm[counter]
    if counter < 3
      t3_first_euler_out[counter] := t3_euler[counter]

PUB calcHeading(pitch, roll, magBodyX, magBodyY)| temp

  magBodyX := magBodyX - pitch*13/1000
  magBodyY := magBodyY + roll*13/1000


  if (magBodyX =< -30 )  ' North
    temp :=  magBodyY*90/60
  else 'South
    if (magBodyY =< 0) 
      temp :=  -180 - magBodyY*90/60       'SW             
    else
      temp :=  180 - magBodyY*90/60        'SE

 {
  if (magBodyX =>0 AND magBodyY =>0)         '(+,+) btw +90 to +180 degree
    temp :=  180 - magBodyY*90/50

  elseif (magBodyX => 0 AND magBodyY < 0)    '(+,-) btw -90 to -180 degree 
    temp :=  -180 - magBodyY*90/50 
                                                          
  elseif (magBodyX < 0 AND magBodyY => 0)    '(-,+) btw  +0 to +90 degree
    temp :=  magBodyY*90/50  

  elseif (magBodyX < 0 AND magBodyY < 0)     '(-,-) btw  -0 to -90 degree
    temp :=  magBodyY*90/50
               }
  return temp*100

PUB acc2ang | x, y, temp

  temp := t3_avgAcc[2] * t3_avgAcc[2]+t3_avgAcc[1] * t3_avgAcc[1]
  x := ^^(temp)
  y := t3_acc[0]

  t3_first_euler_in[0] := tr.atan2(x, y)  ' theta
  t3_first_euler_in[1] := tr.atan2(-t3_avgAcc[2], -t3_avgAcc[1])
  t3_first_euler_in[2] := 0

PUB getSign(value)

  if value >= 0
    result := 1
  else
    result := -1

PUB getEye
  t3_eye[0] := 10000
  t3_eye[1] := 0
  t3_eye[2] := 0
  t3_eye[3] := 0
  t3_eye[4] := 10000
  t3_eye[5] := 0
  t3_eye[6] := 0
  t3_eye[7] := 0
  t3_eye[8] := 10000


PUB d2a | counter, temp1[9]

  repeat counter from 0 to 8
    temp1[counter] := t3_dcm[counter] * 32768 /CMNSCALE

  t3_euler[0] := -tr.asin(temp1[6]*2)           ' q, pitch, theta
  t3_euler[1] := tr.atan2(temp1[8], temp1[7]) ' p, roll, psi
  t3_euler[2] := tr.atan2(temp1[0], temp1[3]) ' r, yaw, phi
  'if (t3_euler[2] > 0)
    't3_euler[2] := (t3_euler[2]-2300)


PUB stopDcm
  if t3_dcmCogId
    cogstop(t3_dcmCogId ~ -1)

PUB startDcm
  stopDcm
  t3_dcmCogId := cognew(runDcm, @t3_dcmStack) + 1

PUB runDcm 

  setUpDCM  ' initial angle calculation based on acc/

  repeat
    base[4] := cnt
    repeat while NOT gyroIsReady
    'calculate DCM w/o compensation
    calcDcm
    gyroIsReady := false      
     
    'compensate only when it is available
    if accIsReady
      getAvgAcc
      t3_avgAccCnt++
      accIsReady := false
      if t3_avgAccCnt => 14
        calcCompensation
        clearAvgAcc
        t3_avgAccCnt := 0
  
    d2a      

    dt[4] := cnt - base[4]
    t3_dt_mpu := dt[4]
    
PUB clearAvgAcc | i
  repeat i from 0 to 12
    t3_prevAccX[i] := 0
    
PUB getAvgAcc | i, avgCoef

  avgCoef:= 14   'once a 13 iteration = 10 Hz of compensation 

  repeat i from 0 to (avgCoef-2)
    t3_prevAccX[i] := t3_prevAccX[i+1]
    t3_prevAccY[i] := t3_prevAccY[i+1]
    t3_prevAccZ[i] := t3_prevAccZ[i+1] 
  t3_prevAccX[avgCoef-1] := t3_acc[0]
  t3_prevAccY[avgCoef-1] := t3_acc[1]
  t3_prevAccZ[avgCoef-1] := t3_acc[2]
    
  t3_avgAccInter[0] := 0
  t3_avgAccInter[1] := 0
  t3_avgAccInter[2] := 0
    
  repeat i from 0 to (avgCoef-1)
    t3_avgAccInter[0] += t3_prevAccX[i]'/avgCoef 
    t3_avgAccInter[1] += t3_prevAccY[i]'/avgCoef
    t3_avgAccInter[2] += t3_prevAccZ[i]'/avgCoef

  t3_avgAcc[0] := t3_avgAccInter[0] /avgCoef
  t3_avgAcc[1] := t3_avgAccInter[1] /avgCoef
  t3_avgAcc[2] := t3_avgAccInter[2] /avgCoef

  

PUB calcDcm

  dcmStep1
  dcmStep2

PUB calcCompensation

  dcmStep3
  dcmStep4
  dcmStep5
  dcmStep6
  dcmStep7
  dcmStep8


'-----------------------------------------------------------
'-----------------------------------------------------------
'-----------------------------------------------------------
'-----------------------------------------------------------
  'CMNSCALE: common scale: 10_000
  'Max omega value could occur with 250 degree setting of mpu
  'is 4.36 rad/s(=250 (deg) *3.14/180 );
  'It is represented as 43611 in spin code by multiplying CMNSCALE
  ' Note1 : spin DCM representation is in 10^4 of the value; for ex) 0.9 is 9000 in spin
  ' Note2 : if two elements in 10^4 scale are multiplied, the result should be divided by 10^4

'Step1: Skew and integration

PUB getOmega  |ki

  ki:= 0'30
  repeat t3_counter from 0 to 2
    t3_omega[t3_counter] := t3_gyro[t3_counter]*CMNSCALE/131*314/100/180 + t3_I[t3_counter]* ki/100    '10_000 rad/s

PUB dcmStep1

  getOmega
  
  'repeat t3_counter from 0 to 2
  '  t3_matrix_monitor2[t3_counter] := t3_omega[t3_counter]

  ' Skew
  t3_imdt[0] := 0
  t3_imdt[1] := -t3_omega[2]
  t3_imdt[2] := t3_omega[1]
  t3_imdt[3] := t3_omega[2]
  t3_imdt[4] := 0
  t3_imdt[5] := -t3_omega[0]
  t3_imdt[6] := -t3_omega[1]
  t3_imdt[7] := t3_omega[0]
  t3_imdt[8] := 0

  'skewOmegaAnomalyChecker

 ' repeat t3_counter from 0 to 8
 '   t3_matrix_monitor2[t3_counter] := t3_imdt[t3_counter]

  t3_freq_mpu := clkfreq / t3_dt_mpu

  getEye
  repeat t3_counter from 0 to 8
    t3_imdt[t3_counter] := t3_imdt[t3_counter] / t3_freq_mpu        ' omega * dt
    t3_imdt[t3_counter] := t3_eye[t3_counter] + t3_imdt[t3_counter] ' I + omega*dt

'  repeat t3_counter from 0 to 8
'    t3_matrix_monitor2[t3_counter] := t3_imdt[t3_counter]


  ' DCM*(I + omega*dt)
  t3_imdt2[0] := t3_dcm[0]*t3_imdt[0] + t3_dcm[1]*t3_imdt[3] + t3_dcm[2]*t3_imdt[6]
  t3_imdt2[1] := t3_dcm[0]*t3_imdt[1] + t3_dcm[1]*t3_imdt[4] + t3_dcm[2]*t3_imdt[7]
  t3_imdt2[2] := t3_dcm[0]*t3_imdt[2] + t3_dcm[1]*t3_imdt[5] + t3_dcm[2]*t3_imdt[8]
  t3_imdt2[3] := t3_dcm[3]*t3_imdt[0] + t3_dcm[4]*t3_imdt[3] + t3_dcm[5]*t3_imdt[6]
  t3_imdt2[4] := t3_dcm[3]*t3_imdt[1] + t3_dcm[4]*t3_imdt[4] + t3_dcm[5]*t3_imdt[7]
  t3_imdt2[5] := t3_dcm[3]*t3_imdt[2] + t3_dcm[4]*t3_imdt[5] + t3_dcm[5]*t3_imdt[8]
  t3_imdt2[6] := t3_dcm[6]*t3_imdt[0] + t3_dcm[7]*t3_imdt[3] + t3_dcm[8]*t3_imdt[6]
  t3_imdt2[7] := t3_dcm[6]*t3_imdt[1] + t3_dcm[7]*t3_imdt[4] + t3_dcm[8]*t3_imdt[7]
  t3_imdt2[8] := t3_dcm[6]*t3_imdt[2] + t3_dcm[7]*t3_imdt[5] + t3_dcm[8]*t3_imdt[8]

  repeat t3_counter from 0 to 8
    t3_imdt2[t3_counter] := t3_imdt2[t3_counter]/CMNSCALE
    t3_dcm[t3_counter] := t3_imdt2[t3_counter]

 ' repeat t3_counter from 0 to 8
 '   t3_matrix_monitor1[t3_counter] := t3_imdt2[t3_counter]


'Step 2: orthogonalization
PUB dcmStep2| il , temp1[9],  col1[3], col2[3], col3[3], err_orth, x_orth[3], y_orth[3], z_orth[3], x_norm[3], y_norm[3], z_norm[3], magnitude[3]

  ' make column vectors
  't3_DCM (i,j) from left to right
  ' 0 1 2
  ' 3 4 5
  ' 6 7 8
  repeat il from 0 to 2
    col1[il] := t3_dcm[3*il]    '0, 3, 6 th elements: first column of DCM
    col2[il] := t3_dcm[3*il+1]  'second column of DCM
    col3[il] := t3_dcm[3*il+2]  'third column of DCM

  'err_orth = dot(R(:,1)', R(:,2));
  err_orth := (col1[0]*col2[0] + col1[1]*col2[1] + col1[2]*col2[2])/CMNSCALE

  'x_orth = R(:,1) - err_orth/2*R(:,2);
  'y_orth = R(:,2) - err_orth/2*R(:,1);
  repeat il from 0 to 2
    x_orth[il] := col1[il] - col2[il]*err_orth/2/CMNSCALE
    y_orth[il] := col2[il] - col1[il]*err_orth/2/CMNSCALE

  'z_orth = cross(x_orth, y_orth);
  z_orth[0] := (x_orth[1]*y_orth[2] - x_orth[2]*y_orth[1])/CMNSCALE
  z_orth[1] := (x_orth[2]*y_orth[0] - x_orth[0]*y_orth[2])/CMNSCALE
  z_orth[2] := (x_orth[0]*y_orth[1] - x_orth[1]*y_orth[0])/CMNSCALE

  'x_norm = 0.5*(3-dot(x_orth, x_orth))*x_orth;
  'y_norm = 0.5*(3-dot(y_orth, y_orth))*y_orth;
  'z_norm = 0.5*(3-dot(z_orth, z_orth))*z_orth;
  magnitude[0] := (x_orth[0]*x_orth[0] + x_orth[1]*x_orth[1] + x_orth[2]*x_orth[2])/CMNSCALE
  magnitude[1] := (y_orth[0]*y_orth[0] + y_orth[1]*y_orth[1] + y_orth[2]*y_orth[2])/CMNSCALE
  magnitude[2] := (z_orth[0]*z_orth[0] + z_orth[1]*z_orth[1] + z_orth[2]*z_orth[2])/CMNSCALE

  repeat il from 0 to 2
    x_norm[il] := (3*x_orth[il] - magnitude[0]*x_orth[il]/CMNSCALE)/2
    y_norm[il] := (3*y_orth[il] - magnitude[1]*y_orth[il]/CMNSCALE)/2
    z_norm[il] := (3*z_orth[il] - magnitude[2]*z_orth[il]/CMNSCALE)/2


  'R = [x_norm y_norm z_norm];
  repeat il from 0 to 2
    t3_dcm[il*3] := x_norm[il]
    t3_dcm[il*3+1] := y_norm[il]
    t3_dcm[il*3+2] := z_norm[il]

  'repeat t3_counter from 0 to 8
  '  t3_matrix_monitor2[t3_counter] := t3_dcm[t3_counter] -t3_matrix_monitor1[t3_counter]


' Step 3: get acc and mag in ground frames
PUB dcmStep3  | mag_norm, mag_earth[3]

  ' Accelerometer: convert units in body frame
  '9.81 m/s^2 is representedas 98100
  'mpu setting is 2g -> max g can occur is 98100*2 = 196200 -> no overflow
  t3_acc_body[0] := t3_acc[0]*CMNSCALE/16384  * 981 /100
  t3_acc_body[1] := t3_acc[1]*CMNSCALE/16384  * 981 /100
  t3_acc_body[2] := t3_acc[2]*CMNSCALE/16384  * 981 /100


  ' Accelerometer: acc in ground frame -> t3_acc_earth in CMNSCALE
  t3_acc_earth[0] := (t3_dcm[0]/10*t3_acc_body[0]/10 + t3_dcm[1]/10*t3_acc_body[1]/10 + t3_dcm[2]/10*t3_acc_body[2]/10) /100
  t3_acc_earth[1] := (t3_dcm[3]/10*t3_acc_body[0]/10 + t3_dcm[4]/10*t3_acc_body[1]/10 + t3_dcm[5]/10*t3_acc_body[2]/10) /100
  t3_acc_earth[2] := (t3_dcm[6]/10*t3_acc_body[0]/10 + t3_dcm[7]/10*t3_acc_body[1]/10 + t3_dcm[8]/10*t3_acc_body[2]/10) /100


  repeat t3_counter from 0 to 2
     t3_matrix_monitor1[t3_counter*3] := t3_acc_body[t3_counter]
     t3_matrix_monitor1[t3_counter*3 + 1] := t3_mag[t3_counter]
     t3_matrix_monitor1[t3_counter*3 + 2] := 0  ' t3_err_earth[t3_counter]

  if NOT magIsReady
    t3_mag[0] := 1
    t3_mag[1] := 0
    t3_mag[2] := 0
  else
    magIsReady := false
  
  ' Magnetometer: mag in ground frame = ||mag_earth|| = 10_000
 ' mag_earth[0] := (t3_dcm[0]/100*t3_mag[0] + t3_dcm[1]/100*t3_mag[1] + t3_dcm[2]/100*t3_mag[2])/100'/10_000  ' in non CMNSCALE
 ' mag_earth[1] := (t3_dcm[3]/100*t3_mag[0] + t3_dcm[4]/100*t3_mag[1] + t3_dcm[5]/100*t3_mag[2])/100'/10_000
 ' mag_earth[2] :=  0


 ' mag_norm := ^^(mag_earth[0]*mag_earth[0] + mag_earth[1]*mag_earth[1])
 ' t3_mag_earth[0] := (1000*mag_earth[0])/mag_norm
 ' t3_mag_earth[1] := (1000*mag_earth[1])/mag_norm
 ' t3_mag_earth[2] := 0
  '||mag_earth|| = 10_000

  repeat t3_counter from 0 to 2
 '   t3_matrix_monitor2[t3_counter*3+0] := t3_acc_earth[t3_counter]
    t3_matrix_monitor2[t3_counter*3+1] := t3_mag_earth[t3_counter]

  mag_norm := ^^(t3_mag_earth[0]*t3_mag_earth[0] + t3_mag_earth[1]*t3_mag_earth[1])
  t3_matrix_monitor3[3] := mag_norm

' Step 4: Calculate Error in earth frame
PUB dcmStep4  | g[3], magSize[2], norm_mag_earth[3], norm_first_mag_earth[3], norm_acc

  ' Ground frame gravity vector g[]
  g[0] := 0
  g[1] := 0
  g[2] := -98100

  ' commented outs: g[0] and g[1] are zero
  ' divided by CMNSCALE because DMC is in 10^4 and g is in 10^4 -> needs to keep in 10^4 scale


  norm_acc :=  ^^(t3_acc_earth[0]/100*t3_acc_earth[0]/100 + t3_acc_earth[1]/100*t3_acc_earth[1]/100 + t3_acc_earth[2]/100*t3_acc_earth[2]/100) * ^^(CMNSCALE)
  t3_matrix_monitor3[0] := norm_acc

  t3_err_acc_earth[0] := CMNSCALE*t3_acc_earth[1]  / norm_acc  'in CMNSCALE
  t3_err_acc_earth[1] := CMNSCALE*(-t3_acc_earth[0]) / norm_acc  'in CMNSCALE
  t3_err_acc_earth[2] := 0


   'err_mag_earth = cross(Mag_earth(i,:),  Mag_earth(1,:));
 ' t3_err_mag_earth[0] := 0'(norm_mag_earth[1]*norm_first_mag_earth[2] - norm_mag_earth[2]*norm_first_mag_earth[1])/CMNSCALE
 ' t3_err_mag_earth[1] := 0'(norm_mag_earth[2]*norm_first_mag_earth[0] - norm_mag_earth[0]*norm_first_mag_earth[2])/CMNSCALE
 ' t3_err_mag_earth[2] := -t3_mag_earth[1]


  'Err_earth(i,:) = err_acc_earth + err_mag_earth;
  t3_err_earth[0] := t3_err_acc_earth[0]
  t3_err_earth[1] := t3_err_acc_earth[1]
  t3_err_earth[2] := -(calcHeading(t3_euler[0], t3_euler[1], t3_mag[0], t3_mag[1]) - t3_euler[2])    't3_err_mag_earth[2]    


  repeat t3_counter from 0 to 2
    t3_matrix_monitor2[t3_counter*3] := t3_acc_earth[t3_counter]
    't3_matrix_monitor2[t3_counter*3 + 1] := t3_mag_earth[t3_counter]
    't3_matrix_monitor1[t3_counter*3 + 2] := t3_err_earth[t3_counter]

' calcualte error in body frame
PUB dcmStep5 | DCMTrans[9]

  'Transpose of DCM
  DCMTrans[0] := t3_dcm[0]
  DCMTrans[1] := t3_dcm[3]
  DCMTrans[2] := t3_dcm[6]
  DCMTrans[3] := t3_dcm[1]
  DCMTrans[4] := t3_dcm[4]
  DCMTrans[5] := t3_dcm[7]
  DCMTrans[6] := t3_dcm[2]
  DCMTrans[7] := t3_dcm[5]
  DCMTrans[8] := t3_dcm[8]

  ' err body is  in NON CMNSCALE
  t3_err_body[0] := (DCMTrans[0]/100*t3_err_earth[0] + DCMTrans[1]/100*t3_err_earth[1] + DCMTrans[2]/100*t3_err_earth[2])/100 '/CMNSCALE
  t3_err_body[1] := (DCMTrans[3]/100*t3_err_earth[0] + DCMTrans[4]/100*t3_err_earth[1] + DCMTrans[5]/100*t3_err_earth[2])/100 '/CMNSCALE
  t3_err_body[2] := (DCMTrans[6]/100*t3_err_earth[0] + DCMTrans[7]/100*t3_err_earth[1] + DCMTrans[8]/100*t3_err_earth[2])/100 '/CMNSCALE

  t3_err_body[2] /= 200

  repeat t3_counter from 0 to 2
    t3_matrix_monitor3[t3_counter*3+1] := t3_err_earth[t3_counter]
    t3_matrix_monitor3[t3_counter*3+2] := t3_err_body[t3_counter]

' proportional compensation
PUB dcmStep6 | kp, kp_mag

  kp := 120_000
  kp_mag := kp*130

  t3_imdt[0] := 0
  t3_imdt[1] := t3_err_body[2] * kp_mag / CMNSCALE
  t3_imdt[2] := -t3_err_body[1]  * kp / CMNSCALE
  t3_imdt[3] := -t3_err_body[2]  * kp_mag / CMNSCALE
  t3_imdt[4] := 0
  t3_imdt[5] := t3_err_body[0] * kp / CMNSCALE
  t3_imdt[6] := t3_err_body[1] * kp / CMNSCALE
  t3_imdt[7] := -t3_err_body[0]  * kp / CMNSCALE
  t3_imdt[8] := 0

'  repeat t3_counter from 0 to 8
'    t3_matrix_monitor1[t3_counter] :=  t3_imdt[t3_counter]


  t3_freq_mpu := clkfreq / t3_dt_mpu

  getEye

  ' (eye(3) + skew(Err_body(i,:))*kp*dt(i))
  repeat t3_counter from 0 to 8
    t3_imdt[t3_counter] := t3_imdt[t3_counter] / t3_freq_mpu
    t3_imdt[t3_counter] := t3_eye[t3_counter] + t3_imdt[t3_counter]


'  repeat t3_counter from 0 to 8
'    t3_matrix_monitor2[t3_counter] :=  t3_imdt[t3_counter]

  'R*(eye(3) + skew(Err_body(i,:))*kp*dt(i))
  t3_imdt2[0] := t3_dcm[0]*t3_imdt[0] + t3_dcm[1]*t3_imdt[3] + t3_dcm[2]*t3_imdt[6]
  t3_imdt2[1] := t3_dcm[0]*t3_imdt[1] + t3_dcm[1]*t3_imdt[4] + t3_dcm[2]*t3_imdt[7]
  t3_imdt2[2] := t3_dcm[0]*t3_imdt[2] + t3_dcm[1]*t3_imdt[5] + t3_dcm[2]*t3_imdt[8]
  t3_imdt2[3] := t3_dcm[3]*t3_imdt[0] + t3_dcm[4]*t3_imdt[3] + t3_dcm[5]*t3_imdt[6]
  t3_imdt2[4] := t3_dcm[3]*t3_imdt[1] + t3_dcm[4]*t3_imdt[4] + t3_dcm[5]*t3_imdt[7]
  t3_imdt2[5] := t3_dcm[3]*t3_imdt[2] + t3_dcm[4]*t3_imdt[5] + t3_dcm[5]*t3_imdt[8]
  t3_imdt2[6] := t3_dcm[6]*t3_imdt[0] + t3_dcm[7]*t3_imdt[3] + t3_dcm[8]*t3_imdt[6]
  t3_imdt2[7] := t3_dcm[6]*t3_imdt[1] + t3_dcm[7]*t3_imdt[4] + t3_dcm[8]*t3_imdt[7]
  t3_imdt2[8] := t3_dcm[6]*t3_imdt[2] + t3_dcm[7]*t3_imdt[5] + t3_dcm[8]*t3_imdt[8]

  repeat t3_counter from 0 to 8
    t3_imdt2[t3_counter] := t3_imdt2[t3_counter]/CMNSCALE
    t3_dcm[t3_counter] := t3_imdt2[t3_counter]


' integrate error
PUB dcmStep7

  repeat t3_counter from 0 to 2
    t3_intrmdtI[t3_counter] += t3_err_body[t3_counter]
    't3_I[t3_counter] := -100 #> t3_intrmdtI[t3_counter] <# 100
     if (t3_intrmdtI[t3_counter]  > 100)
      t3_intrmdtI[t3_counter]  := 100
     elseif(t3_intrmdtI[t3_counter] <-100)
      t3_intrmdtI[t3_counter] := -100
    t3_I[t3_counter] :=  t3_intrmdtI[t3_counter]

  repeat t3_counter from 0 to 2
    t3_matrix_monitor2[t3_counter*3+2] := t3_I[t3_counter]

PUB dcmStep8

  ' done in 'getOmega



' this methods is for debugging purpose
PUB freezResult | local_c

 ' long t3_dcm_d[9], t3_eye_d[9], t3_imdt_d[9]
 ' long t3_omega_d[3]
  repeat local_c from 0 to 8
    if local_c < 3
      t3_acc_d[local_c] := t3_acc[local_c]
      t3_gyro_d[local_c] := t3_gyro[local_c]
      t3_mag_d[local_c] := t3_mag[local_c]
      t3_omega_d[local_c] := t3_omega[local_c]
      t3_euler_d[local_c] := t3_euler[local_c]
      t3_avg_acc_d[local_c] := t3_avgAcc[local_c]
      t3_first_mag_d[local_c] := t3_first_mag[local_c]
      t3_first_euler_in_d[local_c] := t3_first_euler_in[local_c]
      t3_first_euler_out_d[local_c] := t3_first_euler_out[local_c]

    t3_dcm_d[local_c] := t3_dcm[local_c]
    t3_first_dcm_d[local_c] := t3_first_dcm[local_c]
    t3_imdt_d[local_c] := t3_imdt[local_c]
    t3_imdt2_d[local_c] := t3_imdt2[local_c]
    t3_imdt3_d[local_c] := t3_imdt3[local_c]

    t3_matrix_monitor1_d[local_c] := t3_matrix_monitor1[local_c]
    t3_matrix_monitor2_d[local_c] := t3_matrix_monitor2[local_c]
    t3_matrix_monitor3_d[local_c] := t3_matrix_monitor3[local_c]

PRI printCurrentMag
  debug.newline(db)
  debug.strLn(db,String("Current Magnetometer "))
  debug.str(db,String("magX = "))
  debug.decln(db,t3_mag_d[0])

  debug.str(db,String("magY = "))
  debug.decln(db,t3_mag_d[1])

  debug.str(db,String("magZ = "))
  debug.decln(db,t3_mag_d[2])

PRI printAcc | counter

  debug.newline(db)
  debug.str(db,String("accX = "))
  debug.decln(db,t3_acc_d[0])

  debug.str(db,String("accY = "))
  debug.decln(db,t3_acc_d[1])

  debug.str(db,String("accZ = "))
  debug.decln(db,t3_acc_d[2])

PRI printGyro

  debug.newline(db)
  debug.strLn(db,String("Gyro"))
  debug.str(db,String("X: "))
  debug.decLn(db,t3_gyro_d[0])
  debug.str(db,String("Y: "))
  debug.decLn(db,t3_gyro_d[1])
  debug.str(db,String("Z: "))
  debug.decLn(db,t3_gyro_d[2])

PUB printDt

  debug.str(db,String("         Gyro time       "))
  debug.str(db,String("Acc time        "))
  debug.str(db,String("Mag time        "))
  debug.strLn(db,String("DCM time        "))

  debug.str(db,String("dt          "))
  debug.dec(db,dt[2])
  debug.str(db,String("         "))
  debug.dec(db,dt[0])
  debug.str(db,String("         "))
  debug.dec(db,dt[1])
  debug.str(db,String("          "))
  debug.dec(db,dt[4])
  debug.newline(db)

  debug.str(db,String("freq(Hz)       "))
  debug.dec(db,clkfreq/dt[2])
  debug.str(db,String("            "))
  debug.dec(db,clkfreq/dt[0])
  debug.str(db,String("              "))
  debug.dec(db,clkfreq/dt[1])
  debug.str(db,String("             "))
  debug.dec(db,clkfreq/dt[4])
  debug.newline(db)

PRI printOmega

  debug.newline(db)
  debug.strLn(db,String("omega(10 mili rad/s)"))
  debug.str(db,String("X: "))
  debug.decLn(db,t3_omega_d[0])
  debug.str(db,String("Y: "))
  debug.decLn(db,t3_omega_d[1])
  debug.str(db,String("Z: "))
  debug.decLn(db,t3_omega_d[2])
  debug.newline(db)




PRI printFirstEulerInput

  debug.newline(db)
  debug.strLn(db,String("First Euler Angles In"))

  debug.str(db,String("pitch = "))
  debug.dec(db,t3_first_euler_in_d[0])
  debug.strLn(db,String("  centi degree"))


  debug.str(db,String("roll = "))
  debug.dec(db,t3_first_euler_in_d[1])
  debug.strLn(db,String("  centi degree"))

  debug.str(db,string("yaw = "))
  debug.dec(db,t3_first_euler_in_d[2])
  debug.strLn(db,String("  centi degree"))
  debug.newline(db)
  

PRI printFisrtDCM | iter, digit, counter

  debug.str(db,String("First DCM (rad*10000) : "))
  debug.newline(db)

  repeat iter from 0 to 8
    digit := getDigit(t3_first_dcm_d[iter])
    counter := 0
    debug.dec(db,t3_first_dcm_d[iter])
    repeat counter from 0 to (12-digit)
      debug.str(db,String(" "))
    if ((iter+1)//3 == 0)
      debug.newline(db)
    else
      debug.str(db,string(" "))

PRI printFirstEulerOutput

  debug.newline(db)
  debug.strLn(db,String("First Euler Angles Out"))

  debug.str(db,String("pitch = "))
  debug.dec(db,t3_first_euler_out_d[0])
  debug.strLn(db,String("  centi degree"))


  debug.str(db,String("roll = "))
  debug.dec(db,t3_first_euler_out_d[1])
  debug.strLn(db,String("  centi degree"))

  debug.str(db,string("yaw = "))
  debug.dec(db,t3_first_euler_out_d[2])
  debug.strLn(db,String("  centi degree"))



PRI getDigit(input)| ans, flag

  ans := 0

  if input < 0
    input := -input
    flag := 1

  if (input <10)
    ans := 1
  elseif (input <100)
    ans := 2
  elseif (input <1000)
    ans := 3
  elseif (input < 10000)
    ans := 4
  elseif (input < 100000)
    ans := 5
  elseif(input <1000000)
    ans:= 6
  elseif(input <10000000)
    ans:= 7
  elseif(input <100000000)
    ans:= 8
  elseif(input <1000000000)
    ans:= 9
  else
    ans :=10

  if flag ==1
    ans += 1

  return ans

PRI printDCM | iter, digit, counter

  debug.newline(db) 
  debug.str(db,String("DCM (rad*10000) : "))
  debug.newline(db)

  repeat iter from 0 to 8
    digit := getDigit(t3_dcm_d[iter])
    counter := 0
    debug.dec(db,t3_dcm_d[iter])
    repeat counter from 0 to (12-digit)
      debug.str(db,String(" "))
    if ((iter+1)//3 == 0)
      debug.newline(db)
    else
      debug.str(db,string(" "))
  debug.newline(db)


PRI printEuler | i

  repeat i from 0 to 2
    debug.str(db, String("[c"))
    case i
      0: debug.str(db, String("x"))
      1: debug.str(db, String("y"))
      2: debug.str(db, String("z"))
    debug.dec(db, t3_euler_d[i])
    debug.str(db, String("]"))
    {
debug.newline(db)
  
  debug.strLn(db,String("Calcualted Euler Angles"))

  debug.str(db,String("pitch = "))
  debug.dec(db,t3_euler_d[0]/100)
  debug.str(db,String("."))
  debug.dec(db,||(t3_euler_d[0])//100)
  debug.strLn(db,String(" degree"))


  debug.str(db,String("roll = "))
  debug.dec(db,t3_euler_d[1]/100)
  debug.str(db,String("."))
  debug.dec(db,||(t3_euler_d[1])//100)
  debug.strLn(db,String(" degree"))

  debug.str(db,string("yaw = "))
  debug.dec(db,t3_euler_d[2]/100)
  debug.str(db,String("."))
  debug.dec(db,||(t3_euler_d[2])//100)
  debug.strLn(db,String(" degree"))
  debug.newline(db) 

             }

  

PRI printMatrixMonitor | iter, digit, counter

  debug.strln(db,String("Monitor1 :"))
  debug.str(db,String("acc_body        mag_body"))
  debug.newline(db)
  
  repeat iter from 0 to 8
    digit := getDigit(t3_matrix_monitor1_d[iter])
    counter := 0
    debug.dec(db,t3_matrix_monitor1_d[iter])
    repeat counter from 0 to (12-digit)
      debug.str(db,String(" "))
    if ((iter+1)//3 == 0)
      debug.newline(db)
    else
      debug.str(db,string(" "))

  debug.newline(db)
  debug.strln(db,String("Monitor2 :"))
  debug.str(db,String("Acc_earth      Mag_Earth      err_integral "))
  debug.newline(db)

  repeat iter from 0 to 8
    digit := getDigit(t3_matrix_monitor2_d[iter])
    counter := 0
    debug.dec(db,t3_matrix_monitor2_d[iter])
    repeat counter from 0 to (12-digit)
      debug.str(db,String(" "))
    if ((iter+1)//3 == 0)
      debug.newline(db)
    else
      debug.str(db,string(" "))

  debug.newline(db)
  debug.strln(db,String("Monitor3:"))
  debug.str(db,String("Norm(acc, mag),    err_earth,    err_body "))
  debug.newline(db)

  repeat iter from 0 to 8
    digit := getDigit(t3_matrix_monitor3_d[iter])
    counter := 0
    debug.dec(db,t3_matrix_monitor3_d[iter])
    repeat counter from 0 to (12-digit)
      debug.str(db,String(" "))
    if ((iter+1)//3 == 0)
      debug.newline(db)
    else
      debug.str(db,string(" "))



 {

PRI printAnomalyMonitor

  debug.strLn(String("Anomaly Monitor Running"))
  debug.str(String("t3_an_skew_omega: "))
  if (t3_an_skew_omega > 0)
    debug.dec(t3_an_skew_omega)
    debug.str(String(" Skew Omega Error Detected."))
  else
    debug.str(String(" Skew Omega is working"))
  debug.newline

PRI printAvgAcc | counter
  debug.strLn(String("First average Accel"))
  debug.str(String("accX = "))
  debug.decln(t3_avg_acc_d[0])
  'debug.strln(String(" (10000^-1 m/s)"))

  debug.str(String("accY = "))
  debug.decln(t3_avg_acc_d[1])
  'debug.strln(String(" (10000^-1 m/s)"))

  debug.str(String("accZ = "))
  debug.decln(t3_avg_acc_d[2])
  'debug.strln(String(" (10000^-1 m/s)"))


PRI printFirstMag

  debug.strLn(String("First Magnetometer "))
  debug.str(String("magX = "))
  debug.decln(t3_first_mag_d[0])

  debug.str(String("magY = "))
  debug.decln(t3_first_mag_d[1])

  debug.str(String("magZ = "))
  debug.decln(t3_first_mag_d[2])


PUB skewOmegaAnomalyChecker
  t3_an_skew_omega := 0
  if (t3_imdt[0] <> 0)
    t3_an_skew_omega := 1
  if (t3_imdt[1] <> -t3_omega[2])
    t3_an_skew_omega := 1
  if (t3_imdt[2] <> t3_omega[1])
    t3_an_skew_omega := 1
  if (t3_imdt[3] <> t3_omega[2])
    t3_an_skew_omega := 1
  if (t3_imdt[4] <> 0)
    t3_an_skew_omega := 1
  if (t3_imdt[5] <> -t3_omega[0])
    t3_an_skew_omega := 1
  if (t3_imdt[6] <> -t3_omega[1])
    t3_an_skew_omega := 1
  if (t3_imdt[7] <> t3_omega[0])
    t3_an_skew_omega := 1

PUB omegaDtAnomalyChecker



}