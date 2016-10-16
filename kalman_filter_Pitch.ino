// KasBot V2  -  Kalman filter module - http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1284738418 - http://www.y-firm.com/?page_id=145
//with slightly modifications by Kristian Lauszus

    float Q_angleY  =  0.001; 
    float Q_gyroY   =  0.003;  
    float R_angleY  =  0.03;  

    float y_angle = 0;
    float y_bias = 0;
    float PY_00 = 0, PY_01 = 0, PY_10 = 0, PY_11 = 0;	
    float dtY, yY, SY;
    float KY_0, KY_1;

  float kalmanCalculatePitch(float newAngle, float newRate,int looptime) {
    dtY = float(looptime)/1000;                                    // YYYYYYYYY arevoir
    y_angle += dtY * (newRate - y_bias);
    PY_00 +=  - dtY * (PY_10 + PY_01) + Q_angleY * dtY;
    PY_01 +=  - dtY * PY_11;
    PY_10 +=  - dtY * PY_11;
    PY_11 +=  + Q_gyroY * dtY;
    
    yY = newAngle - y_angle;
    SY = PY_00 + R_angleY;
    KY_0 = PY_00 / SY;
    KY_1 = PY_10 / SY;
    
    y_angle +=  KY_0 * yY;
    y_bias  +=  KY_1 * yY;
    PY_00 -= KY_0 * PY_00;
    PY_01 -= KY_0 * PY_01;
    PY_10 -= KY_1 * PY_00;
    PY_11 -= KY_1 * PY_01;
    
    return y_angle;
  }

 
