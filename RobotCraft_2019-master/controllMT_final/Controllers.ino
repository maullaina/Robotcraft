// Change direction of the motor based on input 
void changeDIR(float RightCMD, float LeftCMD){
    if (RightCMD < 0){
      digitalWrite(DIRR,HIGH);
    }
    else{
      digitalWrite(DIRR,LOW);
    }
    if (LeftCMD < 0){
      digitalWrite(DIRL,HIGH);
    }
    else{
      digitalWrite(DIRL,LOW);
    }

  }

/*
// Model based controller - Bogdan [step response 0.2 sec, variables calculated using matlab]
void ModelController(float Vd, float Wd, float w_l_real, float w_r_real){
  
    // Declair variables [r,u,y,r1,u1,y1]
    float refr, refl;
    float inputr, inputl;
    float outputr, outputl;
    static float refr_1, refl_1;
    static float inputr_1, inputl_1;
    static float outputr_1, outputl_1;

    // Init static variables
    if(ctr_Flag == 0){
      refr_1 = 0;
      refl_1 = 0;
      inputr_1 = 0;
      inputl_1 = 0;
      outputr_1 = 0;
      outputl_1 = 0;
      
      ctr_Flag = 1;
    }

    // 1: Calc ref and update current output [r,y]
    refl = (Vd - (b*Wd)/2.0)/r; 
    refr = (Vd + (b*Wd)/2.0)/r;

    // Setting bounds on possible desired wheel velocities +/- 5.0 [rad/s]
    if(refl < -max_angular_vel){
      refl = -max_angular_vel;
    }
    if(refl > max_angular_vel){
      refl = max_angular_vel;
    }
     if(refr < -max_angular_vel){
      refr = -max_angular_vel;
    }
    if(refr > max_angular_vel){
      refr = max_angular_vel;
    }
    outputr = w_r_real;
    outputl = w_l_real;

    // 2: Calc input [u]
    inputr = inputr_1 + a_*refr + b_*refr_1 - c_*outputr - d_*outputr_1;
    inputl = inputl_1 + a_*refl + b_*refl_1 - c_*outputl - d_*outputl_1;

    // 3: Write output
    
    changeDIR(inputr,inputl);

    analogWrite(PWMR,fabs(inputr));
    analogWrite(PWML,fabs(inputl));

    // 4: Update [r1,u1,y1]
    refr_1 = refr;
    refl_1 = refl;
    inputr_1 = inputr;
    inputl_1 = inputl;
    outputr_1 = outputr;
    outputl_1 = outputl;
  }*/

// PID controller - Bogdan & Joao [step response 0.8 sec with Kp = 2, Ki = 250, Kd = 0]
void pid_controller1(float Vd, float Wd, float w_l_real, float w_r_real){

    float inputr, inputl;
    
    // Finding desired wheel velocities
    float w_l_des, w_r_des;
    w_l_des = (Vd - (b*Wd)/2.0)/r;
    w_r_des = (Vd + (b*Wd)/2.0)/r;

    // Setting bounds on possible desired wheel velocities +/- 4.7 [rad/s]
    if(w_l_des < -max_angular_vel){
      w_l_des = -max_angular_vel;
    }
    if(w_l_des > max_angular_vel){
      w_l_des = max_angular_vel;
    }
    if(w_r_des < -max_angular_vel){
      w_r_des = -max_angular_vel;
    }
    if(w_r_des > max_angular_vel){
      w_r_des = max_angular_vel;
    }

    // Calculating the error
    float error_l = w_l_des - w_l_real;
    float error_r = w_r_des - w_r_real;
    
    // Static varibales
    float integral_active_zone = 5;
    static float last_error_r;
    static float last_error_l;
    static float integral_error_r;
    static float integral_error_l;
   
    
    // Calculating proportional part
    float proportional_r = kp*error_r;
    float proportional_l = kp*error_l;
    
    // Calculating integral part
    integral_error_r += error_r*dt;    
    integral_error_l += error_l*dt;
    float integral_r = ki*integral_error_r;
    float integral_l = ki*integral_error_l;
    
    // Calculating derivative part
    float derivative_r = kd*((error_r - last_error_r)/dt);
    float derivative_l = kd*((error_l - last_error_l)/dt);
  
    
    // Calculating total output
    inputr = proportional_r + integral_r + derivative_r;
    inputl = proportional_l + integral_l + derivative_l;
    
    
    //Updating the error
    last_error_l = error_l;
    last_error_r = error_r;

    // Change DIR of the motors if necessary
    changeDIR (inputr,inputl);

    // Write to the motors
    analogWrite(PWMR,fabs(inputr));
    analogWrite(PWML,fabs(inputl));
}
