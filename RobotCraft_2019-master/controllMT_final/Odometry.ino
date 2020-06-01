// Read encoders and update encoder values
void encUpdate(){
    // Record readings for M1
    long newPosition_R = myEnc_1.read();
    NR=newPosition_R - oldPosition_R;
    oldPosition_R = newPosition_R;
    
    // Record readings for M2
    long newPosition_L = myEnc_2.read();
    NL=newPosition_L - oldPosition_L;
    oldPosition_L = newPosition_L;
}

// Update postion of the robot based on encoder readings
void poseUpdate(){
 //Computing real velocities
  Vr = (2.0*M_PI*r*(NR+NL))/(C*2.0*dt); // [m/s]
  Wr = (2.0*M_PI*r*(NR-NL))/(C*b*dt);   // [rad/s]
  
  current_pos.theta = atan2(sin(current_pos.theta + Wr*dt),cos(current_pos.theta+Wr*dt)) ;
  current_pos.x = current_pos.x + Vr*dt*cos(current_pos.theta);
  current_pos.y = current_pos.y + Vr*dt*sin(current_pos.theta);
}

// Calculate angular velocities of the wheels in [rad/s] given linear and angular velocities of the robot V [m/s], W [rad/s]
void cmd_vel2wheel(float V,float W,float *WL, float *WR){
  *WL = (V - b*W/2)/r;
  *WR = (V + b*W/2)/r;
}
