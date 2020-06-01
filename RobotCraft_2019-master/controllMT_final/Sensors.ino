// Read sensors and update sensor values
float readSensors(float* left, float* mid, float* right){
  int i;
  float sumr=0, suml=0, summ=0;
  float left_[10];
  float right_[10];
  float mid_[10];

  for (i=0; i<10; i++){
    float valR = analogRead(sensorpinR);
    right_[i] = 57.33*(pow(valR, -0.9998)) - 0.0282;
    sumr = sumr + right_[i];
 
    
    float valM = analogRead(sensorpinM);
    mid_[i] = 265.5*(pow(valM, -1.305)) + 0.01485;
    summ = summ + mid_[i];
    
    float valL = analogRead(sensorpinL);
    left_[i] = 198.9*(pow(valL, -1.265)) + 0.01932;
    suml = suml + left_[i];
  }
  float left_sens = suml/10;
  float mid_sens = summ/10;
  float right_sens = sumr/10;
  if((suml/10) > 0.8){
    left_sens = 0.8;
  }
  if((sumr/10) > 0.8){
    right_sens = 0.8;
  }
  if((summ/10) > 0.8){
    mid_sens = 0.8;
  }
  *left = left_sens;
  *mid = mid_sens;
  *right = right_sens;
}
