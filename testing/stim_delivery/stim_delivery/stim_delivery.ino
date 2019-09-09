#include <math.h>

void setup()
{
  Serial.begin(9600);
}
 
void loop()
{
  for(int i=0;i<100;i+=10){
    int j = toDac(1);
    dacWrite(25,j);
    Serial.println(j);
    delay(2000);            
  } 
}

// convert a percentage of max current to a DAC value
int toDac(int percentage)
{
  float minVoltage = 0.8; // threshold voltage of BJT
  float maxVoltage = 3.1; // max produceable voltage
  int dacMaxVal = 255;

  float outputVoltage = ((percentage * 0.01) * (maxVoltage - minVoltage)) + minVoltage; // calculate the output voltage we want
  int dacVal = round(outputVoltage * (255 / maxVoltage)); // convert required voltage to a DAC value
  
  return dacVal;
}
