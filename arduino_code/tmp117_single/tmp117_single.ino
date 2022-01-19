#include <Wire.h> // Used to establish serial communication on the I2C bus
#include <SparkFun_TMP117.h> // Used to send and recieve specific information from the sensor

#define HEAT_ON 6 //in 15 ms
#define HEAT_OFF 600
// The default address of the device is 0x48 (GND)
TMP117 sensor; // Initalize sensor

unsigned long prev_time;
unsigned long l=0;
int heater = 0;
void setup()
{
  Wire.begin();
  Serial.begin(115200);    // Start serial communication at 115200 baud
  Wire.setClock(400000);   // Set clock speed to be the fastest for better communication (fast mode)
  sensor.begin(); // Function to check if the sensor will correctly self-identify with the proper Device ID/Address
  delay(1000);
  sensor.setConversionAverageMode(0);
  sensor.setConversionCycleBit(0);
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(PD2,OUTPUT);
  pinMode(PD3,OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); 
  digitalWrite(PD2, LOW); 
  digitalWrite(PD3, LOW); 
}

void loop()
{
  // Data Ready is a flag for the conversion modes - in continous conversion the dataReady flag should always be high
//  if (sensor.dataReady() == true) // Function to make sure that there is data ready to be printed, only prints temperature values when data is ready
//  {

    if(l==1){
      // Turn heater on
      digitalWrite(LED_BUILTIN, HIGH); 
      digitalWrite(PD3, HIGH);
      heater = 100;
    }
    else if (l==HEAT_ON+1){
      // Turn heater off
      digitalWrite(LED_BUILTIN, LOW); 
      digitalWrite(PD3, LOW);
      heater = 0;
    }
    else if (l>=HEAT_OFF+HEAT_ON+1){
      // Wait
      l = 0;
    }
    l++;
    prev_time = millis();
    float tempC = sensor.readTempC();
    Serial.print(prev_time);
    Serial.print(" ");
    Serial.print(tempC);
    Serial.print(" ");
    Serial.println(heater);
    while(millis()-prev_time < 15);
//  }
}
