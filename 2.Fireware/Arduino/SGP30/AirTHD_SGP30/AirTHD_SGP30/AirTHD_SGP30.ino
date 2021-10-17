#include "Wire.h"
#include "Adafruit_SGP30.h"

#define SDA 32
#define SCL 33

Adafruit_SGP30 sgp;

float TVOC;
float eCO2;
float Raw_H2;
float Raw_Ethanol;

void setup() {
  // put your setup code here, to run once:
  Wire.begin(SDA, SCL);
  Serial.begin(115200);
  
  Serial.println("SGP30 test");
    if (! sgp.begin(&Wire)){
    Serial.println("Sensor not found :(");
    while (1);
  }
  Serial.print("Found SGP30 serial #");
  Serial.print(sgp.serialnumber[0], HEX);
  Serial.print(sgp.serialnumber[1], HEX);
  Serial.println(sgp.serialnumber[2], HEX);
  

}

void loop() {
  // put your main code here, to run repeatedly:
  if (! sgp.IAQmeasure()) {
    Serial.println("Measurement failed");
    return;
  }
  if (! sgp.IAQmeasureRaw()) {
    Serial.println("Raw Measurement failed");
    return;
  }

  Raw_H2 = sgp.rawH2;
  Raw_Ethanol = sgp.rawEthanol;

  TVOC = sgp.TVOC;    // [ppb]
  eCO2 = sgp.eCO2;    // [ppm]

  Serial.print("TVOC:");
  Serial.print(TVOC);
  Serial.println("ppb");
   
  Serial.print("eCO2:");
  Serial.print(eCO2);
  Serial.println("ppm");
  
  Serial.print("Raw_H2:");
  Serial.println(Raw_H2);

  Serial.print("Raw_Ethanol:");
  Serial.println(Raw_Ethanol);

  delay(500);
  
}
