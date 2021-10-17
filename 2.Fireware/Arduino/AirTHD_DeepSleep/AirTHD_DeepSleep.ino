/*
Simple Deep Sleep with Timer Wake Up
=====================================
ESP32 offers a deep sleep mode for effective power
saving as power is an important factor for IoT
applications. In this mode CPUs, most of the RAM,
and all the digital peripherals which are clocked
from APB_CLK are powered off. The only parts of
the chip which can still be powered on are:
RTC controller, RTC peripherals ,and RTC memories

This code displays the most basic deep sleep with
a timer to wake it up and how to store data in
RTC memory to use it over reboots

This code is under Public Domain License.

Author:
Pranav Cherukupalli <cherukupallip@gmail.com>
*/

#include "Wire.h"
#include <Adafruit_SSD1306.h>
#include "Adafruit_SGP30.h"

#define SDA 32
#define SCL 33
#define SHT30_Addr 0x44
#define OLED_RESET 4


float TVOC;
float eCO2;
float Raw_H2;
float Raw_Ethanol;

float Humidity;
float cTemp;
float fTemp;

Adafruit_SGP30 sgp;
Adafruit_SSD1306 oled(128, 64, &Wire, OLED_RESET);

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  60        /* Time ESP32 will go to sleep (in seconds) */

RTC_DATA_ATTR int bootCount = 0;

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

void setup(){
  Serial.begin(115200);  
  Wire.begin(SDA, SCL);
    Serial.println("SGP30 test");
    if (! sgp.begin(&Wire)){
    Serial.println("Sensor not found :(");
    while (1);
  }
  Serial.print("Found SGP30 serial #");
  Serial.print(sgp.serialnumber[0], HEX);
  Serial.print(sgp.serialnumber[1], HEX);
  Serial.println(sgp.serialnumber[2], HEX);
  delay(1000); //Take some time to open up the Serial Monitor

  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  //Print the wakeup reason for ESP32
  print_wakeup_reason();

  /*
  First we configure the wake up source
  We set our ESP32 to wake up every 5 seconds
  */
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
  " Seconds");

  updateSHT30();
  updateSGP30();
  updateOLED();
  Serial.println("Going to sleep now");
  delay(1000);
  Serial.flush(); 
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
  


  /*
  Next we decide what all peripherals to shut down/keep on
  By default, ESP32 will automatically power down the peripherals
  not needed by the wakeup source, but if you want to be a poweruser
  this is for you. Read in detail at the API docs
  http://esp-idf.readthedocs.io/en/latest/api-reference/system/deep_sleep.html
  Left the line commented as an example of how to configure peripherals.
  The line below turns off all RTC peripherals in deep sleep.
  */
  //esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  //Serial.println("Configured all RTC Peripherals to be powered down in sleep");

  /*
  Now that we have setup a wake cause and if needed setup the
  peripherals state in deep sleep, we can now start going to
  deep sleep.
  In the case that no wake up sources were provided but deep
  sleep was started, it will sleep forever unless hardware
  reset occurs.
  */

}


void updateSHT30(){
    unsigned int data[6]; //存储获取到的六个数据
  //开始IIC
  //写地址
  Wire.beginTransmission(SHT30_Addr);
  //发送测量命令 0x2C06,但是因为IIC一次只能发一个8位数据，所以得发两次
  Wire.write(0x2C);
  Wire.write(0x06);
  //停止IIC
  Wire.endTransmission();
  //等待500ms是等待SHT30器件测量数据，实际上这个时间可以很短
  //delay(5000);

  //请求获取6字节的数据，然后会存到8266的内存里
  Wire.requestFrom(SHT30_Addr, 6);

  //读取6字节的数据
  //这六个字节分别为：温度8位高数据，温度8位低数据，温度8位CRC校验数据
  //               湿度8位高数据，湿度8位低数据，湿度8位CRC校验数据
  if (Wire.available() == 6)
  {
    data[0] = Wire.read();
    data[1] = Wire.read();
    data[2] = Wire.read();
    data[3] = Wire.read();
    data[4] = Wire.read();
    data[5] = Wire.read();
  }

  //然后计算得到的数据，要转化为摄氏度、华氏度、相对湿度
  cTemp = ((((data[0] * 256.0) + data[1]) * 175) / 65535.0) - 45;
  fTemp = (cTemp * 1.8) + 32;
  Humidity = ((((data[3] * 256.0) + data[4]) * 100) / 65535.0);

  //在串口里输出得到的数据
  /*Serial.print("相对湿度：");
  Serial.print(Humidity);
  Serial.println(" %RH");
  Serial.print("摄氏度温度：");
  Serial.print(cTemp);
  Serial.println(" C");
  Serial.print("华氏度温度：");
  Serial.print(fTemp);
  Serial.println(" F");*/
}

uint32_t getAbsoluteHumidity(float temperature, float humidity) {
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
    return absoluteHumidityScaled;
}

void updateSGP30(){
  sgp.setHumidity(getAbsoluteHumidity(cTemp, Humidity));
  // put your main code here, to run repeatedly:
  if (! sgp.IAQmeasure()) {
    //Serial.println("Measurement failed");
    return;
  }
  if (! sgp.IAQmeasureRaw()) {
    //Serial.println("Raw Measurement failed");
    return;
  }

  Raw_H2 = sgp.rawH2;
  Raw_Ethanol = sgp.rawEthanol;

  TVOC = sgp.TVOC;    // [ppb]
  eCO2 = sgp.eCO2;    // [ppm]
}

void updateOLED(){
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.setTextColor(WHITE);//开像素点发光
  oled.clearDisplay();//清屏
  oled.setTextSize(1); //设置字体大小

  
  // put your main code here, to run repeatedly:
  oled.clearDisplay();//清屏

  oled.setCursor(0, 0);//设置显示位置
  oled.println("T:");
  oled.setCursor(15, 0);//设置显示位置
  oled.println(cTemp);
  oled.setCursor(45, 0);//设置显示位置
  oled.println("C");

  oled.setCursor(70, 0);//设置显示位置
  oled.println("H:");
  oled.setCursor(85, 0);//设置显示位置
  oled.println(Humidity);
  oled.setCursor(115, 0);//设置显示位置
  oled.println("%");

  oled.setTextSize(1.5);
  oled.setCursor(0, 16);//设置显示位置
  oled.println("TVOC:");
  oled.setCursor(40, 16);//设置显示位置
  oled.println(TVOC);
  oled.setCursor(100, 16);//设置显示位置
  oled.println("ppb");

  oled.setCursor(0, 32);//设置显示位置
  oled.println("eCO2:");
  oled.setCursor(40, 32);//设置显示位置
  oled.println(eCO2);
  oled.setCursor(100, 32);//设置显示位置
  oled.println("ppm");

  oled.setCursor(0, 43);//设置显示位置
  oled.println("Raw_H2:");
  oled.setCursor(45, 43);//设置显示位置
  oled.println(Raw_H2);
  
  oled.setCursor(0, 53);//设置显示位置
  oled.println("Raw_Ethanol:");
  oled.setCursor(75, 53);//设置显示位置
  oled.println(Raw_Ethanol);

  oled.display(); // 开显示
}

void loop(){
  //This is not going to be called
  updateSHT30();
  updateSGP30();
  updateOLED();
}
