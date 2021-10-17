#include "Wire.h"
#include <Adafruit_SSD1306.h>
#include "Adafruit_SGP30.h"

#define SDA 32
#define SCL 33
#define SHT30_Addr 0x44
#define OLED_RESET 4

float TVOC; //用于存放TVOC指标
float eCO2; //用于存放eCO2指标
float Raw_H2; //用于存放H2原始数据
float Raw_Ethanol; //用于存放Ethanol原始数据

float Humidity; //用于存放湿度数据
float cTemp;  //用于存放摄氏温度
float fTemp;  //用于存放华氏温度

//创建SGP30对象
Adafruit_SGP30 sgp;
//创建OLED屏对象
Adafruit_SSD1306 oled(128, 64, &Wire, OLED_RESET);

void setup() {
  // put your setup code here, to run once:
  //设置SDA引脚和SCL引脚
  Wire.begin(SDA, SCL);
  //设置串口波特率
  Serial.begin(115200);

  //检测SGP30是否正常调用
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

//用于更新从SHT30获取的数据
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
  delay(500);

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
  Serial.print("相对湿度：");
  Serial.print(Humidity);
  Serial.println(" %RH");
  Serial.print("摄氏度温度：");
  Serial.print(cTemp);
  Serial.println(" C");
  Serial.print("华氏度温度：");
  Serial.print(fTemp);
  Serial.println(" F");
}

//计算用于给SGP30校准的数据
uint32_t getAbsoluteHumidity(float temperature, float humidity) {
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
    return absoluteHumidityScaled;
}

//用于更新从SGP30获取的数据
void updateSGP30(){
  sgp.setHumidity(getAbsoluteHumidity(cTemp, Humidity));
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
}


//刷新128*64OLED屏幕
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

  /*oled.setCursor(0, 43);//设置显示位置
  oled.println("Raw_H2:");
  oled.setCursor(45, 43);//设置显示位置
  oled.println(Raw_H2);
  
  oled.setCursor(0, 53);//设置显示位置
  oled.println("Raw_Ethanol:");
  oled.setCursor(75, 53);//设置显示位置
  oled.println(Raw_Ethanol);*/

  oled.display(); // 开显示
}

//主函数
void loop()
{
  //获取数据
  updateSHT30();
  updateSGP30();
  //刷新屏幕
  updateOLED();
  //延时
  delay(500);
}
