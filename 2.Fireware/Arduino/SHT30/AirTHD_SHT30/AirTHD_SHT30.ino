#include "Wire.h"

#define SDA 32
#define SCL 33
#define Addr 0x44

float TEM_Val = 0;
float HUM_Val = 0;

float Humidity;
float Temperature;

void setup() {
  // put your setup code here, to run once:
  Wire.begin(SDA, SCL);
  Serial.begin(115200);
}
void loop()
{
  unsigned int data[6]; //存储获取到的六个数据
  //开始IIC
  //写地址
  Wire.beginTransmission(Addr);
  //发送测量命令 0x2C06,但是因为IIC一次只能发一个8位数据，所以得发两次
  Wire.write(0x2C);
  Wire.write(0x06);
  //停止IIC
  Wire.endTransmission();
  //等待500ms是等待SHT30器件测量数据，实际上这个时间可以很短
  delay(5000);

  //请求获取6字节的数据，然后会存到8266的内存里
  Wire.requestFrom(Addr, 6);

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
  float cTemp = ((((data[0] * 256.0) + data[1]) * 175) / 65535.0) - 45;
  float fTemp = (cTemp * 1.8) + 32;
  float humidity = ((((data[3] * 256.0) + data[4]) * 100) / 65535.0);

  //在串口里输出得到的数据
  Serial.print("相对湿度：");
  Serial.print(humidity);
  Serial.println(" %RH");
  Serial.print("摄氏度温度：");
  Serial.print(cTemp);
  Serial.println(" C");
  Serial.print("华氏度温度：");
  Serial.print(fTemp);
  Serial.println(" F");
  delay(500);
}
