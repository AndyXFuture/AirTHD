#include <Wire.h>
#include <Adafruit_SSD1306.h>

#define SDA 32
#define SCL 33
#define OLED_RESET 4

int i;

Adafruit_SSD1306 oled(128, 64, &Wire, OLED_RESET);

void setup() {
  // put your setup code here, to run once:

  Wire.begin(SDA,SCL);

}

void loop() {


  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.setTextColor(WHITE);//开像素点发光
  oled.clearDisplay();//清屏
  oled.setTextSize(1); //设置字体大小

  
  // put your main code here, to run repeatedly:
  oled.clearDisplay();//清屏

  oled.setCursor(0, 0);//设置显示位置
  oled.println(i);
  i++;
  oled.display(); // 开显示
}
