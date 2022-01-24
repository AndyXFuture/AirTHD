# AirTHD

​	**——基于ESP32的微型空气质量检测仪**

## 关于该项目

用于毕业设计，以及用来解决个人检测二氧化碳浓度的需求的一个项目

![](/5.Img/Img_1_1.png)

采用ESP-IDF框架开发（Arduino里的代码仅有获取传感器数据并显示）

主要功能：获取空气质量数据并输出

---

## 项目文件说明
* Hardware：用Altium Designer设计PCB
* Fireware：主要固件在ESP-IDF里，Arduino只做了传感器获取和显示功能
* Software：目前仅有一个手机APP，用于读取蓝牙广播数据并计算相应物理量
* 3DModel：外壳3D打印文件
* Docs：相关资料

---
## 关于硬件方案
已验证，焊接能力不强建议钢网铁板烧

板载SHT30部分电路仅为个人MCU测温使用可以不焊，实际使用需直接买SHT30模块叠在SGP30上方

动手能力强的可以自行添加电池及无线充电，外壳有空间（可改外壳自行适配电池）放电池和无线充电模块

![](/5.Img/Img_3.png)

---

### 功能列表：
* 获取传感器数据：SHT30中的`温湿度`与SGP30中的`TVOC`值及`二氧化碳`浓度

* 连接WIFI获取`B站粉丝数`、`室外温度`和`天气类型`

* 屏幕显示通过LVGL展示上述的数据

* 串口输出（波特率115200）

* 蓝牙BLE广播传感器数据（Major与Minor值）

  ​	如Major = 30625即为温度30.6度，湿度25%

  ​	如Minor = 14512即为eCO2=1450ppm，TVOC=1200ppb

  

  ​	注：eCO2维持在800ppm以下一般不用开窗通风

  

---

### 还有一个简单的安卓APP

![](/5.Img/Img_2.jpg)

## 其他的后续再补充，有用的话记得点星星~
