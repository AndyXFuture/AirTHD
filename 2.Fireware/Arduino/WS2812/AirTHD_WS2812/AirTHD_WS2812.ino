#include <FastLED.h>
#define LED_PIN     27
#define NUM_LEDS    1
#define TIME        10
CRGB leds[NUM_LEDS];

int R = 255;
int G = 0;
int B = 0;
int max_bright = 32;       // LED亮度控制变量，可使用数值为 0 ～ 255， 数值越大则光带亮度越高
bool beath_state = 0;
int light = max_bright;

void setup() {
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(max_bright);
}


void loop() {
    //变色
    if(R==255 && B==0){
      G++;
    }
    if(G==255 && B==0){
      R--;
    }
    if(R==0 && G==255){
      B++;
    }
    if(R==0 && B==255){
      G--;
    }
    if(G==0 && B==255){
      R++;
    }
    if(R==255 && G==0){
      B--;
    }
    
    //显示
    for(int i=0;i<NUM_LEDS;i++){
      leds[i] = CRGB(R, G, B);
    }
    FastLED.show();
    delay(TIME);

  }
