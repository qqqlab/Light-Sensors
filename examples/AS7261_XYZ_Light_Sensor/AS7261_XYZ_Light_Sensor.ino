#include <qqqAS7261.h>
#include <Wire.h>
#include <math.h>

AS7261 as7261;

#include <M5Core2.h>

//I2C SETUP grove Core2
#define SDA_PIN 32
#define SCL_PIN 33

void setup() {
  Serial.begin(115200);
  Serial.flush();
  delay(50);
  
  Serial.println("XYZ Meter");
  Serial.println("Board: " ARDUINO_BOARD);

  M5.begin();
  M5.Lcd.setRotation(1);  //1=[ ]A] 3=[A[ ]
  M5.Axp.ScreenBreath(12);  
  M5.Lcd.fillScreen(TFT_BLACK);  
  M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setFreeFont(&FreeSerifItalic24pt7b);
  M5.Lcd.drawCentreString("XYZ Meter", 120, 0, 1);

  M5.Axp.SetLcdVoltage(3300); // v from 2500 to 3300
  
  //I2C
  Wire.begin (SDA_PIN, SCL_PIN);  
  delay(100); //this delay is REQUIRED ....
  Wire.begin (SDA_PIN, SCL_PIN);   

  as7261.begin();
}

float rgb2srgb(float C) {
  if (fabs(C) < 0.0031308) {
    return 12.92 * C;
  }
  return 1.055 * pow(C, 0.41666) - 0.055;
}

void loop() {
  as7261.autoGain(); 
  
  Serial.printf("$MEAS");
  
  //CIE XYZ
  float X = as7261.getCalibratedX();
  float Y = as7261.getCalibratedY();
  float Z = as7261.getCalibratedZ();
  Serial.printf(",X,%f,Y,%f,Z,%f", X, Y, Z);
  
  float x = X / (X + Y + Z);
  float y = Y / (X + Y + Z);
  Serial.printf(",x,%f,y,%f", x, y);

  //CIE 1976 L*, u*, v* CIELUV color space https://en.wikipedia.org/wiki/CIELUV
  float up = 4*X / (X + 15*Y + 3*Z);
  float vp = 9*Y / (X + 15*Y + 3*Z);
  //Serial.printf(",up,%f,vp,%f", up, vp);
  
  //CIE 1960 color space https://en.wikipedia.org/wiki/CIE_1960_color_space
  float u = 4*X / (X + 15*Y + 3*Z); //=up
  float v = 6*Y / (X + 15*Y + 3*Z); //=uv*2/3
  //Serial.printf(",u,%f,v,%f", u, v);

  //RGB https://en.wikipedia.org/wiki/CIE_1931_color_space
  float R =  0.41847*X    - 0.15866*Y   - 0.082835*Z;
  float G = -0.091169*X   + 0.25243*Y   + 0.015708*Z;
  float B =  0.00092090*X - 0.0025498*Y + 0.17860*Z;  
  //Serial.printf(",R%f,G,%f,B,%f", R, G, B);

  //sRGB
  float sR = rgb2srgb(R);
  float sG = rgb2srgb(G);
  float sB = rgb2srgb(B);
  //Serial.printf(",sR%f,sG,%f,sB,%f", sR, sG, sB);

  //other registers (all calculated from X,Y,Z)
  //for(uint8_t r=0x30;r<=0x40;r+=4) Serial.printf(",reg%02X,%f", r, getCalibratedValue(r));

  //integration time and gain
  Serial.printf(",intTime,%d,gain,%f", as7261.getIntegrationTime(), as7261.getGainValue());

  //RAW AS7261 readings
  Serial.printf(",Xr,%d,Yr,%d,Zr,%d,NIR,%d,D,%d,C,%d", as7261.getX_CIE(),as7261.getY_CIE(),as7261.getZ_CIE(),as7261.getNIR(),as7261.getDark(),as7261.getClear());

  Serial.println();

  delay(2000);
}
