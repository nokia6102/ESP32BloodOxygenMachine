/*
 * PPG 版即實顯示波型版:有偵測滴滴聲音/血氧<90一直BB聲
 * 
 *參考:https://engineerworkshop.com/blog/graph-your-pulse-with-an-arduino-pulse-ox-plethysmograph/
 */
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "ESP32Servo.h"
#define ONBOARD_LED  2
//---------------------------------------------------------------------
#include <WiFi.h>
#include <HTTPClient.h>
//const char * ssid = "ENTER_YOUR_WIFI_SSID";
const char * ssid = "Xiaomi_home";
//const char * password = "ENTER_YOUR_WIFI_PASSWORD";
const char * password = "0000000";
//String GOOGLE_SCRIPT_ID = "ENTER_GOOGLE_DEPLOYMENT_ID";
String GOOGLE_SCRIPT_ID = "00AKfycbxJ4WatmZwZljDp9iWTnpIfFkNtmOh7e1oMs9UEejNHl4o4Bi9Z2PEnnfitFTLDA980000";
//---------------------------------------------------------------------
TaskHandle_t Task2;   //宣告任務變數Task2
bool SendFlag = false;

MAX30105 particleSensor;
int Tonepin = 4;
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
byte x;
byte y;
byte z;
byte lastx;
byte lasty;
long baseValue = 0;
long lastMin=2200000;
long lastMax=0;
long rollingMin = 2200000;
long rollingMax=0;

//beatRate
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;

//計算血氧用變數
double avered = 0;
double aveir = 0;
double sumirrms = 0;
double sumredrms = 0;
double SpO2 = 0;
double ESpO2 = 90.0;//初始值
double FSpO2 = 0.7; //filter factor for estimated SpO2
double frate = 0.95; //low pass filter for IR/red LED value to eliminate AC component
int i = 0;
int Num = 30;//取樣100次才計算1次
#define FINGER_ON 50000 //紅外線最小量（判斷手指有沒有上）
#define MINIMUM_SPO2 80.0//血氧最小量
//Oled
int xO2=0;
int lastxO2=0;
int lastyO2=0;


//OLED設定
#define SCREEN_WIDTH 128 //OLED寬度
#define SCREEN_HEIGHT 64 //OLED高度
#define OLED_RESET    -1 //Reset pin
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); //Declaring the display name (display)

uint32_t ir, red ;
long irValue;



//任務1副程式Task1_senddata
void Task2code(void * pvParameters ) {
  //--------------------------------------------
//  WiFi.mode(WIFI_STA);
//  WiFi.begin(ssid, password);
//
//  Serial.print("Connecting to Wi-Fi");
//  while (WiFi.status() != WL_CONNECTED) {
//    vTaskDelay(500);
//    Serial.print(".");
//  }
//  Serial.println("OK");
 //--------------------------------------------
//無窮迴圈
for (;;) {
  //偵測上傳旗標是否為true
   if (SendFlag) {
      Serial.print("Task1：啟動網頁連線，at core:");
      Serial.println(xPortGetCoreID());
      HTTPClient http;
      //將溫度及濕度以http get參數方式補入網址後方
      String param;
      param  = "bpm="+String(beatAvg);
      param += "&spo="+String(ESpO2);    
      param += "&ir="+String(irValue);
      
      String url="https://script.google.com/macros/s/"+GOOGLE_SCRIPT_ID+"/exec?"+param;
      Serial.print(url);
      Serial.println("Postring GPS data to Google Sheet");
      //---------------------------------------------------------------------
      //starts posting data to google sheet
      tone(Tonepin, 1000);//發出聲音
      delay(20);
      noTone(Tonepin);//停止聲音
      http.begin(url.c_str());
      http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
      int httpCode = http.GET();  
      Serial.print("HTTP Status Code: ");
      Serial.println(httpCode);
      //---------------------------------------------------------------------
      //getting response from google sheet
      String payload;
      if (httpCode == HTTP_CODE_OK) {
        payload = http.getString();
         //讀取網頁內容到payload
        Serial.print("網頁內容=");
        Serial.println("Payload: "+payload); 
      }else{
         //傳送失敗
        Serial.println("網路傳送失敗");
      }
      //---------------------------------------------------------------------
      //修改完畢，修改傳送旗標=false
      SendFlag = false;
      http.end();
    } else {
      //Task1休息，delay(1)不可省略
      delay(1);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");
  pinMode(ONBOARD_LED,OUTPUT);

  //建立Task2任務並指定在核心0中執行
  xTaskCreatePinnedToCore(
                    Task2code,   /* 任務函數 */
                    "Task2",     /* 任務名稱 */
                    10000,       /* 任務推疊大小 */
                    NULL,        /* 任務參數 */
                    1,           /* 任務優先權(0是最低優先權 */
                    NULL,      /* 欲追蹤處理的任務名稱 (可使用 &Task2 或 NULL) */
                    0);          /* 指定此任務的執行核心(0或1) */ 

 
  
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");
  
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

  //Take an average of IR readings at power up; this allows us to center the plot on start up
  const byte avgAmount = 30;
  long reading;
  for (byte x = 0 ; x < avgAmount ; x++){
    reading = particleSensor.getIR();

    // Find max IR reading in sample
    if (reading > lastMax){
      lastMax = reading;
    }

    // Find min IR reading in sample
    if (reading < lastMin){
      lastMin = reading;
    }
  }
  
  x = 0;
  y = 0;
  lastx = 0;
  lasty = 0;
  delay(2000);
  oled.clearDisplay();
  //--------------------------------------------
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("OK");
  //--------------------------------------------
}



void loop() {

//red


//beatRate
irValue = particleSensor.getIR();
if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();
    beatsPerMinute = 60 / (delta / 1000.0);
    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", Red=");
  Serial.print(red);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.println(beatAvg);
//  print_BPM();       //print to google Sheet
   
  if (irValue < FINGER_ON)
  {
    Serial.println(" No finger?");
    //清除心跳數據
    for (byte rx = 0 ; rx < RATE_SIZE ; rx++) rates[rx] = 0;
    beatAvg = 0; rateSpot = 0; lastBeat = 0;
    //清除血氧數據
    avered = 0; aveir = 0; sumirrms = 0; sumredrms = 0;
    SpO2 = 0; ESpO2 = 90.0;
    noTone(Tonepin);//停止聲音
  }
  
//  Serial.println();
//--beatRate
  // Display is only 128 pixels wide, so if we're add the end of the display, clear the display and start back over
  if(x>127)  
  {
    oled.clearDisplay();
    x=0;
    lastx=x;
    lastxO2=xO2;
  }

  // Even though we're keeping track of min/max on a rolling basis, periodically reset the min/max so we don't end up with a loss of waveform amplitude
  if (z > 30) {
    z = 0;
    lastMax = rollingMax;
    lastMin = rollingMin;
    rollingMin = 2200000;
    rollingMax = 0;
  }
 
  oled.setTextColor(WHITE);
  long reading = particleSensor.getIR();    // Read pulse ox sensor; since this is a pulse pleth, we're really only after the IR component
  int y=50-(map(reading, lastMin, lastMax, 0, 40));   // Normalize the pleth waveform against the rolling IR min/max to keep waveform centered
  if (irValue < 50000) y=42;
  //心脈跳動實際波形
  oled.drawLine(lastx,lasty,x,y,WHITE);

  //血氧曲線
  int yO2=32-(map(ESpO2, 80, 100, 0, 32)); 
  oled.writeLine(lastxO2,lastyO2,x,yO2,WHITE);
  lastyO2=yO2;
  lastxO2=x;
     
  oled.writeFillRect(0,52,128,14,BLACK);
  oled.setCursor(0,52);
  oled.print("BPM:");
  oled.print(beatAvg);
  
  if (beatAvg>30){
    oled.print(" Oxygen:");
    oled.print(ESpO2);
    oled.print("%"); 
    //修改上傳旗標=true
     SendFlag = true;       
 } 

  // Keep track of min/max IR readings to keep waveform centered
  if (reading > rollingMax){
    rollingMax = reading;
  }

  if (reading < rollingMin){
    rollingMin = reading;
  }
  
  // Keep track of this IR reading so we can draw a line from it on the next reading
  lasty=y;
  lastx=x;
  
  //計算血氧
//    uint32_t ir, red ;
    double fred, fir;
    particleSensor.check(); //Check the sensor, read up to 3 samples
    if (particleSensor.available()) {
      i++;
      red = particleSensor.getFIFOIR(); //讀取紅光
      ir = particleSensor.getFIFORed(); //讀取紅外線
      //Serial.println("red=" + String(red) + ",IR=" + String(ir) + ",i=" + String(i));
      fred = (double)red;//轉double
      fir = (double)ir;//轉double
      avered = avered * frate + (double)red * (1.0 - frate);//average red level by low pass filter
      aveir = aveir * frate + (double)ir * (1.0 - frate); //average IR level by low pass filter
      sumredrms += (fred - avered) * (fred - avered); //square sum of alternate component of red level
      sumirrms += (fir - aveir) * (fir - aveir);//square sum of alternate component of IR level
       if ((i % Num) == 0) {
        double R = (sqrt(sumredrms) / avered) / (sqrt(sumirrms) / aveir);
        SpO2 = -23.3 * (R - 0.4) + 100;
        ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;//low pass filter
        if (ESpO2 <= MINIMUM_SPO2) ESpO2 = MINIMUM_SPO2; //indicator for finger detached
        if (ESpO2 > 100) ESpO2 = 99.9;
        Serial.print(", Oxygen % = "); Serial.print(ESpO2);
        sumredrms = 0.0; sumirrms = 0.0; SpO2 = 0;
        i = 0;
        if (irValue >= FINGER_ON ){
            tone(Tonepin, 1000);//發出聲音
            delay(20);
            noTone(Tonepin);//停止聲音
        }
       }
      particleSensor.nextSample(); //We're finished with this sample so move to next 
    }
 
  oled.display();
  x++;
  z++;
}
