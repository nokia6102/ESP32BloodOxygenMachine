//心跳圖版:雙曲線 + 跑雙核 + GoogleSheet
//
//跑雙核 https://youyouyou.pixnet.net/blog/post/120275992-%E7%AC%AC%E4%BA%8C%E5%8D%81%E7%AF%87-esp32-%E7%89%B9%E6%AE%8A%E6%87%89%E7%94%A8%EF%BC%9A%E5%A4%9A%E5%9F%B7%E8%A1%8C%E7%B7%92

//離線版版本20210705，https://youtu.be/ghTtpUTSc4o
//安裝4個程式庫：1.Adafruit SSD1306、2.MAX30105、3.ESP32Servo、4.U8g2
//關於MAX30102可以參閱文件：https://datasheets.maximintegrated.com/en/ds/MAX30102.pdf
//https://pdfserv.maximintegrated.com/en/an/AN6409.pdf
//中文字庫參考資料：https://blog.jmaker.com.tw/chinese_oled/
//              https://blog.csdn.net/menghuanbeike/article/details/75666266  
#include <Adafruit_GFX.h>        //OLED libraries
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include "MAX30105.h"           //MAX3010x library
#include "heartRate.h"          //Heart rate calculating algorithm
#include "ESP32Servo.h"
#include <U8g2lib.h>            //中文字庫
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);   //中文字庫用變數

//---------------------------------------------------------------------
#include <WiFi.h>
#include <HTTPClient.h>
//const char * ssid = "ENTER_YOUR_WIFI_SSID";
const char * ssid = "Xiaomi_home";
//const char * password = "ENTER_YOUR_WIFI_PASSWORD";
const char * password = "00000000";
//String GOOGLE_SCRIPT_ID = "ENTER_GOOGLE_DEPLOYMENT_ID";
String GOOGLE_SCRIPT_ID = "00AKfycbwdyTba04kz3-8ZuPCeRjHwgUyTwewkBnu1butj3eAbvRPqEBlV__chJ86De-gpXOxl0000";
//---------------------------------------------------------------------
TaskHandle_t Task1;   //宣告任務變數Task1
TaskHandle_t Task2;   //宣告任務變數Task2
bool SendFlag = false;

// LED pins
const int led1 = 2;
const int led2 = 4;

int x=0;
int lastx=0;
int lasty=0;

int xO2=0;
int lastxO2=0;
int lastyO2=0;


#define CHT 0          
bool cht=CHT;       //預設是中文顯示

//---重新定義FlashButton按鍵
#define FlashButtonPIN 0
volatile int dMode = 1;

int last_dMode = 0;

void IRAM_ATTR handleInterrupt() {
  dMode++; 
}
//---

MAX30105 particleSensor;
int Tonepin = 4;
//計算心跳用變數
const byte RATE_SIZE = 10; //多少平均數量
byte rates[RATE_SIZE]; //心跳陣列
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
static int beatAvg;

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
#define FINGER_ON 7000 //紅外線最小量（判斷手指有沒有上）
#define MINIMUM_SPO2 90.0//血氧最小量

//OLED設定
#define SCREEN_WIDTH 128 //OLED寬度
#define SCREEN_HEIGHT 64 //OLED高度
#define OLED_RESET    -1 //Reset pin
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); //Declaring the display name (display)

//心跳小圖
static const unsigned char PROGMEM logo2_bmp[] =
{ 0x03, 0xC0, 0xF0, 0x06, 0x71, 0x8C, 0x0C, 0x1B, 0x06, 0x18, 0x0E, 0x02, 0x10, 0x0C, 0x03, 0x10,              //Logo2 and Logo3 are two bmp pictures that display on the OLED if called
  0x04, 0x01, 0x10, 0x04, 0x01, 0x10, 0x40, 0x01, 0x10, 0x40, 0x01, 0x10, 0xC0, 0x03, 0x08, 0x88,
  0x02, 0x08, 0xB8, 0x04, 0xFF, 0x37, 0x08, 0x01, 0x30, 0x18, 0x01, 0x90, 0x30, 0x00, 0xC0, 0x60,
  0x00, 0x60, 0xC0, 0x00, 0x31, 0x80, 0x00, 0x1B, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x04, 0x00,
};
//心跳大圖
static const unsigned char PROGMEM logo3_bmp[] =
{ 0x01, 0xF0, 0x0F, 0x80, 0x06, 0x1C, 0x38, 0x60, 0x18, 0x06, 0x60, 0x18, 0x10, 0x01, 0x80, 0x08,
  0x20, 0x01, 0x80, 0x04, 0x40, 0x00, 0x00, 0x02, 0x40, 0x00, 0x00, 0x02, 0xC0, 0x00, 0x08, 0x03,
  0x80, 0x00, 0x08, 0x01, 0x80, 0x00, 0x18, 0x01, 0x80, 0x00, 0x1C, 0x01, 0x80, 0x00, 0x14, 0x00,
  0x80, 0x00, 0x14, 0x00, 0x80, 0x00, 0x14, 0x00, 0x40, 0x10, 0x12, 0x00, 0x40, 0x10, 0x12, 0x00,
  0x7E, 0x1F, 0x23, 0xFE, 0x03, 0x31, 0xA0, 0x04, 0x01, 0xA0, 0xA0, 0x0C, 0x00, 0xA0, 0xA0, 0x08,
  0x00, 0x60, 0xE0, 0x10, 0x00, 0x20, 0x60, 0x20, 0x06, 0x00, 0x40, 0x60, 0x03, 0x00, 0x40, 0xC0,
  0x01, 0x80, 0x01, 0x80, 0x00, 0xC0, 0x03, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x30, 0x0C, 0x00,
  0x00, 0x08, 0x10, 0x00, 0x00, 0x06, 0x60, 0x00, 0x00, 0x03, 0xC0, 0x00, 0x00, 0x01, 0x80, 0x00
};
//氧氣圖示
static const unsigned char PROGMEM O2_bmp[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x3f, 0xc3, 0xf8, 0x00, 0xff, 0xf3, 0xfc,
  0x03, 0xff, 0xff, 0xfe, 0x07, 0xff, 0xff, 0xfe, 0x0f, 0xff, 0xff, 0xfe, 0x0f, 0xff, 0xff, 0x7e,
  0x1f, 0x80, 0xff, 0xfc, 0x1f, 0x00, 0x7f, 0xb8, 0x3e, 0x3e, 0x3f, 0xb0, 0x3e, 0x3f, 0x3f, 0xc0,
  0x3e, 0x3f, 0x1f, 0xc0, 0x3e, 0x3f, 0x1f, 0xc0, 0x3e, 0x3f, 0x1f, 0xc0, 0x3e, 0x3e, 0x2f, 0xc0,
  0x3e, 0x3f, 0x0f, 0x80, 0x1f, 0x1c, 0x2f, 0x80, 0x1f, 0x80, 0xcf, 0x80, 0x1f, 0xe3, 0x9f, 0x00,
  0x0f, 0xff, 0x3f, 0x00, 0x07, 0xfe, 0xfe, 0x00, 0x0b, 0xfe, 0x0c, 0x00, 0x1d, 0xff, 0xf8, 0x00,
  0x1e, 0xff, 0xe0, 0x00, 0x1f, 0xff, 0x00, 0x00, 0x1f, 0xf0, 0x00, 0x00, 0x1f, 0xe0, 0x00, 0x00,
  0x0f, 0xe0, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};



//===========================Google Sheet Functions===============================
void write_to_google_sheet(String params) {
   HTTPClient http;
   String url="https://script.google.com/macros/s/"+GOOGLE_SCRIPT_ID+"/exec?"+params;
   Serial.print(url);
    Serial.println("Postring GPS data to Google Sheet");
    //---------------------------------------------------------------------
    //starts posting data to google sheet
    http.begin(url.c_str());
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    int httpCode = http.GET();  
    Serial.print("HTTP Status Code: ");
    Serial.println(httpCode);
    //---------------------------------------------------------------------
    //getting response from google sheet
    String payload;
    if (httpCode > 0) {
        payload = http.getString();
        Serial.println("Payload: "+payload);     
    }
    //---------------------------------------------------------------------
    http.end();
}

uint32_t ir, red ;
long irValue;

void print_BPM()
{
    String param;
//    param  = "bpm="+String(beatsPerMinute);
//    param += "&avgbpm="+String(beatAvg);
//    param += "&spo2="+String(ESpO2);
//    param += "&ir="+String(irValue);
//    param += "&red="+String(red);

    param  = "bpm="+String(beatAvg);
    param += "&spo="+String(ESpO2);    
    
    write_to_google_sheet(param);
    
    Serial.println(param);
}
//===========================Google Sheet Functions===============================end


//第1個任務LED1每隔  1000 ms閃爍
void Task1code( void * pvParameters ){

  for(;;)
  {
    Serial.print("Task1 running on core "); 
    Serial.println(xPortGetCoreID());     //輸出執行此函式的核心編號
     vTaskDelay(500);
  }
   
}
 
////第2個任務LED2每隔 700 ms閃爍
//void Task2code( void * pvParameters ){
//
//  for(;;){               //任務函數必須無限循環執行，如果離開函式會自動RESET
//Serial.print("Task1 running on core ");
//    Serial.println(xPortGetCoreID());     //輸出執行此函式的核心編號
//
//    vTaskDelay(500);
//    print_BPM();
//    vTaskDelay(500);
//  }
//}


//任務1副程式Task1_senddata
void Task2code(void * pvParameters ) {
  //--------------------------------------------
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(500);
    Serial.print(".");
  }
  Serial.println("OK");
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
      
      String url="https://script.google.com/macros/s/"+GOOGLE_SCRIPT_ID+"/exec?"+param;
      Serial.print(url);
      Serial.println("Postring GPS data to Google Sheet");
      //---------------------------------------------------------------------
      //starts posting data to google sheet
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


//  u8g2.begin();
//  u8g2.enableUTF8Print();  //啟用UTF8文字的功能 

  pinMode(FlashButtonPIN, INPUT_PULLUP);    //設定一個中斷給按鍵
  attachInterrupt(digitalPinToInterrupt(FlashButtonPIN), handleInterrupt, FALLING);
  
  Serial.begin(115200);
  Serial.println("System Start");

//   //建立Task1任務並指定在核心0中執行
//  xTaskCreatePinnedToCore(
//                    Task1code,   /* 任務函數 */
//                    "Task1",     /* 任務名稱 */
//                    10000,       /* 任務推疊大小 */
//                    NULL,        /* 任務參數 */
//                    1,           /* 任務優先權(0是最低優先權 */
//                    NULL,      /* 欲追蹤處理的任務名稱 (可使用 &Task1 或 NULL) */
//                    0);          /* 指定此任務的執行核心(0或1) */                  
 delay(500);
//建立Task2任務並指定在核心1中執行
  xTaskCreatePinnedToCore(
                    Task2code,   /* 任務函數 */
                    "Task2",     /* 任務名稱 */
                    10000,       /* 任務推疊大小 */
                    NULL,        /* 任務參數 */
                    1,           /* 任務優先權(0是最低優先權 */
                    NULL,      /* 欲追蹤處理的任務名稱 (可使用 &Task2 或 NULL) */
                    0);          /* 指定此任務的執行核心(0或1) */ 
 


  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); //Start the OLED display
  display.display();
  delay(3000);
  //檢查
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("找不到MAX30102");
    while (1);
  }
  byte ledBrightness = 0x7F; //亮度Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only(心跳), 2 = Red + IR(血氧)
  //Options: 1 = IR only, 2 = Red + IR on MH-ET LIVE MAX30102 board
  int sampleRate = 800; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 215; //Options: 69, 118, 215, 411
  int adcRange = 16384; //Options: 2048, 4096, 8192, 16384
  // Set up the wanted parameters
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  particleSensor.enableDIETEMPRDY();

  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
  display.clearDisplay();//清除螢幕

//  //--------------------------------------------
//  WiFi.mode(WIFI_STA);
//  WiFi.begin(ssid, password);
//
//  Serial.print("Connecting to Wi-Fi");
//  while (WiFi.status() != WL_CONNECTED) {
//    delay(500);
//    Serial.print(".");
//  }
//  Serial.println("OK");
//  //--------------------------------------------

//  u8g2.setFont(u8g2_font_unifont_t_chinese1); //使用我們做好的字型
//  u8g2.firstPage();
//     do {
//     u8g2.setCursor(35, 40);
//     u8g2.print("放上手指");
//   } while ( u8g2.nextPage() );
}


 



//顯示
void showRate(){
          display.setTextColor(WHITE);
          //心律曲線
          int y=62-beatAvg/3;
          display.writeLine(lastx,lasty,x,y,WHITE);
          lasty=y;
          lastx=x;
          //血氧曲線
          int yO2=31-ESpO2/6;
          display.writeLine(lastxO2,lastyO2,x,yO2,WHITE);
          lastyO2=yO2;
          lastxO2=x;
          display.display();//顯示螢幕
          x++; 
          //文字
         display.writeFillRect(0,0,128,12,BLACK);
         display.setCursor(0,0);
         display.print("BPM:");
         display.print(beatAvg);
         if (beatAvg>30){
          display.print(" Oxygen:");
          display.print(ESpO2);
          display.print("%");
          //修改上傳旗標=true
          SendFlag = true;
         }
}



void loop() { 
  //是否有按下按鈕
//  Serial.print("loop主流程:，at core:");
//  Serial.println(xPortGetCoreID());
  
  if (dMode != last_dMode) {   // != logical "not equal"
    Serial.println("Buttonpress detected");
    last_dMode = dMode;
    cht=!cht;
  }

  if(x>127)  
  {
    display.clearDisplay();
    x=0;
    lastx=x;
    lastxO2=x;
  }
  
//  long irValue = particleSensor.getIR();    //Reading the IR value it will permit us to know if there's a finger on the sensor or not
   irValue = particleSensor.getIR();    //Reading the IR value it will permit us to know if there's a finger on the sensor or not
  //是否有放手指
  if (irValue > FINGER_ON )  {
//    display.clearDisplay();//清除螢幕
//    display.drawBitmap(5, 5, logo2_bmp, 24, 21, WHITE);//顯示小的心跳圖示
//    display.setTextSize(2);//設定文字大小
//    display.setTextColor(WHITE);//文字顏色
//    display.setCursor(42, 10);//設定游標位置
//    display.print(beatAvg); display.println(" BPM");//顯示心跳數值
//    display.drawBitmap(0, 35, O2_bmp, 32, 32, WHITE);//顯示氧氣圖示
//    display.setCursor(42, 40);//設定游標位置

    //是否有心跳
    if (checkForBeat(irValue) == true) {
//      display.clearDisplay();//清除螢幕
//      display.drawBitmap(0, 0, logo3_bmp, 32, 32, WHITE);//顯示大的心跳圖示
//      display.setTextSize(2);//設定文字大小
//      display.setTextColor(WHITE);//文字顏色
//      display.setCursor(42, 10);//設定游標位置
//      display.print(beatAvg); display.println(" BPM");//顯示心跳數值
//      display.drawBitmap(0, 35, O2_bmp, 32, 32, WHITE);//顯示氧氣圖示
//      display.setCursor(42, 40);//設定游標位置
      
      tone(Tonepin, 1000);//發出聲音
      delay(10);
      noTone(Tonepin);//停止聲音
      Serial.print("beatAvg="); Serial.println(beatAvg);//將心跳顯示到序列
      long delta = millis() - lastBeat;//計算心跳差
      lastBeat = millis();
      beatsPerMinute = 60 / (delta / 1000.0);//計算平均心跳
      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        //心跳必須再20-255之間
        rates[rateSpot++] = (byte)beatsPerMinute; //儲存心跳數值陣列
        rateSpot %= RATE_SIZE;
        beatAvg = 0;//計算平均值
        for (byte x = 0 ; x < RATE_SIZE ; x++) beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
        showRate(); 
//        print_BPM(); 
      }
    }
    
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
        Serial.print("Oxygen % = "); Serial.println(ESpO2);
        sumredrms = 0.0; sumirrms = 0.0; SpO2 = 0;
        i = 0;
      }
      particleSensor.nextSample(); //We're finished with this sample so move to next sample
    }

  } else {
    //清除心跳數據
    for (byte rx = 0 ; rx < RATE_SIZE ; rx++) rates[rx] = 0;
    beatAvg = 0; rateSpot = 0; lastBeat = 0;
    //清除血氧數據
    avered = 0; aveir = 0; sumirrms = 0; sumredrms = 0;
    SpO2 = 0; ESpO2 = 90.0;
    if (cht) {
      showRate();
    }
    noTone(Tonepin);
  }                                                                                                                                                                                                                                             


  
}
