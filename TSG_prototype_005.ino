//------------------------------------------------------------
//    姿勢制御フィルタリングプログラム
//                Arduino　IDE　1.6.11
//
//　　　Arduino　　　　　　　　LSM9DS1基板　
//　　　　3.3V　　　------　　　　3.3V
//　　　　GND       ------   　　 GND
//　　　　SCL       ------        SCL
//　　　　SDA       ------        SDA
//
//　センサーで取得した値をシリアルモニターに表示する
//
//　　　　
//----------------------------------------------------------//


#include <SPI.h>                                //SPIライブラリ
#include <Wire.h>                               //I2Cライブラリ
#include <SparkFunLSM9DS1.h>                  //LSM9DS1ライブラリ：https://github.com/sparkfun/LSM9DS1_Breakout
#include <SD.h>
#include <LSM9DS1_Registers.h>
#include <LSM9DS1_Types.h>
#include <SoftwareSerial.h>
#include <MsTimer2.h>            //https://github.com/PaulStoffregen/MsTimer2


//#define ADAddr 0x48//

#define LSM9DS1_M  0x1E                 // SPIアドレス設定 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B                // SPIアドレス設定 if SDO_AG is LOW

//#define PRINT_CALCULATED              //表示用の定義
//#define DEBUG_GYRO                    //ジャイロスコープの表示


#define RX 8                            //GPS用のソフトウェアシリアル
#define TX 9                            //GPS用のソフトウェアシリアル
#define SENTENCES_BUFLEN      128        // GPSのメッセージデータバッファの個数

#define UPDATE_INTERVAL        50       //モーションセンサーの値取得間隔
#define DATAPUSH_INTERVAL     200       //モーションセンサーの値記録間隔

//-------------------------------------------------------------------------
//[Global valiables]

LSM9DS1 imu;


//###############################################
//MicroSD 
//const int chipSelect = 4;//Arduino UNO
const int chipSelect = 10;//Arduino Micro

File dataFile;                          //SD CARD
boolean sdOpened = false;
//###############################################

const int tact_switch = 7;//タクトスイッチ
boolean switchIs;
boolean switchOn;
boolean switchRelease;


volatile boolean enableWrite = false;

///////////////////////カルマンフィルタ/////////////////////////////
String motionData;
volatile unsigned long time;

////////////////////////////////////////////////////////////////


//@@@@@@@@@@@@@@@@@@@@@@@@@@@@
unsigned long delta;

unsigned long delta1;
unsigned long delta2;
void test(String s)
{
  Serial.print(millis() - delta);
  Serial.print("::DEBUG:");
  Serial.println(s);

  delta = millis();
  
}
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@


void setup(void) {

  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  //=== SD Card Initialize ====================================
  Serial.print(F("Initializing SD card..."));
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println(F("Card failed, or not present"));
    // don't do anything more:
    return;
  }
  Serial.println(F("card initialized."));
  //=======================================================

  //===タクトスイッチ===========================================
  pinMode(tact_switch, INPUT);
  switchIs = false;
  //=======================================================

  //=== LSM9DS1 Initialize =====================================
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress  = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;

  if (!imu.begin())              //センサ接続エラー時の表示
  {
    Serial.println(F("Failed to communicate with LSM9DS1."));
    while (1)
      ;
  }
  //=======================================================


  //===MotionSensorのタイマー化 =================
  delay(100);
  motionData = "";
  //MsTimer2::set(100, updateMotionSensors);
  //MsTimer2::start();
  //=======================================================
  
  //DEBUG
  delta = millis();
  delta1 = millis();
  delta2 = millis();

  time = 0;
}



/**
 * 　==================================================================================================================================================
 * loop
 * ずっと繰り返される関数（何秒周期？）
 * 【概要】
 * 　10msでセンサーデータをサンプリング。
 * 　記録用に、100ms単位でデータ化。
 * 　蓄積したデータをまとめて、1000ms単位でSDカードにデータを出力する。
 * 　==================================================================================================================================================
 */
void loop(void) {

  //START switch ============================================
  switch(digitalRead(tact_switch)){

   case 0://ボタンを押した
          switchOn = true;
          break;

   case 1://ボタン押していない
           if(switchOn){
             //すでにOnなら、falseにする
             if(switchIs)
               switchIs = false;
             //すでにOffなら、trueにする
             else
               switchIs = true;

             switchOn = false;
           }
           break;

    default:
           break;
  }

  //スイッチの判定
  if(!switchIs){ //falseなら、ループする
    digitalWrite(13, 0);
    if (sdOpened) {
      sdcardClose();
    }
    else{
      ; 
    }
    return;
  }
  else{
    digitalWrite(13, 1);
    if (sdOpened) {
      ;
    }
    else{
      sdcardOpen();
    }
  }
  //END switch ============================================


  //START MotionSensor ============================================


    pushMotionData();

  
  //END MotionSensor ============================================

  //SDカードへの出力
  writeDataToSdcard();


}


//==================================================================================================================================================


/**
 * sdcardOpen
 */
void sdcardOpen()
{


  // ファイル名決定
  String s;
  int fileNum = 0;
  char fileName[16];
  
  while(1){
    s = "LOG";
    if (fileNum < 10) {
      s += "00";
    } else if(fileNum < 100) {
      s += "0";
    }
    s += fileNum;
    s += ".TXT";
    s.toCharArray(fileName, 16);
    if(!SD.exists(fileName)) break;
    fileNum++;
  }


  dataFile = SD.open(fileName, FILE_WRITE);

  if(dataFile){
    Serial.println(fileName);
    sdOpened = true;
  }
  else
    Serial.println("fileError");

}

/**
 * sdcardClose
 */
void sdcardClose()
{
    dataFile.close();
    Serial.println(F("SD Closed."));
    sdOpened = false;

}


/**
 * writeDataToSdcard
 */
 
void writeDataToSdcard()
{

  //File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    
    dataFile.print(motionData);

    //dataFile.close();
    
    Serial.println(motionData);


    //クリア
    motionData = "";;
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println(F("error opening datalog.txt"));
  }

}


/**
 * updateMotionSensors
 * 
 */

void updateMotionSensors()
{
  imu.readGyro();
  imu.readAccel();
  imu.readMag();    
}


/**
 * pushMotionSensor
 * 
 *  * @enableWrite    GPS受信 "A"
 */
void pushMotionData()
{


      //時間の更新
      double dt = (double)(millis() - time); // Calculate delta time  
      time = millis();

      updateMotionSensors();
          
      motionData += "$MOTION"; 
      motionData += ",";
    
      motionData += dt; 
      motionData += ",";
    
      motionData += imu.ax;
      motionData += ",";
      motionData += imu.ay;
      motionData += ",";
      motionData += imu.az;
      motionData += ",";
    
      motionData += imu.gx;
      motionData += ",";
      motionData += imu.gy;
      motionData += ",";
      motionData += imu.gz;
      motionData += ",";
      
      motionData += imu.mx;
      motionData += ",";
      motionData += imu.my;
      motionData += ",";
      motionData += imu.mz;
    
    
      motionData += "\n";

}

