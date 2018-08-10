/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <math.h>
#include <ESP8266WiFi.h>
#include <time.h>

/*
const char* ssid = "Boneless Pizza";
const char* password = "understandable";
int timezone = 7 * 3600;
int dst = 0;
String tanggal;
String jam;
String waktu;
*/


// OLED display TWI address
#define OLED_ADDR   0x3C
// reset pin not used on 4-pin OLED module
Adafruit_SSD1306 display(-1);  // -1 = no reset pin
// 128 x 64 pixel display
#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif


#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
// sensitivity scale factor respective to full scale setting provided in datasheet 
const uint16_t AccelScaleFactor = 16384;
const uint16_t GyroScaleFactor = 131;

// initialitation
double accX, accY, accZ, Ax, Ay, Az;
double R, PGA, PGATotal;
double gyroX, gyroY, gyroZ;
unsigned long startTime, endTime;
float m_PGA, i, j, pwm, defuz;
float durasi [3];
float golongan [5];
float rule [3][5];
float result[15][2];
byte state;
unsigned int counter;
int16_t tempRaw;
int statusPin,statusRelay;
String waktuMulai, waktuSelesai, waktuKejadian;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// Select SDA and SCL pins for I2C communication 
const uint8_t scl = 12; //D6
const uint8_t sda = 13; //D7
const uint8_t led = 5; //D1
const int relayPin = 2; //D4

// TODO: Make calibration routine

void setup() {
  Wire.begin(sda,scl);
  // initialize and clear display
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  display.display();
  delay(2000);
  
  Serial.begin(115200);
  
  pinMode(led, OUTPUT);
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);
  statusRelay = digitalRead(relayPin);
  
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
   
}

void loop() {

//-------------------------------------------------kalman filter-MPU6050
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

//----------------------------------------------------MAIN CODE------------
//divide each with their sensitivity scale factor
  Ax = (double)accX/AccelScaleFactor;
  Ay = (double)accY/AccelScaleFactor;
  Az = (double)accZ/AccelScaleFactor;
  R =  sqrt((Ax*Ax)+(Ay*Ay)+(Az*Az));
  PGA = R - 0.95;
  
    /* Print Data */
#if 1 // Set to 1 to activate
  //Serial.print(Ax); Serial.print("\t");
  //Serial.print(Ay); Serial.print("\t");
  //Serial.print(Az); Serial.print("\t");
  Serial.print(PGA);
  Serial.print(",");
  Serial.print(statusPin);
  Serial.print(",");
  Serial.println(statusRelay);
  
#endif
delay (50);
//----------------------------------------------------HITUNG GEMPA------------
if (PGA > 0.014){
    if(state == 0){
      startTime = millis();
      state++;
    }
    else if (state == 1){
      if((millis() - startTime)>250){
          delay(5);
          display.setTextSize(1);
          display.setTextColor(WHITE);
          display.setCursor(15,0);
          display.print("GEMPA TERDETEKSI");
          display.display();
          delay(5);
          digitalWrite(led, HIGH);
          statusPin = digitalRead(led);
        PGATotal = 0; counter = 0; m_PGA=0;
        state++;
      }
    }
    if (state == 2){
      endTime = millis();
      PGATotal += fabs(PGA);
      if (PGA>m_PGA){
        m_PGA=PGA;
      }
      counter++;
      if((counter % 10)==0){
        hitungGempa();
        display.clearDisplay();
      }
    }
  }
  else {
    if((state == 2)&&(millis() - endTime)>2000){
     double h = millis()-endTime;
     //waktuSelesai = waktu;
     //Serial.print(" END TIME : ");Serial.println(h);
     //Serial.println("Gempa Berakhir");
     digitalWrite(led, LOW);
     statusPin = digitalRead(led);
     hitungGempa();
     m_PGA=0;
     state = 0;
    }
    if(state==1){
      state = 0;
    }
  }
  delay(50);

}
//----------------------------------------VOID
void hitungGempa(){
  double acc = PGATotal / counter;
  double waktu = (endTime - startTime) / 1000.0;
  double logMMI = 3.625*(log(acc*100)/log(10));
  double MMI = 0.287+logMMI;//1.68+2.58*log(acc);//
  double X = 0.287+3.625*log(fabs(Ax-0.04)*100);
  double Y = 0.287+3.625*log(fabs(Ay+0.04)*100);
  double Z = 0.287+3.625*log(fabs(Az-0.95)*100);
  float richter = 2.2+1.8*log((acc*100)/log(10));
  delay(1);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  
  display.setCursor(0,15);
  display.print("MMI: ");
  display.setCursor(25,15);
  display.print(floor(MMI));
  
  display.setCursor(0,27);
  display.print("Richter: ");
  display.setCursor(50,27);
  display.print(richter);
  
  display.setCursor(0,37);
  display.print("Durasi: ");
  display.setCursor(45,37);
  display.print(waktu);
  
  display.setCursor(0,47);
  display.print("Acc: ");
  display.setCursor(25,47);
  display.print(acc);  
  
  display.display();
  delay(50);
  

  //Serial.print(" MMI : ");Serial.println(String(floor(MMI)));
  //Serial.print(" Waktu: ");Serial.println(String(waktu));
  //Serial.print(" Acc  : ");Serial.println(String(acc));
  //Serial.print(" Richter ACC : ");Serial.println(String(richter));
//----------------------------------------------------------FUZZY DURASI

  //  Singkat
  if (waktu <= 2)
  {  durasi [0] = 1;}
  else if (waktu > 2 && waktu < 4)
  {  durasi [0] = (4 - waktu)/(4 - 2); }
  else
  { durasi [0] = 0;}

  // Normal
  if (waktu <= 2)
  {
  durasi [1] = 0;}
  else if (waktu > 2 && waktu < 4)
  { durasi [1] = (waktu-2)/(4-2);}
  else if (waktu >= 4 && waktu <= 5)
  { durasi [1] = 1;}
  else if (waktu >5 && waktu <7)
  { durasi [1] = (7 - waktu)/(7 - 5);}
  else{
    durasi [1] = 0;
  }

  // Lama
  if (waktu <= 5)
  { durasi [2] = 0;}
  else if (waktu > 5 && waktu <= 7)
  { durasi [2] = (waktu-5)/(7-5);}
  else
  { durasi [2] = 1;}

//---------------------------------------------------------FUZZY GOLONGAN
  // gol4
  if (acc <= 0.092)
  { golongan [0] = 1;}
  else if (acc > 0.092 && acc <= 0.18)
  {  golongan [0] = (0.18 - acc)/(0.18 - 0.092); }
  else
  { golongan [0] = 0;}

  // gol6
  if (acc <= 0.092)
  { golongan [1] = 0;}
  else if (acc > 0.092 && acc <= 0.18)
  { golongan [1] = (acc-0.092)/(0.18-0.092);}
  else if (acc > 0.18 && acc <= 0.34)
  { golongan [1] = (0.34-acc)/(0.34 - 0.18);}
  else
 { golongan [1] = 0;}

  // gol7
  if (acc <= 0.18)
  { golongan [2] = 0;}
  else if (acc > 0.18 && acc <= 0.34)
  { golongan [2] = (acc-0.18)/(0.34-0.18);}
  else if (acc > 0.34 && acc <= 0.65)
  { golongan [2] = (0.65-acc)/(0.65 - 0.34);}
  else
 { golongan [2] = 0;}

  // gol8
  if (acc <= 0.34)
  { golongan [3] = 0;}
  else if (acc > 0.34 && acc <= 0.65)
  { golongan [3] = (acc-0.34)/(0.65-0.34);}
  else if (acc > 0.65 && acc <= 1.24)
  { golongan [3] = (1.24-acc)/(1.24 - 0.65);}
  else
 { golongan [3] = 0;}

  // gol9
  if (acc <= 0.65)
  { golongan [4] = 0;}
  else if (acc > 0.65 && acc <= 1.24)
  { golongan [4] = (acc-0.65)/(1.24-0.65);}
  else
  { golongan [4] = 1;}
//---------------------------------------------------- DEFUZZYFIKASI
 delay(1);
 int i, j, k=1;
 float temp;
 for ( i=0; i<=4; i=i+1)
 {
   for ( j=0; j<=2; j=j+1)
   {
     temp = min(golongan[i], durasi[j]);
   
       result[k][0] = temp;
    delay(1);
       if((i==0)&&(j==0)){
         result[k][1]=1;
       }else if ((i==0)&&(j==1)){
         result[k][1]=1;
       }else if((i==0)&&(j==2)){
         result[k][1]=1;
       }else if((i==1)&&(j==0)){
         result[k][1]=1;
       }else if((i==1)&&(j==1)){
         result[k][1]=1;
       }else{
         result[k][1]=0;
       }
       delay(1);
       k++;
  }    
 }
 
  int nyala,mati,p,q;
  float max1=0,max0=0;
  float resultmax1[5];
  for (q=1; q<5; q++){
    resultmax1[q] = result[q][0];
  }
  Serial.println(" ");
  for ( p=1; p<=5; p++){
      //nyala++;
      max1=max(max1,resultmax1[p]);             
  }
  for ( p=6; p<=15; p++){
      // mati++;
      max0=max(max0,result[p][0]); 
  }
  if(max1<max0){
    digitalWrite(relayPin, HIGH);
    statusRelay = digitalRead(relayPin);
  }
}
