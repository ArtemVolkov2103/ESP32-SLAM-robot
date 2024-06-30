#include "BluetoothSerial.h"
// Проверка, что встроенный Bluetooth доступен на плате
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;

#include <Wire.h>
#define I2C_FREQ 400000
#define SDA_1 21
#define SCL_1 22
#define SDA_2 18
#define SCL_2 19
TwoWire I2C_1 = TwoWire(0);
TwoWire I2C_2 = TwoWire(1);

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &I2C_1);  // гироскоп подключен по I2C к 21 и 22 gpio, адррес выставлен подтяжкой к земле

#include "xv_lib.h"
#define LDS_MOTOR_EN_PIN 4  // LDS enable pin
#define LDS_MOTOR_PWM_CHANNEL 4
#define LDS_MOTOR_PWM_BITS 11
#define LDS_MOTOR_PWM_HZ 30000
const int SENSOR_TX = 32;
const int SENSOR_RX = 33;
void scan_callback(uint16_t, uint16_t, uint16_t, byte);
void motor_callback(int);
void packet_callback(uint16_t, byte*, uint16_t);
XV xv_lds;
long previousMillis = 0;
long interval = 5;
int LDS_Data[4];
int LDS_Distance[360];

bool isCompited = false;
byte stage = 0;
int iter = 0;
//byte array[100][200];
float deltaX = 0, deltaY = 0;
float xCoordNew = 0, yCoordNew = 0, xCoord = 0, yCoord = 0;  //старое целевое положение робота и старое целевое
float nextDist = 0.0, robotAngle = 0, newRobotAngle = 0,
      currentX = 0.0, currentY = 0.0;  //положение на координатах в текущий момент
String inString = "";
long tmr_St = 0;
char inChar;

float gyroAngle = 0;
int x, y, z;
int left_intr = 0;
int right_intr = 0;
int h_intr = 0;
unsigned int rotR = 0;
unsigned int rotL = 0;
unsigned long int tm;
unsigned long int spdR = 210;
unsigned long int spdL = 250;
unsigned int dt = 0;

unsigned long currentTimeR = 0, currentTimeL = 0, previousTimeR = 0, previousTimeL = 0, timeInterval = 1000;
// задаем свойства ШИМ-сигнала:
const int freq = 500;
const int resolution = 8;
const int RMotorChannel0 = 0;
const int RMotorChannel1 = 1;
const int LMotorChannel2 = 2;
const int LMotorChannel3 = 3;

byte CurrentMove = 0;  //текущее движение: стоп-0 вперед-1 вправо-2 влево-3

void Left_ISR() {
  unsigned long currentTime = micros();  // or micros()
  if (currentTime - previousTimeL > timeInterval) {
    left_intr++;
    rotL++;
    previousTimeL = currentTime;
  }
  //delay(2);
}
void Right_ISR() {
  unsigned long currentTime = micros();  // or micros()
  if (currentTime - previousTimeR > timeInterval) {
    right_intr++;
    rotR++;
    previousTimeR = currentTime;
  }
  //delay(2);
}


void setup() {
  digitalWrite(4, LOW);
  Serial.begin(115200);
  SerialBT.begin("ESP32");            //имя, которое будет отобрааться для других устройств
  I2C_1.begin(SDA_1, SCL_1, 400000);  // SDA pin 21, SCL pin 22 TTGO TQ
  I2C_2.begin(SDA_2, SCL_2, 400000);  // SDA2 pin 18, SCL2 pin 19
  delay(800);
  if (!bno.begin()) {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }
  Serial.println("BNO055 started");

  delay(1000);
  bno.setExtCrystalUse(true);
  delay(1000);  //задержка нобходима для корректной инициализации датчиков по i2c

  pinMode(LDS_MOTOR_EN_PIN, OUTPUT);
  ledcAttachChannel(LDS_MOTOR_EN_PIN, LDS_MOTOR_PWM_HZ, LDS_MOTOR_PWM_BITS, LDS_MOTOR_PWM_CHANNEL);
  ledcWrite(LDS_MOTOR_EN_PIN, 2047);
  Serial1.begin(115200, SERIAL_8N1, SENSOR_RX, SENSOR_TX);  // XV LDS data

  xv_lds.setScanPointCallback(xv_scan_callback);
  xv_lds.setMotorPwmCallback(xv_motor_pwm_callback);
  xv_lds.setPacketCallback(xv_packet_callback);
  xv_lds.enableMotor(true);
  pinMode(36, INPUT_PULLUP);
  pinMode(39, INPUT_PULLUP);
  pinMode(23, INPUT_PULLUP);
  attachInterrupt(36, Left_ISR, RISING);   //функция Left_ISR будет вызываться когда будет поступать прерывание от левого колеса
  attachInterrupt(39, Right_ISR, RISING);  //функция Right_ISR будет вызываться когда будет поступать прерывание от правого колеса
  ledcAttachChannel(25, freq, resolution, RMotorChannel0);
  ledcAttachChannel(26, freq, resolution, RMotorChannel1);
  ledcAttachChannel(27, freq, resolution, LMotorChannel2);
  ledcAttachChannel(13, freq, resolution, LMotorChannel3);
}


void loop() {
  UpdateGyroKompas();
  GetLidarData();
  GetCoordinates();
  MoveTo();
  //Serial.println(currentX);
}
String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
void GetCoordinates() {
  if (Serial.available())  // Слушаем порт
  {
    //char inChar = Serial.read();  // Читаем символ с SerialBT модуля
    //inString = inString + inChar;  // Переводим Char в String, набираем нужное кол-во символов
    //inString = Serial.readStringUntil('\n');
    inString = Serial.readString();
    float part01 = getValue(inString, ' ', 0).toFloat();
    float part02 = getValue(inString, ' ', 1).toFloat();

    /*Serial.println(part01);
    Serial.println(part02);*/
    GetVector(part01, part02);
    /*currentX = xCoord;
    currentY = yCoord;*/
    inString = "";  // Обнуляем строку
  }
}

void GetVector(float x, float y) {
  xCoordNew = x;
  yCoordNew = y;

  nextDist = sqrt((sq(x - xCoord) + sq(y - yCoord)));  // дистанция до точки
  deltaX = xCoord - xCoordNew;                         //координаты вектора от одной точки к другой
  deltaY = yCoord - yCoordNew;
  deltaX = abs(deltaX);

  if (nextDist != 0)
    newRobotAngle = acos(deltaX / sqrt(sq(deltaX) + sq(deltaY))) * 57.3;  //угол радиус-вектора точки относительно вектора i(1;0)
  else newRobotAngle = robotAngle;
  if (yCoordNew < yCoord) {  //если точка ниже робота по Y
    newRobotAngle = ((2 * 180) - newRobotAngle);
  }
  if (xCoordNew < xCoord) {  //если точка ниже робота по Y
    newRobotAngle = (180 - newRobotAngle);
  }
}

void MoveTo() {
  //Serial.print("newRobotAngle: "); Serial.print(newRobotAngle);
  //Serial.print(" robotAngle: "); Serial.print(robotAngle);

  //Serial.print(" gyroAngle: "); Serial.println(gyroAngle);
  /*Serial.print(" nextDist: "); Serial.print(nextDist);*/

  /*Serial.print(" xCoord: "); Serial.print(xCoord);
  Serial.print(" yCoord: "); Serial.print(yCoord);
  Serial.print(" currentX: "); Serial.print(currentX);
  Serial.print(" currentY: "); Serial.println(currentY);*/


  if (abs(newRobotAngle - robotAngle) >= 3 && nextDist > 0) {
    GoToAngle(newRobotAngle);

  }

  else if (abs(newRobotAngle - robotAngle) < 3 && nextDist > 0) {  // необходимый угол достигнут
    Forward(nextDist);
    //Serial.print("  Forward: "); Serial.println(nextDist);
  }

  /*if(abs(newRobotAngle-robotAngle) >= 175 && nextDist > 0){ // необходимый угол достигнут
    Backward(nextDist);
    //Serial.print("  Backward: "); Serial.println(nextDist);
  }*/

  /*else if ((newRobotAngle+7.0) < robotAngle || (newRobotAngle-robotAngle) > 180) {
      GoToAngle(newRobotAngle);
  }*/
  else {  //робот встал в необходимый угол и проехал N метров, контролирую текущее направление
    if (abs(newRobotAngle - gyroAngle) >= 3)
      GoToAngle(newRobotAngle);
    else Stop();
  }
}

void GetLidarData() {
  while (Serial1.available() > 0) {  // read byte from LDS
    xv_lds.processByte(Serial1.read());
  }
  if (!xv_lds.loop()) {
    // LDS motor error
    //Serial.println("LDS motor error");
    //xv_lds.enableMotor(false);
  }
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
    if (LDS_Data[3] == 0) {
      /*SerialBT.print(LDS_Data[0]);
        SerialBT.print("\t");
        SerialBT.println(LDS_Data[1]);*/
      /*Serial.print(LDS_Data[0]);  //distance_mm
      Serial.print("\t");
      Serial.println(LDS_Data[1]);  //angle_deg*/
      
        SerialBT.print(int(cos((LDS_Data[1] + gyroAngle) * 0.017) * LDS_Data[0] + currentX*100));
      SerialBT.print("\t");
      SerialBT.println(int(sin((LDS_Data[1] + gyroAngle) * 0.017) * LDS_Data[0] + currentY*100));
      
      
      /*Serial.print("ANG ");Serial.print(LDS_Data[0]); Serial.print("\t");
      Serial.print("DIST ");Serial.print(LDS_Data[1]); Serial.print("\t");
      Serial.print("QUAL ");Serial.print(LDS_Data[2]); Serial.print("\t");
      Serial.print("ERR ");Serial.println(LDS_Data[3]);*/
    }
  }
}
void xv_scan_callback(uint16_t angle_deg, uint16_t distance_mm,
                      uint16_t quality, byte err) {
  LDS_Data[0] = distance_mm/10;
  LDS_Data[1] = angle_deg;
  LDS_Data[2] = quality;
  LDS_Data[3] = err;
  if (err == 0) {
    LDS_Distance[angle_deg] = distance_mm;
  }
  /*Serial.print(angle_deg);
  Serial.print(" ");
  Serial.print(distance_mm);
  Serial.print(" ");
  Serial.print(quality);
  Serial.print(" ");
  Serial.println(err);*/
  /*Serial.print(" ");
  Serial.println(xv_lds.getScanRPM());*/
}
void xv_motor_pwm_callback(float pwm) {

  /*Serial.print("Motor callback ");
  Serial.print(xv_lds.getScanRPM());
  Serial.print(" ");
  Serial.print(pwm);
  Serial.print(" ");*/
  int pwm_value = ((1 << LDS_MOTOR_PWM_BITS) - 1) * pwm;
  ledcWrite(LDS_MOTOR_EN_PIN, pwm_value);
  //Serial.println(pwm_value);
}
void xv_packet_callback(uint16_t starting_angle_deg, byte* packet, uint16_t length) {
  /*Serial.print(starting_angle_deg);
  Serial.print(" ");
  Serial.print(length);
  Serial.print(" ");
  Serial.println(xv_lds.getScanRPM());*/
  //Serial.write(packet, length); // dump raw data
}

void UpdateGyroKompas() {
  static uint32_t tmr;
  if (millis() - tmr >= 20) {  // таймер на 11 мс (на всякий случай)
    /* Get a new sensor event */
    sensors_event_t event;
    bno.getEvent(&event);
    gyroAngle = event.orientation.x;
    //SerialBT.println(gyroAngle);
    tmr = millis();  // сброс таймера
  }
}

void Action(char act, float val) {
  //x = cos(robotAngle*lastDistanceTravelled);
  //y = sin(robotAngle*lastDistanceTravelled);

  switch (act) {
    case 'F':
      while (!Forward(val))
        ;
      break;
    case 'B':
      if (!Backward(val))
        ;
      break;
    case 'L':
      left_intr = 0;
      Left();
      break;
    case 'R':
      left_intr = 0;
      Right();
      break;
    case 'A':
      GoToAngle(val);
      break;
    case 'S':
      left_intr = 0;
      Stop();
      break;
    default:
      Stop();
  }
}

void GoToAngle(float angle) {
    int rotation = gyroAngle - angle;  //359-0=359
    bool dir = true;                   //left

    if (rotation < 0)
      rotation = rotation + 360;

    if (rotation >= 180) {        //359 > 180
      dir = false;                //right
      rotation = 360 - rotation;  // 360-359=1°
    }
    if (rotation > 1) {
      if (rotation > 1 && rotation < 10) {  // уменьшаю мощность, чтобы не развернуться больше необходимого
        if (dir) Left(120);
        else Right(120);
      } else {
        if (dir) Left(255);
        else Right(255);
      }
    }
   else {
    Stop();
    isCompited = true;
  }
}

bool Forward(float meters) {
  int intrToGo = meters * 210;
  float rotation = gyroAngle - newRobotAngle;  //359-0=359
  bool dir = true;                             //left

  if (intrToGo - 1 > left_intr) {
    float left_intr_float = left_intr;
    currentX = xCoord + (left_intr_float / 210) * cos(radians(robotAngle));
    currentY = yCoord + (left_intr_float / 210) * sin(radians(robotAngle));
    isCompited = false;
    spdL = 255;
    spdR = 255;
    /*Serial.print(intrToGo);
    Serial.print(" LF:");
    Serial.println(left_intr);
    */
    
    if (rotation < 0)
      rotation = rotation + 360;

    if (rotation >= 180) {        //359 > 180
      dir = false;                //right
      rotation = 360 - rotation;  // 360-359=1°
    }
    if (rotation > 1) {
      if (dir) spdL = 255 / rotation;
      else spdR = 255 / rotation;
    }
    ledcWrite(25, spdR);
    ledcWrite(26, 0);
    ledcWrite(27, spdL);
    ledcWrite(13, 0);
    return isCompited;
  } else {
    nextDist = 0;
    Stop();
    isCompited = true;
    return isCompited;
  }
}
bool Backward(float meters) {
  int intrToGo = meters * 298;

  if (intrToGo - 1 > left_intr) {
    isCompited = false;
    /*Serial.print(intrToGo);
    Serial.print(" LB:");
    Serial.println(left_intr);*/
    ledcWrite(25, 0);
    ledcWrite(26, 210);
    ledcWrite(27, 0);
    ledcWrite(13, 255);
    return isCompited;
  } else {
    Stop();
    isCompited = true;
    return isCompited;
  }
}
void Left() {
  ledcWrite(25, 210);
  ledcWrite(26, 0);
  ledcWrite(27, 0);
  ledcWrite(13, 255);
}
void Left(int pwr) {
  ledcWrite(25, pwr);
  ledcWrite(26, 0);
  ledcWrite(27, 0);
  ledcWrite(13, pwr);
}
void Right() {
  ledcWrite(25, 0);
  ledcWrite(26, 210);
  ledcWrite(27, 255);
  ledcWrite(13, 0);
}
void Right(int pwr) {
  ledcWrite(25, 0);
  ledcWrite(26, pwr);
  ledcWrite(27, pwr);
  ledcWrite(13, 0);
}
void Stop() {
  ledcWrite(25, 0);
  ledcWrite(26, 0);
  ledcWrite(13, 0);
  ledcWrite(27, 0);
  //delay(100);
  xCoord = xCoordNew;
  yCoord = yCoordNew;
  robotAngle = newRobotAngle;
  left_intr = 0;
  //nextDist = 0;
}
