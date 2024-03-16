#include "BluetoothSerial.h"
// Проверка, что встроенный Bluetooth доступен на плате
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // гироскоп подключен по I2C к 21 и 22 gpio

#include "Adafruit_VL53L0X.h"
Adafruit_VL53L0X lox = Adafruit_VL53L0X();//подключаются ко второму I2C, 16 и 17 gpio
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure;
VL53L0X_RangingMeasurementData_t measure2;
float mapX1, mapY1, mapX2, mapY2;
float delta = 0;
float angle = 0;
float angle2 = 180;

long tmr_start;
long tmr_Lidar;
float lox_del = 0;
bool loop_starts = false;
int rotations = 0;
float prevTimer = 0;
float timeTo360 = 0.0;

bool isCompited = false;
byte stage = 0;
int iter = 0;
//byte array[100][200];
float deltaX = 0, deltaY = 0;
float xCoordNew = 0, yCoordNew = 0, xCoord = 0, yCoord = 0;  //старое целевое положение робота и старое целевое
float nextDist = 0.0, robotAngle = 0, newRobotAngle = 0,
      currentX = 0, currentY = 0;  //положение на координатах в текущий момент
String inString = "";
long tmr_St = 0;
char inChar;

float gyroAngle = 0;
int x, y, z;
float left_intr = 0;
int right_intr = 0;
int h_intr = 0;
unsigned int rotR = 0;
unsigned int rotL = 0;
unsigned long int tm;
unsigned long int spdR = 210;
unsigned long int spdL = 250;
unsigned int dt = 0;

void setup() {

  Wire.begin();
  Serial.begin(9600);
  SerialBT.begin("ESP32"); //имя, которое будет отобрааться для других устройств

  pinMode(25, OUTPUT);//пин подключения мосфета для мотора лидара
  pinMode(32, OUTPUT);//пин для настройки 1-го дальномера
  pinMode(33, OUTPUT);//для 2-го дальномера
  digitalWrite(32, LOW);
  digitalWrite(33, LOW);
  digitalWrite(32, HIGH);
  if (!lox.begin()) {
    Serial.println("Failed to boot first VL53L0X");
    while (1)
      ;
  }
  lox.setAddress((uint8_t)01);
  //lox.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED);
  lox.setMeasurementTimingBudgetMicroSeconds(20000);
  lox.startRangeContinuous();
  Serial.println("first VL53L0X started");
  digitalWrite(33, HIGH);
  if (!lox2.begin()) {
    Serial.println("Failed to boot second VL53L0X");
    while (1)
      ;
  }
  lox2.setAddress((uint8_t)02);
  lox2.setMeasurementTimingBudgetMicroSeconds(20000);
  lox2.startRangeContinuous();
  Serial.println("second VL53L0X started");

  /* Initialise the sensor */
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }
  Serial.println("BNO055 started");
  delay(1000);
  bno.setExtCrystalUse(true);

  digitalWrite(25, HIGH);//если лидар правильно инициализировался, то начинает вращаться
  attachInterrupt(digitalPinToInterrupt(26), Left_ISR, RISING);   //функция Left_ISR будет вызываться когда будет поступать прерывание от левого колеса
  attachInterrupt(digitalPinToInterrupt(27), Right_ISR, RISING);  //функция Right_ISR будет вызываться когда будет поступать прерывание от правого колеса
  attachInterrupt(digitalPinToInterrupt(23), H_ISR, RISING);//обработка прерываний для лидара
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


void loop() {
  UpdateGyroKompas();
  GetLidarData();
  if (SerialBT.available())  // Слушаем порт
  {
    char inChar = SerialBT.read();  // Читаем символ с SerialBT модуля
    //inString = inString + inChar;  // Переводим Char в String, набираем нужное кол-во символов
    inString = SerialBT.readStringUntil('\n');
    float part01 = getValue(inString, ' ', 0).toFloat();
    float part02 = getValue(inString, ' ', 1).toFloat();

    /*SerialBT.println(part01);
      SerialBT.println(part02);*/
    GetVector(part01, part02);
    currentX = xCoord;
    currentY = yCoord;
    inString = "";  // Обнуляем строку
  }
  MoveTo();
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
  /*Serial.print("newRobotAngle: "); Serial.print(newRobotAngle);
  Serial.print(" robotAngle: "); Serial.print(robotAngle);

  Serial.print(" gyroAngle: "); Serial.print(gyroAngle);
  Serial.print(" nextDist: "); Serial.print(nextDist);

  Serial.print(" xCoord: "); Serial.print(xCoord);
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
  tmr_start = millis();
  lox.rangingTest(&measure, false);
  lox2.rangingTest(&measure2, false);
  if (rotations > 1) {
    if (measure.RangeStatus != 4 && (measure.RangeMilliMeter / 10 - 3) < 150) {  // если дальномер не ответил, то ничего не пишем в порт
      mapX1 = currentX + (measure.RangeMilliMeter / 10 - 3) * cos(angle + 180);
      mapY1 = currentY + (measure.RangeMilliMeter / 10 - 3) * sin(angle + 180);
      SerialBT.print(int(measure.RangeMilliMeter / 10 - 3));  //int(measure.RangeMilliMeter / 10 - 3)
      SerialBT.print("\t ");
      SerialBT.print(int(angle));  //int(angle)
      SerialBT.println("\t ");
      /*SerialBT.print(int(mapX1));
      SerialBT.print("\t ");
      SerialBT.print(int(mapY1));
      SerialBT.println("\t ");*/
      /*SerialBT.print(0);//int(measure.RangeMilliMeter / 10 - 3)
      SerialBT.print("\t ");
      SerialBT.print(0);//int(angle)
      SerialBT.print("\t ");
      SerialBT.print(int(measure2.RangeMilliMeter / 10 - 3));//int(measure2.RangeMilliMeter / 10 - 3)
      SerialBT.print("\t ");
      SerialBT.print(int(map(angle, 0, 360, 360, 0)));//int(angle-180)
      SerialBT.println("\t ");*/
      //Serial.print(" ");
    }
    if (measure2.RangeStatus != 4 && (measure2.RangeMilliMeter / 10) < 150) {
      mapX2 = currentX + (measure2.RangeMilliMeter / 10 - 3) * cos(angle);
      mapY2 = currentY + (measure2.RangeMilliMeter / 10 - 3) * sin(angle);

      SerialBT.print(int(measure2.RangeMilliMeter / 10));  //int(measure2.RangeMilliMeter / 10 - 3)
      SerialBT.print("\t ");
      SerialBT.print(int(angle2));  //int(angle-180)
      SerialBT.println("\t ");
      //lidarData[(int)angle] = measure2.RangeMilliMeter/ 10 - 3;
      /*SerialBT.print(mapX2);
      SerialBT.print("\t ");
      SerialBT.println(mapY2);*/
      //Serial.println(measure2.RangeMilliMeter / 10 - 3);
    }
    //SerialBT.println();
    /*Serial.print(" \t");
    Serial.print(angle);
    Serial.print(" \t");
    Serial.print(delta);
    Serial.print(" \t");
    Serial.println(timeTo360);*/
    angle -= delta;
    angle2 -= delta;
    if (angle2 < 0) angle2 = 360;
  }
  lox_del = millis() - tmr_start;
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

void CalculateWay() {
  if (lidarData[0] > lidarData[90] && lidarData[0] > lidarData[270]) {
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
      if (dir) Left(100);
      else Right(100);
    } else {
      if (dir) Left(120);
      else Right(120);
    }

  } else {
    Stop();
    isCompited = true;
  }
}

bool Forward(float meters) {
  int intrToGo = meters * 298;
  float rotation = gyroAngle - newRobotAngle;  //359-0=359
  bool dir = true;                             //left

  if (intrToGo - 1 > left_intr) {
    currentX = xCoord + (left_intr / 298) * cos(radians(robotAngle));
    currentY = yCoord + (left_intr / 298) * sin(radians(robotAngle));
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
    analogWrite(9, spdR);
    analogWrite(10, 0);
    analogWrite(11, spdL);
    analogWrite(12, 0);
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
    analogWrite(9, 0);
    analogWrite(10, 210);
    analogWrite(11, 0);
    analogWrite(12, 250);
    return isCompited;
  } else {
    Stop();
    isCompited = true;
    return isCompited;
  }
}
void Left() {
  analogWrite(9, 210);
  analogWrite(10, 0);
  analogWrite(11, 0);
  analogWrite(12, 250);
}

void Left(int pwr) {
  analogWrite(9, pwr);
  analogWrite(10, 0);
  analogWrite(11, 0);
  analogWrite(12, pwr);
}
void Right() {
  analogWrite(9, 0);
  analogWrite(10, 210);
  analogWrite(11, 250);
  analogWrite(12, 0);
}
void Right(int pwr) {
  analogWrite(9, 0);
  analogWrite(10, pwr);
  analogWrite(11, pwr);
  analogWrite(12, 0);
}
void Stop() {
  analogWrite(9, 0);
  analogWrite(10, 0);
  analogWrite(12, 0);
  analogWrite(11, 0);
  //delay(100);
  xCoord = xCoordNew;
  yCoord = yCoordNew;
  robotAngle = newRobotAngle;
  left_intr = 0;
  //nextDist = 0;
}

void Left_ISR() {
  left_intr++;
  rotL++;
  delay(2);
}
void Right_ISR() {
  right_intr++;
  rotR++;
  delay(2);
}
void H_ISR() {
  int t = millis() - tmr_Lidar;
  if (t > 2000) {
    timeTo360 = t;
    h_intr++;
    loop_starts = true;
    delta = 9;  //360 / (timeTo360 / lox_del);//360 / (timeTo360 / lox_del);
    rotations += 1;
    angle = 360;
    angle2 = 180;

    tmr_Lidar = millis();
  }
}
