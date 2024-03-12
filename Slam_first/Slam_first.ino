/*#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;
uint8_t fifoBuffer[45];  // буфер
float ypr[3];


#include <HMC5883L_Simple.h>
HMC5883L_Simple Compass;
float heading;

#define addr 0x1E  // I2C 7-битный адрес датчика HMC5883*/
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

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

#include "Adafruit_VL53L0X.h"
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
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

/*#include "GyverPID.h"
GyverPID regulatorR(0.1, 1.9, 0.0, 25);  // коэф. П, коэф. И, коэф. Д, период дискретизации dt (мс)
GyverPID regulatorL(0.1, 1.9, 0.0, 25);  // коэф. П, коэф. И, коэф. Д, период дискретизации dt (мс)
*/
bool isCompited = false;
byte stage = 0;
int iter = 0;
//byte array[100][200];
//int lidarValuesPerRotation[360];//попробовать один оборот записывать значения в массив, а в конце отправлять на SD 
float deltaX = 0, deltaY = 0;
float  xCoordNew = 0, yCoordNew = 0, xCoord = 0, yCoord = 0; //старое целевое положение робота и старое целевое 
float nextDist=0.0, robotAngle = 0, newRobotAngle = 0,
 currentX = 0, currentY = 0; //положение на координатах в текущий момент
String inString = "";
long tmr_St = 0;
char inChar;

byte lidarData[360];

void setup() {

  Wire.begin();
  Serial.begin(9600);
  Serial3.begin(9600);  //Скорость порта для связи Arduino с Serial3 модулем (15, 14) RX, TX
  
  /*
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);

  Compass.SetDeclination(+15, 13, 'N'); // на балкон
  Compass.SetSamplingMode(COMPASS_SINGLE);
  Compass.SetScale(COMPASS_SCALE_130);
  Compass.SetOrientation(COMPASS_HORIZONTAL_X_NORTH);*/

  pinMode(8, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);
  digitalWrite(22, LOW);
  digitalWrite(23, LOW);
  digitalWrite(22, HIGH);
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
  digitalWrite(23, HIGH);
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
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  Serial.println("BNO055 started");
  delay(1000);
  bno.setExtCrystalUse(true);
/*
  regulatorR.setDirection(NORMAL);  // направление регулирования (NORMAL/REVERSE). ПО УМОЛЧАНИЮ СТОИТ NORMAL
  regulatorR.setLimits(0, 255);     // пределы (ставим для 8 битного ШИМ). ПО УМОЛЧАНИЮ СТОЯТ 0 И 255
  regulatorR.setpoint = 200;         // сообщаем регулятору скорость, которую он должен поддерживать
  regulatorL.setDirection(NORMAL);  // направление регулирования (NORMAL/REVERSE). ПО УМОЛЧАНИЮ СТОИТ NORMAL
  regulatorL.setLimits(0, 255);     // пределы (ставим для 8 битного ШИМ). ПО УМОЛЧАНИЮ СТОЯТ 0 И 255
  regulatorL.setpoint = 200;         // сообщаем регулятору скорость, которую он должен поддерживать
  // в процессе работы можно менять коэффициенты
  */
  digitalWrite(8, HIGH);
  attachInterrupt(digitalPinToInterrupt(18), Left_ISR, RISING);   //функция Left_ISR будет вызываться когда будет поступать прерывание от левого колеса
  attachInterrupt(digitalPinToInterrupt(19), Right_ISR, RISING);  //функция Right_ISR будет вызываться когда будет поступать прерывание от правого колеса
  attachInterrupt(digitalPinToInterrupt(2), H_ISR, RISING);
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
  //UpdateSpeedPID();
  UpdateGyroKompas();
  GetLidarData();
  /*if (Serial.available())  // Слушаем порт
  {
    char inChar = Serial.read();   // Читаем символ
    //inString = inString + inChar;  // Переводим Char в String, набираем нужное кол-во символов
    inString = Serial.readStringUntil('\n');
    //if (inString.length() == 7)  // Поймали 5 символов
    //{
      float part01 = getValue(inString, ' ', 0).toFloat(); 
      float part02 = getValue(inString, ' ', 1).toFloat();

      Serial.println(part01);
      Serial.println(part02);
      GetVector(part01, part02);
      currentX = xCoord;
      currentY = yCoord;
      inString = "";  // Обнуляем строку
    //}
  }*/
  if (Serial3.available())  // Слушаем порт
  {
    char inChar = Serial3.read();   // Читаем символ с Serial3 модуля
    //inString = inString + inChar;  // Переводим Char в String, набираем нужное кол-во символов
    inString = Serial3.readStringUntil('\n');
      float part01 = getValue(inString, ' ', 0).toFloat(); 
      float part02 = getValue(inString, ' ', 1).toFloat();

      /*Serial3.println(part01);
      Serial3.println(part02);*/
      GetVector(part01, part02);
      currentX = xCoord;
      currentY = yCoord;
      inString = "";  // Обнуляем строку
  }
  //Forward(100000);
  MoveTo();
  /*if (Serial3.available())  // Слушаем порт
    inChar = Serial3.read();   // Читаем символ с Serial3 модуля
  switch(inChar){
      case 'F':
        analogWrite(9, regulatorR.getResultTimer());
        analogWrite(10, 0);
        analogWrite(12, regulatorL.getResultTimer());
        analogWrite(11, 0);
        break;
      case 'B':
        analogWrite(10, regulatorR.getResultTimer());
        analogWrite(9, 0);
        analogWrite(11, regulatorL.getResultTimer());
        analogWrite(12, 0);
        break;
      case 'L':
        Left();
        break;
      case 'R':
        Right();
        break;
      case 'S':
        Stop();
        break;
      default:
      Stop();
  }*/
  //Serial.print(currentX); Serial.print(" "); Serial.println(currentY); 
  //Serial.print("newRobotAngle: "); Serial.print(newRobotAngle);
  //Serial.print(" GyroAngle : ");  Serial.println(gyroAngle);
  /*Serial.print(" xCoord : "); Serial.print(xCoord);
  Serial.print(" deltaX : "); Serial.print(deltaX);
  Serial.print(" deltaY : "); Serial.print(deltaY);
  Serial.print(" Next x: "); Serial.print(x);
  Serial.print("  y: "); Serial.print(y);
  Serial.print(" Next dist: "); Serial.print(nextDist);
  Serial.print("  newRobotAngle: "); Serial.print(newRobotAngle);
  Serial.print(" GyroAngle : ");  Serial.print(gyroAngle);
  Serial.print("   old x: "); Serial.print(xCoord);
  Serial.print("  y: "); Serial.println(yCoord);*/
  
  //GoToAngle(180);
}

void GetVector(float x, float y){
  xCoordNew = x;
  yCoordNew = y;

  nextDist = sqrt( (sq(x-xCoord) + sq(y-yCoord)) ); // дистанция до точки
  deltaX = xCoord - xCoordNew; //координаты вектора от одной точки к другой
  deltaY = yCoord - yCoordNew;
  deltaX = abs(deltaX);
  
  if(nextDist != 0)
    newRobotAngle = acos( deltaX / sqrt( sq(deltaX) + sq(deltaY) ) )* 57.3;     //угол радиус-вектора точки относительно вектора i(1;0)
  else newRobotAngle = robotAngle;
  if (yCoordNew < yCoord){ //если точка ниже робота по Y
    newRobotAngle = ((2*180) - newRobotAngle);} 
  if (xCoordNew < xCoord){ //если точка ниже робота по Y
    newRobotAngle = (180 - newRobotAngle);} 
}

void MoveTo(){
  /*Serial.print("newRobotAngle: "); Serial.print(newRobotAngle);
  Serial.print(" robotAngle: "); Serial.print(robotAngle);

  Serial.print(" gyroAngle: "); Serial.print(gyroAngle);
  Serial.print(" nextDist: "); Serial.print(nextDist);

  Serial.print(" xCoord: "); Serial.print(xCoord);
  Serial.print(" yCoord: "); Serial.print(yCoord);
  Serial.print(" currentX: "); Serial.print(currentX);
  Serial.print(" currentY: "); Serial.println(currentY);*/


  if(abs(newRobotAngle-robotAngle) >= 3 && nextDist > 0){
    GoToAngle(newRobotAngle);
    
    }

  else if(abs(newRobotAngle-robotAngle) < 3 && nextDist > 0){ // необходимый угол достигнут
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
  else {//робот встал в необходимый угол и проехал N метров, контролирую текущее направление
    if(abs(newRobotAngle-gyroAngle) >= 3)
      GoToAngle(newRobotAngle);
    else Stop();
  }

}

void GetLidarData(){
  tmr_start = millis();
  lox.rangingTest(&measure, false);
  lox2.rangingTest(&measure2, false);
  if (rotations > 1) {
    if (measure.RangeStatus != 4 && (measure.RangeMilliMeter / 10 - 3) < 150 ) { // если дальномер не ответил, то ничего не пишем в порт
      mapX1 = currentX + (measure.RangeMilliMeter / 10 - 3)*cos(angle+180);
      mapY1 = currentY + (measure.RangeMilliMeter / 10 - 3)*sin(angle+180);
      Serial3.print(int(measure.RangeMilliMeter / 10 - 3));//int(measure.RangeMilliMeter / 10 - 3)
      Serial3.print("\t ");
      Serial3.print(int(angle));//int(angle)
      Serial3.println("\t ");
      /*Serial3.print(int(mapX1));
      Serial3.print("\t ");
      Serial3.print(int(mapY1));
      Serial3.println("\t ");*/
      /*Serial3.print(0);//int(measure.RangeMilliMeter / 10 - 3)
      Serial3.print("\t ");
      Serial3.print(0);//int(angle)
      Serial3.print("\t ");
      Serial3.print(int(measure2.RangeMilliMeter / 10 - 3));//int(measure2.RangeMilliMeter / 10 - 3)
      Serial3.print("\t ");
      Serial3.print(int(map(angle, 0, 360, 360, 0)));//int(angle-180)
      Serial3.println("\t ");*/
      //Serial.print(" ");
    }
    if (measure2.RangeStatus != 4 && (measure2.RangeMilliMeter / 10 ) < 150) { 
      mapX2 = currentX + (measure2.RangeMilliMeter / 10 - 3)*cos(angle);
      mapY2 = currentY + (measure2.RangeMilliMeter / 10 - 3)*sin(angle);

      Serial3.print(int(measure2.RangeMilliMeter / 10));//int(measure2.RangeMilliMeter / 10 - 3)
      Serial3.print("\t ");
      Serial3.print(int(angle2));//int(angle-180)
      Serial3.println("\t ");
      //lidarData[(int)angle] = measure2.RangeMilliMeter/ 10 - 3;
      /*Serial3.print(mapX2);
      Serial3.print("\t ");
      Serial3.println(mapY2);*/
      //Serial.println(measure2.RangeMilliMeter / 10 - 3);
    }
    //Serial3.println();
    /*Serial.print(" \t");
    Serial.print(angle);
    Serial.print(" \t");
    Serial.print(delta);
    Serial.print(" \t");
    Serial.println(timeTo360);*/
    angle -= delta;
    angle2 -= delta;
    if(angle2<0)angle2=360;
  }
  lox_del = millis() - tmr_start;
}

/*void UpdateSpeedPID() {
  regulatorR.input = spdR;   // сообщаем регулятору текущую температуру
  regulatorL.input = spdL;   // сообщаем регулятору текущую температуру
  dt = millis() - tm;        // вычисляем время с последнего расчёта
  if (dt >= 100) {            // если прошло 100мс или более, то начинаем расчёт
    spdR = rotR * 60000 / dt / 18;  //rotR*60000/dt/160; обороты в минуту
    spdL = rotL * 60000 / dt / 18;  //обороты в минуту
    rotR = 0;                // обнуляем счётчик
    rotL = 0;                // обнуляем счётчик
    tm = millis();           // запоминаем время расчёта
  }
}*/

void UpdateGyroKompas(){
  static uint32_t tmr;
  if (millis() - tmr >= 20) {  // таймер на 11 мс (на всякий случай)
    /* Get a new sensor event */ 
    sensors_event_t event; 
    bno.getEvent(&event);
    gyroAngle = event.orientation.x;
    //Serial3.println(gyroAngle);

    //Serial.println(gyroAngle);
    /* Display the floating point data */
    /*Serial.print("X: ");
    Serial.print(event.orientation.x, 4);
    Serial.print("\tY: ");
    Serial.print(event.orientation.y, 4);
    Serial.print("\tZ: ");
    Serial.print(event.orientation.z, 4);
    Serial.println("");*/
    /*heading = Compass.GetHeadingDegrees();

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      // переменные для расчёта (ypr можно вынести в глобал)
      Quaternion q;
      VectorFloat gravity;

      // расчёты
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);*/
      // выводим результат в радианах (-3.14, 3.14)
      /*Serial.print(degrees(ypr[0]) - 3.48); // вокруг оси Z
      Serial.print(',');
      Serial.print(degrees(ypr[1])); // вокруг оси Y
      Serial.print(',');
      Serial.print(degrees(ypr[2])); // вокруг оси X
      Serial.println();*/
      // для градусов можно использовать degrees()
     /* gyroAngle = map((degrees(ypr[0]) - 2.0), -180, 180, 0, 360); //degrees(ypr[0]) - 2.0;  
      //robotAngle = gyroAngle;
      
     
    }*/
     tmr = millis();  // сброс таймера
  }
}

void Action(char act, float val) {
  //x = cos(robotAngle*lastDistanceTravelled);
  //y = sin(robotAngle*lastDistanceTravelled);

  switch (act) {
    case 'F':
      while (!Forward(val));
      break;
    case 'B':
      if (!Backward(val));
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

void CalculateWay(){
  if(lidarData[0] > lidarData[90] && lidarData[0] > lidarData[270]){
    
  }
}

void GoToAngle(float angle) {

  int rotation = gyroAngle - angle;//359-0=359
  bool dir = true;  //left

  if (rotation < 0)
    rotation = rotation + 360;

  if (rotation >= 180) { //359 > 180
    dir = false;  //right
    rotation = 360 - rotation; // 360-359=1°
  }
  if (rotation > 1) {
    if (rotation > 1 && rotation < 10) { // уменьшаю мощность, чтобы не развернуться больше необходимого
      /*regulatorL.setpoint = 30;
      regulatorR.setpoint = 30;
      regulatorL.setLimits(0, 80);
      regulatorR.setLimits(0, 80);*/
      if (dir) Left(100);
      else Right(100);
    }
    else{
      if (dir) Left(120);
      else Right(120);
    }

  } 
  else{
    Stop();
    //robotAngle = angle;
    isCompited = true;
  }
  /*Serial.print("rotation: ");
  Serial.println(rotation);*/
  /*regulatorL.setpoint = 20;
  regulatorR.setpoint = 20;
  regulatorL.setLimits(0, 255);
  regulatorR.setLimits(0, 255);*/
  
}

bool Forward(float meters) {
  int intrToGo = meters * 298;
  //UpdateSpeedPID();
  float rotation = gyroAngle - newRobotAngle;//359-0=359
  bool dir = true;  //left
  
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

    if (rotation >= 180) { //359 > 180
      dir = false;  //right
      rotation = 360 - rotation; // 360-359=1°
    }
    if (rotation > 1) {
      if (dir) spdL = 255 / rotation;
      else spdR = 255 / rotation;
    } 
    analogWrite(9, spdR);
    //Serial.print(regulatorR.getResultTimer());
    analogWrite(10, 0);
    analogWrite(11, spdL);
    //Serial.print(" "); Serial.println(regulatorL.getResultTimer());
    analogWrite(12, 0);
    return isCompited;
  }
  else {
    nextDist = 0;
    Stop();
    //lastDistanceTravelled = meters;
    isCompited = true;
    return isCompited;
  }
}
bool Backward(float meters) {
  int intrToGo = meters * 298;

  if (intrToGo - 1  > left_intr) {
    isCompited = false;
    /*Serial.print(intrToGo);
    Serial.print(" LB:");
    Serial.println(left_intr);*/
    analogWrite(9, 0);
    analogWrite(10, 210);
    analogWrite(11, 0);
    analogWrite(12, 250);
    return isCompited;
  }
  else {
    Stop();
    //lastDistanceTravelled = meters;
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
  if(t > 2000){
  timeTo360 = t;
  h_intr++;
  loop_starts = true;
  delta = 9;//360 / (timeTo360 / lox_del);//360 / (timeTo360 / lox_del);
  rotations += 1;
  angle = 360;
  angle2 = 180;

  tmr_Lidar = millis();
  }
}
