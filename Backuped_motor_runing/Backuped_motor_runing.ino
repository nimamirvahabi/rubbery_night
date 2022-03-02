#include <Ultrasonic.h>
#include <Servo.h>
#include <tcs3200.h>
//#include <MPU6050.h>
//#include <Adafruit_PWMServoDriver.h>
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
//#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);
Ultrasonic ultrasonic(35, 37);
//MPU6050 mpu(0x68);
// define for multi
#define A 48
#define B 46
#define C 44
#define D 42
#define data A0
//#define for driver1
#define i1 22
#define i2 2//////////////////
//define for driver2
#define i3 26
#define i4 4
//define for driver3
#define i21 24
#define i22 3
//#define for driver4
#define i23 28
#define i24 5
//for led on shild
//#define led1 A7
//#define led2 A8
//sw dip on shild ;
int sw = A2;
class starter_vars {
  public:

    //srf
    int dis;
    //for tcrt
    int linee [16] ,  blk = 500 ;//blk=black kam     linee=for analog input tcrt
    int linemax [16];//for max input tcrt;
    bool lineD[16] ;//for digital input tcrt
    //for gy521
    bool dmpReady = false;
    uint8_t deviceStatus; //device status , 0 = success ,
    uint16_t packetSize; //expected DMP packet size (defult 42) -- ?
    uint16_t fifoCount; //count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO buffer storage
    //Quaternion q;           // [w, x, y, z]         quaternion container
    //VectorFloat gravity;    // [x, y, z]            gravity vector
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    bool ticket = 1;
    float yaw , output_pid, roll;
    int halat, degree;
    int Speed;
} robot_var;
void motor(int pwm1, int pwm2) {
  // pwm1 for left motors
  // pwm2 for right motors
  pwm1 += robot_var.output_pid;
  pwm2 -= robot_var.output_pid;
  if (pwm1 >= 254) pwm1 = 254;
  if (pwm2 >= 254) pwm2 = 254;
  if (pwm1 <= -254) pwm1 = -254;
  if (pwm2 <= -254) pwm2 = -254;
  if (pwm1 >= 0 && pwm2 >= 0)
  {
    digitalWrite(i1, 1);
    digitalWrite(i3, 1);
    analogWrite(i2, 255 - pwm1);
    analogWrite(i4, 255 - pwm1);


    digitalWrite(i21, 1);
    digitalWrite(i23, 1);
    analogWrite(i22, 255 - pwm2);
    analogWrite(i24, 255 - pwm2);

  }
  else if (pwm1 < 0 && pwm2 > -1)
  {
    digitalWrite(i1, 0);
    digitalWrite(i3, 0);
    analogWrite(i2,  -1 * ( pwm1));
    analogWrite(i4, -1 * ( pwm1));

    digitalWrite(i21, 1);
    digitalWrite(i23, 1);
    analogWrite(i22, 255 - pwm2);
    analogWrite(i24, 255 - pwm2);
  }
  else if (pwm1 > -1  && pwm2 < 0)
  {
    digitalWrite(i1, 1);
    digitalWrite(i3, 1);
    analogWrite(i2, 255 - (-(pwm2)));
    analogWrite(i4, 255 - (-(pwm2)));

    digitalWrite(i21, 0);
    digitalWrite(i23, 0);
    analogWrite(i22, pwm1);
    analogWrite(i24, pwm1);
  }
  else if (pwm1 < 0 && pwm2 < 0)
  {
    digitalWrite(i1, 0);
    digitalWrite(i3, 0);
    analogWrite(i2, 255 - (-(pwm2)));
    analogWrite(i4, 255 - (-(pwm2)));

    digitalWrite(i21, 0);
    digitalWrite(i23, 0);
    analogWrite(i22, -(pwm1));
    analogWrite(i24, -(pwm1));
  }
}
class tcrt {
  public:
    int Analog [16] ,  MaxBlack = 500 ;
    int Digital [16];
    int aPin, bPin, cPin, dPin;
    void mult(int addad)
    {
      digitalWrite(dPin, (addad / 8) % 2);
      digitalWrite(cPin, (addad / 4) % 2);
      digitalWrite(bPin, addad / 2 % 2);
      digitalWrite(aPin, addad % 2);
    }
    void read_tcrt () {
      mult (1 );
      Analog[1] = analogRead((data));
      mult (15 );
      Analog [2] = analogRead((data));
      mult (2 );
      Analog [3] = analogRead((data));
      mult (14);
      Analog [4] = analogRead((data));
      mult (6);
      Analog [5] = analogRead((data));
      mult (11);
      Analog[6] = analogRead((data));
      mult (12);
      Analog[7] = analogRead((data));
      mult (7);
      Analog[8] = analogRead((data));
      mult (0);
      Analog[9] = analogRead((data));
      mult (4);
      Analog[10] = analogRead((data));
      mult (3);
      Analog[11] = analogRead((data));
      mult (8);
      Analog[12] = analogRead((data));
      mult (5);
      Analog[13] = analogRead((data));
      mult (10);
      Analog[14] = analogRead((data));
      mult (9);
      Analog[15] = analogRead((data));
      for ( int y = 1; y < 16; y++) {
        if ( Analog[y] > MaxBlack ) {
          Digital[y] = 1;
          chop(Analog[y], 50);

        }//((linemax[y]*7)/10)
        else  {
          Digital[y] = 0;
          chop(Analog[y], 50);
        }

      }
      Serial.println();
    }
    void max_tcrt()
    {
      read_tcrt();
      for (int i = 1; i <= 16; i++)
      {
        if (robot_var.linemax[i] < Analog[i]) robot_var.linemax[i] = Analog[i];
      }
    }
    void linefollow(int Speed)
    {
      int t = 0;
      tcrt line_board;
      line_board.read_tcrt();
      if (line_board.Digital[8] == 1)motor(Speed, Speed);
      //  else if (lineD[7] == 1)motor(140, -140);
      //  else if (lineD[9] == 1)motor(-140, 140);
      else if (line_board.Digital[7] == 1) { // was 6
        motor(70, -70);
      }
      else if (line_board.Digital[9] == 1) { // was 10
        motor(-70, 70);
      }
      else if (line_board.Digital[6] == 1) {
        motor(150, -150);
      }
      else if (line_board.Digital[10] == 1) {
        motor(-150, 150);
      }
      else if (line_board.Digital[5] == 1)motor(200, -200);
      else if (line_board.Digital[11] == 1)motor(-200, 200);
      //  else if (lineD[12] == 1)motor(-200, 200);
      //  else if (lineD[3] == 1)motor(-255, 255);
      //  else if (lineD[13] == 1)motor(255, -255);
      // else if (lineD[1] == 1)motor(250, -250);
      //else if (lineD[15] == 1)motor(-250, 250);
      else
      {
        motor(70, 70);
        //digitalWrite(led1,1);
        //delay(5);
      }
      // digitalWrite(led1,0);
    }
} line_board;

//void gy521()
//{
//  mpu.resetFIFO();
//  //if DMP not ready don't do anything
//  if (!dmpReady)
//  {
//    if (ticket == 1)
//    {
//      Serial.println("MAIN LOOP: DMP disabled");
//      ticket = 0 ;
//    }
//    else
//      return;
//  }
//  else
//  {
//    if (fifoCount == 1024)
//    {
//      mpu.resetFIFO();
//      Serial.println("FIFO overflow");
//    }
//    else
//    {
//      //waiting until get enough
//      while (fifoCount < packetSize)
//        fifoCount = mpu.getFIFOCount();
//
//      mpu.getFIFOBytes(fifoBuffer, packetSize);
//      fifoCount -= packetSize ;
//
//      if (fifoCount > 2) {
//        ////// clear fifo buffer
//      }
//
//      mpu.dmpGetQuaternion(&q, fifoBuffer);
//      mpu.dmpGetGravity(&gravity, &q);
//      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//      yaw = ypr[0] * 180 / M_PI;
//      roll = ypr[2] * 180 / M_PI;
//      /////////////print ypr data/////////////
//      /*Serial.print(ypr[0] * 180/M_PI);  //yaw
//        Serial.print("\t");
//        Serial.print(ypr[1] * 180/M_PI);  //pitch
//        Serial.print("\t");
//        Serial.println(ypr[2] * 180/M_PI);  //roll
//      *////////////////////////////////////////
//    }
//  }
//}
void lcd_print(int x, int y, int object) {

  //  int adad1 = adad * 10;
  lcd.setCursor(x, y);
  lcd.print(object);
  //  if (adad1 < 0) {
  //    adad1 = adad1 * -1;
  //    lcd.print("-");
  //  }
  //  else {
  //    lcd.print("+");
  //  }
  //  lcd.print(adad1 / 1000);
  //  lcd.print((adad1 / 100) % 10);
  //  lcd.print((adad1 / 10) % 10);
  //  lcd.print(".");
  //  lcd.print(adad1 % 10);
}


void pid_rescue(int p , float input_gyro) {
  robot_var.output_pid = p * input_gyro;
}
void rescue() {

  //  gy521();
  if (robot_var.dis < 20 && robot_var.halat == 1)
  {
    motor(0, 0);
    delay(1000);
    robot_var.degree += 90;
    robot_var.halat = 0;
  }
  robot_var.degree = robot_var.degree % 360;
  if (robot_var.dis > 20) robot_var.halat = 1;
  int pid_input = robot_var.yaw - robot_var.degree;
  if (pid_input > 180) pid_input -= 360;
  if (pid_input < -180) pid_input += 360;
  pid_rescue(5 , pid_input);
  motor(100, 100);
  lcd_print(0, 0, robot_var.yaw);
  lcd_print(0, 1, robot_var.dis);
  lcd_print(7, 0, pid_input);
  srf();
}
void servo_setpwm(int angle, int servonum) {
  int Setup = 1;
  if (Setup == 1) {
    Setup = 0;
    delay(10);
  }
  //  pwm.setPWM(servonum, 0, (angle * 3.2) + 28);
}
void srf()
{
  robot_var.dis = ultrasonic.read();
  Serial.println(robot_var.dis);
  lcd_print(0, 1, robot_var.dis);
}
void ramp()
{
  //  gy521();
  line_board.linefollow(robot_var.Speed);
  if (robot_var.roll > 20)robot_var.Speed = 250;
  else if (robot_var.roll < -20) robot_var.Speed = 100;
  else robot_var.Speed = 175;
  lcd_print(70, 0, robot_var.roll);
}
void chop(double adad, int dopom  ) //dopom=delay of Print on monitor
{
  if (millis() % dopom < 1)
  {
    Serial.print(adad);
    Serial.print("/t");
  }
}
void setup() {

  Wire.begin();
  //MULTI
  pinMode(A, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(C, OUTPUT);
  pinMode(D, OUTPUT);
  line_board.aPin = A;
  line_board.bPin = B;
  line_board.cPin = C;
  line_board.dPin = D;

  // DRIVER 1
  pinMode(i1, OUTPUT);
  pinMode(i2, OUTPUT);
  pinMode(i3, OUTPUT);
  pinMode(i4, OUTPUT);

  //DRIVER 2
  pinMode(i21, OUTPUT);
  pinMode(i22, OUTPUT);
  pinMode(i23, OUTPUT);
  pinMode(i24, OUTPUT);
  //sw
  pinMode(sw, INPUT);
  //  SRF 05
  //  lcd16 * 2
  lcd.init();
  lcd.backlight();
  //  gy521
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 48; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  Serial.begin(9600);
  //  mpu.initialize();
  //  Serial.println(mpu.testConnection() ? F("MPU6050 connection test successed ") : F("MPU6050 connection test failed"));
  //  deviceStatus = mpu.dmpInitialize();
  /////////////Offsets/////////////
  //  mpu.setXGyroOffset(20);
  //  mpu.setYGyroOffset(-41);
  //  mpu.setZGyroOffset(48);
  //  mpu.setXAccelOffset(-3729);
  //  mpu.setYAccelOffset(3067);
  //  mpu.setZAccelOffset(1704);
  /////////////////////////////////
  if (robot_var.deviceStatus == 0)
  {
    Serial.println("DMP initialization success, now enable DMP for use");
    //    mpu.setDMPEnabled(true);

    //wait for first interrupt . currently just leave it false automatically
    robot_var.dmpReady = true;
    Serial.println("DMP is ready to use.");
    //    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    //ERROR! , device status !=0 when initializing DMP
    Serial.print("DMP initialization failed when using MPU6050 library:");
    if (robot_var.deviceStatus == 1)
      Serial.println(" intial memory load failed");
    else if (robot_var.deviceStatus == 2)
      Serial.println(" failed to update DMP configuration");
    else
    {
      Serial.print(" unknow error with code: ");
      Serial.println(robot_var.deviceStatus);
    }
  }
  //for led
  //  pinMode(led1, OUTPUT);
  //  pinMode(led2, OUTPUT);
  //for servo
  //  pwm.begin();'
  //  pwm.setPWMFreq(50);
}
void loop()
{
  motor(100, 100);
  //  line_board.linefollow(100);
  lcd_print(0, 0, line_board.Digital[5]);
  lcd_print(2, 0, line_board.Digital[6]);
  lcd_print(4, 0, line_board.Digital[7]);
  lcd_print(6, 0, line_board.Digital[8]);
  lcd_print(8, 0, line_board.Digital[9]);
  lcd_print(10, 0, line_board.Digital[10]);
  lcd_print(12, 0, line_board.Digital[11]);
  //  Serial.print("left ") , Serial.print(line_board.Digital[11]) , Serial.print("middle ") , Serial.print(line_board.Digital[8]), Serial.print("right ") , Serial.println(line_board.Digital[5]);
}
