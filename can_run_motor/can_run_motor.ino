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

int linee [16] ,  blk = 500 ;//blk=black kam     linee=for analog input tcrt
int linemax [16];//for max input tcrt;
bool lineD[16] ;//for digital input tcrt
void motor(int pwm1, int pwm2) {
  // pwm1 for left motors
  // pwm2 for right motors
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
void mult(int addad)
{
  digitalWrite(D, (addad / 8) % 2);
  digitalWrite(C, (addad / 4) % 2);
  digitalWrite(B, addad / 2 % 2);
  digitalWrite(A, addad % 2);
}
void chop(double adad, int dopom  ) //dopom=delay of Print on monitor
{
  if (millis() % dopom < 1)
  {
    Serial.print(adad);
    Serial.print("/t");
  }
}
void adadgiri_tcrt () {
  mult (1 );
  linee [1] = analogRead((data));
  mult (15 );
  linee [2] = analogRead((data));
  mult (2 );
  linee [3] = analogRead((data));
  mult (14);
  linee [4] = analogRead((data));
  mult (6);
  linee [5] = analogRead((data));
  mult (11);
  linee [6] = analogRead((data));
  mult (12);
  linee [7] = analogRead((data));
  mult (7);
  linee [8] = analogRead((data));
  mult (0);
  linee [9] = analogRead((data));
  mult (4);
  linee [10] = analogRead((data));
  mult (3);
  linee [11] = analogRead((data));
  mult (8);
  linee [12] = analogRead((data));
  mult (5);
  linee [13] = analogRead((data));
  mult (10);
  linee [14] = analogRead((data));
  mult (9);
  linee [15] = analogRead((data));
  for ( int y = 1; y < 16; y++) {
    if ( linee [y] > blk ) {
      lineD[y] = 1;
      chop(linee[y], 50);

    }//((linemax[y]*7)/10)
    else  {
      lineD[y] = 0;
      chop(linee[y], 50);
    }

  }
  Serial.println();


  /*Serial.print(linee[11]);
    Serial.print("\t");
    Serial.print(linee[12]);
    Serial.print("\t");
    Serial.print(linee[13]);
    Serial.print("\t");
    Serial.println();
    delay(200);
  */
}
void setup() {
  // put your setup code here, to run once:
  //MULTI
  pinMode(A, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(C, OUTPUT);
  pinMode(D, OUTPUT);

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
}

void loop() {
  // put your main code here, to run repeatedly:

}
