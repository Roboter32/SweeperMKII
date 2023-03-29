#include <IRremoteInt.h>
#include <IRremote.h>
#include <boarddefs.h>
#include "MOSFETMotor.h"
#include "NavVector.h"





#pragma region Config

//GPIO
#define BUMPER_L PB14
#define BUMPER_R PB15
MOSFETMotor lMotor(PA15, PA12, PB8);
MOSFETMotor rMotor(PB3, PB4, PB9);
IRrecv irrecv(PB6);


//Navigation
decode_results irresults;
bool cleaning = false;
int TargetSpeed = 200;//speed in mm/s
int PathWidth = 200;//cleaning path width in mm
int WheelBaseWidth = 250;
float Kp = 0.15;
float Ki = 0.0015;
float Kd = 1.55;
int NavigationMode = 0;//0-StraightLines, 1-WallFollow, 2-OutwardSpiral, 3-InwardSpiral
int BackOutMode = 0;//0-Offline, 1-Left, 2-Right
int TurnaroundDir = 0;//0-Left, 1-Right, add 1 to obtain BackOutMode
int PrecisionFactor = 1;
float MotionDistTotal = 0;
NavVector CourseDirError = NavVector(0,1);
NavVector buffCourseDirError = NavVector(0,1);
float CourseDistError = 0;
float CourseIntegral = 0;
float CourseDerivative =0;
uint8 courseHoldMode = 1;//0 - off, 1 - straight, 2 - idle rotational

#pragma endregion




void setup()
{
  afio_cfg_debug_ports(AFIO_DEBUG_NONE);

  pinMode(BUMPER_L, INPUT_PULLUP);
  pinMode(BUMPER_R, INPUT_PULLUP);
  pinMode(PC13, OUTPUT);
  pinMode(PB12, OUTPUT);
  pinMode(PB13, OUTPUT);
  digitalWrite(PB13, LOW);

  pinMode(PA8, INPUT);//LTachometer
  attachInterrupt(PA8, LeftPulsed, RISING);
  pinMode(PA11, INPUT);//RTachometer
  attachInterrupt(PA11, RightPulsed, RISING);

  Serial.begin(115200);

#if defined(__STM32F1__)

#ifdef F_CPU
  Serial.print(F("F_CPU: ")); // main Arduino clock
  Serial.println(F_CPU);
  Serial.println();
#endif

  Serial.print(F("SYSCLOCK: ")); // SYSCLOCK is defined in boarddefs.h
  Serial.println(SYSCLOCK);
  Serial.println();

  // irparams.blinkflag = 1; // option to test BLINKLED
#endif

  systick_enable();
  irrecv.enableIRIn(); // Start the receiver
  Serial.println(F("READY!"));
  Serial.println();


}

#pragma region RobotMotion


void Forward()
{
  lMotor.Forward();
  rMotor.Forward();
}

void Backward()
{
  lMotor.Backward();
  rMotor.Backward();
}

void Left()
{
  lMotor.Backward();
  rMotor.Forward();
}

void Right()
{
  lMotor.Forward();
  rMotor.Backward();
}

void Brake()
{
  lMotor.Brake();
  rMotor.Brake();
}




#pragma endregion

#pragma region MultiTasking



//Tachometers

uint32 STimerStart = 0;
float SdeltaT = 0;
int lTachCount = 0;
int rTachCount = 0;
float lSpeed;
float rSpeed;//measured speed in mm/s +- 0.5mm/s at 400mm/s

int lDist = 0;
int rDist = 0;
//float lDistTotal = 0;
//float rDistTotal = 0;

int lPWM = 0;
int rPWM = 0;
float lIntegral = 0;
float rIntegral = 0;
int lErrorPrior = 0;
int rErrorPrior = 0;

int lAvg[16];
int rAvg[16];
int lTargetSpeed = TargetSpeed;
int rTargetSpeed = TargetSpeed;

void MeasureSpeed()
{
  SdeltaT = systick_uptime() - STimerStart;
  if (SdeltaT >= 80)
  {
    //1. Measure speed
    lSpeed = lTachCount * 2587.5 / 800;// s/t = 207/0.080 = 2587.5
    lDist += lTachCount * 207 / 800;// 207mm per rotation, 800 pulses per rotation
    //Serial.print(lDist);
    lTachCount = 0;
    rSpeed = rTachCount * 2587.5 / 800;
    rDist += rTachCount * 207 / 800;
    //Serial.print(" ");
    //Serial.println(rDist);
    rTachCount = 0;

    //2. Smooth it out
    for (int i = 0; i < 15; i++)
    {
      lAvg[i] = lAvg[i + 1];
      rAvg[i] = rAvg[i + 1];
    }
    lAvg[15] = lSpeed;
    rAvg[15] = rSpeed;
    int lSum = 0;
    int rSum = 0;
    for (int i = 0; i < 16; i++)
    {
      lSum += lAvg[i];
      rSum += rAvg[i];
    }
    float lAvg = lSum / 16;
    float rAvg = rSum / 16;
    
    
    
    
    STimerStart = systick_uptime();
    SdeltaT = 0;
  }
}

void StabilizeSpeed()
{
  HoldCourse();

  float lError = lTargetSpeed - lAvg;
  lIntegral = lIntegral + (lErrorPrior + lError * 40);
  float lDerivative = (lError - lErrorPrior) / 80;
  lPWM = (Kp * lError) + (Ki * lIntegral) + (Kd * lDerivative);
  lErrorPrior = lError;
  if (lPWM < 0) lPWM = 0;
  if (lPWM > 255) lPWM = 255;
  lMotor.SetPWM(lPWM);

  float rError = rTargetSpeed - rAvg;
  rIntegral = rIntegral + (rErrorPrior + rError * 40);
  float rDerivative = (rError - rErrorPrior) / 80;
  rPWM = (Kp * rError) + (Ki * rIntegral) + (Kd * rDerivative);
  rErrorPrior = rError;
  if (rPWM < 0) rPWM = 0;
  if (rPWM > 255) rPWM = 255;
  rMotor.SetPWM(rPWM);
}






  



void LeftPulsed() {
  lTachCount++;
}
void RightPulsed() {
  rTachCount++;
}



//Navigation

uint32 BackOutCollisionT = 0;
uint32 CleanStartT = 0;
uint32 BackOutDelayStartT = 0;
uint8 BackOutCS = 0;
uint8 TurnAroundCS = 0;
uint8 MoveCS = 0;
int lMove = 0;
int rMove = 0;
int lTargetPulses = 0;
int rTargetPulses = 0;
float MoveRatio = 1;

uint8 WallFollowCS = 0;
uint32 WallFollowDelayStartT = 0;


void Move()
{
  switch (MoveCS)
  case 0:
    lTargetPulses = lMove * 800 / 207;
    rTargetPulses = rMove * 800 / 207;

    if (lTargetPulses > 0) lMotor.Forward();
    else if (lTargetPulses < 0) {lMotor.Backward(); lTargetPulses *= -1;}
    else lMotor.Brake();

    if (rTargetPulses > 0) rMotor.Forward();
    else if (rTargetPulses < 0) {rMotor.Backward(); rTargetPulses *= -1;}
    else rMotor.Brake();

    MoveRatio = rTargetPulses / lTargetPulses;

    MoveCS++;
  break;
  case 1:

  
}



void BackOut()
{
  if (BackOutCS == 0)
  {
    courseHoldMode = 1;
    lTargetSpeed = 75;
    rTargetSpeed = 75;
    
    
    //RANDOM NAVIGATION
    /*if (systick_uptime() - BackOutCollisionT <= 4000)
    {
      PrecisionFactor = 2;//Engage PrecisionMode
      BackOutMode = 1;//Force left turn
      int r = rand();
      r = r % 10;
      if (r < 2) NavigationMode = 1;//Engage WallFollow
    }
    else if (NavigationMode == 0) PrecisionFactor = 1; //Disengage PrecisionMode
    */
    BackOutCollisionT = systick_uptime();
    Backward();
    BackOutDelayStartT = systick_uptime();
    BackOutCS++;
  }

  //if (BackOutCS == 1 && systick_uptime() - BackOutDelayStartT >= 750 / PrecisionFactor) BackOutCS++;//RN

  if (BackOutCS == 1 && MotionDistTotal < 100)
  {
    if (MotionDistTotal > 75)
    {
      lTargetSpeed = 50;
      rTargetSpeed = 50;
    }
  }

  if (BackOutCS == 2)
  {
    lTargetSpeed = 100;
    rTargetSpeed = 100;
    if (BackOutMode == 1) Left();
    if (BackOutMode == 2) Right();
    BackOutDelayStartT = systick_uptime();
    BackOutCS++;
  }

  if (BackOutCS == 3 && systick_uptime() - BackOutDelayStartT >= 1000 / PrecisionFactor) BackOutCS++;

  if (BackOutCS == 4)
  {
    if (NavigationMode == 0)
    {
      lTargetSpeed = TargetSpeed;
      rTargetSpeed = TargetSpeed;
    }
    Forward();
    BackOutCS = 0;
    BackOutMode = 0;
    WallFollowCS = 0;//reset WallFollow timer in case it's running
  }


}

void FollowWall()
{
  if (WallFollowCS == 0)
  {
    WallFollowDelayStartT = systick_uptime();
    WallFollowCS++;
  }
  if (WallFollowCS == 1 && systick_uptime() - WallFollowDelayStartT >= 500)
  {
    WallFollowCS--;
    BackOutMode = 2;//Set direction to right
    BackOutCS = 2;//Skip the reverse and the forced left
    int r = rand();
    r = r % 10;
    if (r == 0) NavigationMode = 0;
  }
}

void HoldCourse()
{
    if (courseHoldMode == 0)//off
    {
        CourseDirError.x = 0;
        CourseDirError.y = 1;
        buffCourseDirError.x = 0;
        buffCourseDirError.y = 1;
        CourseDistError = 0;
        CourseIntegral = 0;
        CourseDerivative = 0;
        //lTargetSpeed = TargetSpeed;
        //rTargetSpeed = TargetSpeed;
        lDist = 0;
        rDist = 0;

    }
    if (courseHoldMode == 1)//straight
    {
        //1. Update course deviation data
        NavVector lMotionV = CourseDirError, rMotionV = CourseDirError, MotionV = CourseDirError;
        lMotionV.SetLength(lDist);
        rMotionV.SetLength(rDist);
        float MotionDist = lDist + rDist / 2;
        MotionDistTotal += MotionDist;
        MotionV.SetLength(MotionDist);
        CourseDistError += MotionV.x;
        NavVector SidewaysV = CourseDirError, ForwardV = CourseDirError;
        SidewaysV.TurnRight();
        SidewaysV.SetLength(WheelBaseWidth);
        ForwardV.SetLength(rDist - lDist);
        buffCourseDirError = CourseDirError;
        CourseDirError.x = ForwardV.x + SidewaysV.x;
        CourseDirError.y = ForwardV.y + SidewaysV.y;
        CourseDirError.TurnLeft();
        CourseDirError.SetLength(1);
        lDist = 0;
        rDist = 0;

        //2. Use this data to do corrections
        CourseIntegral += CourseDirError.x * 80;
        CourseDerivative += CourseDirError.x - buffCourseDirError.x;
        lTargetSpeed = TargetSpeed - (150 * CourseDirError.x) - (0 * CourseIntegral) - (0.1 * CourseDerivative) - (CourseDistError / 10);
        rTargetSpeed = TargetSpeed + (150 * CourseDirError.x) + (0 * CourseIntegral) + (0.1 * CourseDerivative) + (CourseDistError / 10);

        

    }
    if (courseHoldMode == 2)//idle rotational
    {
      CourseDirError.x = 0;
      CourseDirError.y = 1;
      buffCourseDirError.x = 0;
      buffCourseDirError.y = 1;
      CourseDistError = 0;
      CourseIntegral = 0;
      CourseDerivative = 0;
    }
}

void TurnAround()
{
  Serial.print(MotionDistTotal);
  Serial.print(" ");
  Serial.print(lSpeed);
  Serial.print(" ");
  Serial.println(rSpeed);
  
  
  if (TurnAroundCS == 0)
  {
    courseHoldMode = 1;
    lTargetSpeed = 75;
    rTargetSpeed = 75;
    Backward();
    MotionDistTotal = 0;
    TurnAroundCS++;
  }


  if (TurnAroundCS == 1)
  {    
    if(MotionDistTotal >= 100)
    {
      Brake();
      BackOutDelayStartT = systick_uptime();
      courseHoldMode = 0;
      TurnAroundCS++;
    }
    else if(MotionDistTotal > 75)
    {
      lTargetSpeed = 25;
      rTargetSpeed = 25;
    }
    else if (MotionDistTotal > 50)
    {
      lTargetSpeed = 50;
      rTargetSpeed = 50;
    }
  }

  if (TurnAroundCS == 2 && systick_uptime() - BackOutDelayStartT > 50)
  {
    MotionDistTotal = 0;
    courseHoldMode = 1;
    if(BackOutMode == 1)Left();
    if(BackOutMode == 2)Right();
    TurnAroundCS++;
  }

  if (TurnAroundCS == 3)
  {
    if(MotionDistTotal >= 196)
    {
      Brake();
      BackOutDelayStartT = systick_uptime();
      courseHoldMode = 0;
      TurnAroundCS++;
    }
    else if(MotionDistTotal > 150)
    {
      lTargetSpeed = 25;
      rTargetSpeed = 25;
    }
    else if (MotionDistTotal > 100)
    {
      lTargetSpeed = 50;
      rTargetSpeed = 50;
    }
  }

  if (TurnAroundCS == 4 && systick_uptime() - BackOutDelayStartT > 50)
  {
    MotionDistTotal = 0;
    courseHoldMode = 1;
    Forward();
    TurnAroundCS++;
  }

  if (TurnAroundCS == 5)
  {
    if(MotionDistTotal >= 200)
    {
      Brake();
      BackOutDelayStartT = systick_uptime();
      courseHoldMode = 0;
      TurnAroundCS++;
    }
    else if(MotionDistTotal > 150)
    {
      lTargetSpeed = 25;
      rTargetSpeed = 25;
    }
    else if (MotionDistTotal > 100)
    {
      lTargetSpeed = 50;
      rTargetSpeed = 50;
    }
  }

  if (TurnAroundCS == 6 && systick_uptime() - BackOutDelayStartT > 50)
  {
    MotionDistTotal = 0;
    courseHoldMode = 1;
    if(BackOutMode == 1)Left();
    if(BackOutMode == 2)Right();
    TurnAroundCS++;
  }

  if (TurnAroundCS == 7)
  {
    if(MotionDistTotal >= 196)
    {
      Brake();
      BackOutDelayStartT = systick_uptime();
      courseHoldMode = 0;
      TurnAroundCS++;
    }
    else if(MotionDistTotal > 150)
    {
      lTargetSpeed = 25;
      rTargetSpeed = 25;
    }
    else if (MotionDistTotal > 100)
    {
      lTargetSpeed = 50;
      rTargetSpeed = 50;
    }
  }

  if (TurnAroundCS == 8 && systick_uptime() - BackOutDelayStartT > 50)
  {
    MotionDistTotal = 0;
    courseHoldMode = 1;
    if (NavigationMode == 0)
    {
      lTargetSpeed = TargetSpeed;
      rTargetSpeed = TargetSpeed;
    }
    Forward();
    TurnAroundCS = 0;
    BackOutMode = 0;
  }

  
}



#pragma endregion


void loop()
{
  if (cleaning)
  {
    
    digitalWrite(PC13, LOW);
    digitalWrite(PB12, HIGH);

    //RANDOM NAVIGATION
    /*
    if (BackOutMode == 0)
    {
        if(digitalRead(BUMPER_L))
        {
          BackOutMode = 2;
        }
    
        if(digitalRead(BUMPER_R))
        {
          BackOutMode = 1;
        }
        Forward();
        if (NavigationMode == 1)FollowWall();
        if (NavigationMode == 0)courseHoldMode = 1;
        if (NavigationMode != 0)courseHoldMode = 0;
    }
    else
    {      

      courseHoldMode = 0;
      BackOut();
    }
    
    */

    //SMART NAVIGATION


    if (BackOutMode == 0)
    {
      if (digitalRead(BUMPER_L) || digitalRead(BUMPER_R))
      {
        BackOutMode = TurnaroundDir + 1;
      }


      Forward();
      if (NavigationMode == 1)FollowWall();
    }
    else
    {
      TurnAround();
    }

    
    
    
    
    StabilizeSpeed();

  }
  
  else
  {
    digitalWrite(PC13, HIGH);
    digitalWrite(PB12, LOW);

    lMotor.SetPWM(0);
    rMotor.SetPWM(0);
  }

  if (irrecv.decode(&irresults)) {
    Serial.println(irresults.value);
    //dump(&results); not needed, not defined

    if (irresults.value == 1124692103 || irresults.value == 89149445 || irresults.value == 3737721253 || irresults.value == 3011669255)//play button
    {
      cleaning = !cleaning;
      if (cleaning)
      {
          CleanStartT = systick_uptime();
          lIntegral = 0;
          rIntegral = 0;
      }
    }
    delay(1000);
    irrecv.resume(); // Receive the next value
  }
}
