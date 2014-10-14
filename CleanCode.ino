#include <math.h>
#include "DualVNH5019MotorShield.h"
#include "Motor.h"
#include "PinChangeInt.h"

#define L A3
#define LF A0
#define F A1
#define RF A2
#define RL A4
#define RS A5

DualVNH5019MotorShield md;

const float DISTANCE_LR_SENSOR = 17.6;
int getSensor = 1;

//note: setSpeeds is inversed. speed 1 is right speed 2 is left


void setup(){
  md.init();

  Serial.begin(115200); 

  PCintPort::attachInterrupt(motor1_A, Motor1EncoderA, CHANGE);
  PCintPort::attachInterrupt(motor1_B, Motor1EncoderB, CHANGE);
  PCintPort::attachInterrupt(motor2_A, Motor2EncoderA, CHANGE);
  PCintPort::attachInterrupt(motor2_B, Motor2EncoderB, CHANGE);
}

/* ---------------------------------- Encoder Interrupt Handler (Motor1EncoderA,Motor1EncoderB,Motor2EncoderA,Motor2EncoderB) ----------------------------------*/

void Motor1EncoderA(){
  motor1_Bnew ^ motor1_Aold ? motor1_encoder++ : motor1_encoder--;
  motor1_Aold = digitalRead(motor1_A);
}

void Motor1EncoderB(){
  motor1_Bnew = digitalRead(motor1_B);
  motor1_Bnew ^ motor1_Aold ? motor1_encoder++ : motor1_encoder--;

}

void Motor2EncoderA(){
  motor2_Bnew ^ motor2_Aold ? motor2_encoder++ : motor2_encoder--;
  motor2_Aold = digitalRead(motor2_A);
}

void Motor2EncoderB(){
  motor2_Bnew = digitalRead(motor2_B);
  motor2_Bnew ^ motor2_Aold ? motor2_encoder++ : motor2_encoder--;
}


//motor1=left,motor2=right
//move distance 1 ----- encoder 1125     actual distance 10 cm
//move distance 10 ------ encoder 11984    actual distance 99.3cm

void loop(){

  /* ---------------------------------- Serial Communication Handler ----------------------------------*/
  char command[10];
  int index = 0;
  char newChar;
  int param = 0;
  while (1){

    if (Serial.available()){
      newChar = Serial.read();
      command[index] = newChar;
      index++;
      if (newChar == '|'){
        index = 1;
        break;
      }
    }  
  }

  while (command[index] != '|'){
    param = param * 10 + command[index] - 48;
    index++;
  }

  char movement = command[0];
  switch ( movement ) {
  case 'W':
    {
      if (param == 0){
        moveForward(1);
      }
      else{
        moveForward(param);
      }    
      break;
    }
  case 'S':
    {
      if (param == 0){
        moveBackward(1);
      }
      else{
        moveBackward(param);
      }       
      break;
    }
  case 'A':
    {
      if (param == 0){
        turnLeft(90);
      }
      else{
        turnLeft(param);
      }       
      break;
    }
  case 'D':
    {
      if (param == 0){
        turnRight(90);
      }
      else{
        turnRight(param);
      }
      break;
    }
  case'E':
    {
      test();
      break;
    }
  case 'Q': 
    {
      wallAlignment();
      break;
    }
  case 'X':
    {
      getSensor = 1;
      exportSensors();
      break;
    }
  case 'R':
    {
      getSensor = 0;
    }

  case 'a':
    {
      delay(1000);
      checkList3();
      break;
    }
  default:
    {
      break;
    }
    memset(command,0,sizeof(command));

  }
  delay(500);
}

/* ---------------------------------- Testing and Later Stage Exploration ----------------------------------*/


int test(){

  delay(1000);
      int i = 0;
      while (i<12){
        i++;
        moveForward(1);
      }
      int j = 0;
      while (j<12){
        j++;
        moveBackward(1);
      }
 // delay(1000);
  //md.setSpeeds(90,100);
//  while(1){
//    delay(500);
//    Serial.println(calculateDistance(200));
//    exportSensors();
//    Serial.println();
//  }

  // moveForward(15);
  //  moveBackward(15);
  //  moveForward(10);
  //  moveBackward(10);

  //checkListExtension();
  //while (1){
  ////Serial.print(calculateDistance(LF));
  ////Serial.print("     ");
  ////Serial.println(calculateDistance(200));
  //}

}


/* ---------------------checklist-----------------*/

void checkList3(){
  int distance;
  while(1){
    moveForward(1);
    distance = calculateDistance(F);
    if (distance==1){
      break;
    }
  }  
  turnRight(90);
  moveForward(3);
  turnLeft(90);
  moveForward(4);
  turnLeft(90);
  moveForward(3);
  turnRight(90);
  moveForward(3); 
}

void checkListExtension(){

  int counter = 0;

  while(1)
    moveForward(1);
  counter++;
  // if (counter)

}

//void checkListExtension(){
//  PWM_Mode_Setup();
//  
//  while(1){
//    moveForwardExtension();
//    turnRightExtension();
//  }
//  
//  md.init();
//}


/* ---------------------------------- Standard Motor Function Call (Forward,Backward,Left,Right,Stop) ----------------------------------*/



int moveForward(int distance){

  motor1_encoder=0;
  motor2_encoder=0;  



  ///////////////////////
  // int target_Distance = ((2249/(6*3.142))*(distance*10));
  //  int target_Distance = 1192.9768*distance;
  // int target_Distance = 1150*distance;
  int target_Distance = 1170.5*distance;
  // int left_offset=10.8;  //halfly charged white powerbank done
  int left_offset=265;    //fully charged 
  if (distance == 1)
    //left_offset = 25.8;
    left_offset = 29;
  // left_offset = 0;
  /////////////////////

  int count=0;
  int pwm1=300, pwm2=300; 
  int output=0;
  int LeftPosition,RightPosition;



  while(1){
    LeftPosition = -motor1_encoder;    //hardcoded
    RightPosition = -motor2_encoder;  

    //Acceleration
    if(LeftPosition <=100){
      pwm1 = 100;
      pwm2 = 100;
    } 
    else if(LeftPosition >100 && LeftPosition <=300){
      pwm1 = LeftPosition;
      pwm2 = RightPosition;
    } 
    else {
      pwm1 = 300;
      pwm2 = 300;
    }   

    if(LeftPosition >= target_Distance-70){
      md.setBrakes(400, 400);
      delay(100);
      md.setBrakes(0, 0);
      break;
    }

    if(distance == 1){

      if(LeftPosition >= (target_Distance-70-200) && LeftPosition <= (target_Distance+100)){
        pwm1 = target_Distance-70-LeftPosition+100;
        pwm2 = target_Distance-70-LeftPosition+100;
      }
    }



    output = pidControlForward(motor1_encoder,motor2_encoder);
    md.setSpeeds(pwm1-output+left_offset, pwm2+output);

    //   Serial.print(" motor1_encoder: ");   Serial.print(motor1_encoder);
    //   Serial.print(" motor2_encoder: ");   Serial.print(motor2_encoder);
    //   Serial.print("\n");

  }
  if(getSensor)
    exportSensors();
}

void moveForwardExtension(){

  //  motor1_encoder=0;
  //  motor2_encoder=0;  
  //  int target_Distance = ((2249/(6*3.142))*(distance*10));
  // // int left_offset=10.8;  //halfly charged white powerbank done
  //  int left_offset=300;    //fully charged 
  //  if (distance == 1)
  //    left_offset = 20;
  //
  //  int count=0;
  //  int pwm1=300, pwm2=300; 
  //  int output=0;
  //  int LeftPosition,RightPosition;
  //  
  int counter = 0;

  while(counter < 800){
    counter++;
    if (0<calculateDistance(200)<=5){
      turnRight(90);
      moveForwardExtension();
      return;
    }

    //     LeftPosition = -motor1_encoder;    //hardcoded
    //     RightPosition = -motor2_encoder;  

    //Acceleration

      //      pwm1 = 400;
    //      pwm2 = 400;
    //      
    //   
    //   if(LeftPosition >= target_Distance-70){
    //     
    //   }



    //   if(distance == 1){
    //
    //   if(LeftPosition >= (target_Distance-70-200) && LeftPosition <= (target_Distance+100)){
    //     pwm1 = target_Distance-70-LeftPosition+100;
    //     pwm2 = target_Distance-70-LeftPosition+100;
    //     }
    //   }



    //   output = pidControlForward(motor1_encoder,motor2_encoder);
    // md.setSpeeds(pwm1-output+left_offset, pwm2+output);
    md.setSpeeds(400, 400);
    delay(1);
  }

  //     md.setBrakes(400, 400);
  //     delay(100);
  //     md.setBrakes(0, 0);
  //     break;
}




int moveBackward(int distance){

  motor1_encoder=0;
  motor2_encoder=0;  
  int target_Distance = ((2249/(6*3.142))*(distance*10));
  int left_offset=10.5;
  int count=0;
  int pwm1=300, pwm2=300; 
  int output=0;
  int LeftPosition,RightPosition;

  while(1){
    LeftPosition =  motor1_encoder;   //hardcoded
    RightPosition =  motor2_encoder;  

    //Acceleration
    if(LeftPosition <=100){
      pwm1 = 100;
      pwm2 = 100;
    } 
    else if(LeftPosition >100 && LeftPosition <=300){
      pwm1 =  LeftPosition;
      pwm2 =  RightPosition;
    } 
    else {
      pwm1 = 300;
      pwm2 = 300;
    }   

    if(LeftPosition >= target_Distance-70){
      //  Serial.println("slowing down");

      md.setBrakes(400, 400);
      delay(100);
      md.setBrakes(0, 0);
      break;
    }
    if(distance == 1){
      if(LeftPosition >= (target_Distance-70-200) && LeftPosition <= (target_Distance+100)){
        pwm1 = target_Distance-70-LeftPosition+100;
        pwm2 = target_Distance-70-LeftPosition+100;
      }
    }


    output = pidControlBack(motor1_encoder,motor2_encoder);
    md.setSpeeds(-(pwm1-output)+left_offset, -(pwm2+output));
  } 
  if(getSensor)
    exportSensors();
}



int turnLeft(int angle){
  motor1_encoder=0;
  motor2_encoder=0;  
  int pwm1=300, pwm2=300, output=0,LeftPosition,RightPosition;
  int target_Angle;


  if (angle <= 5){
    target_Angle = angle * 12.5;   
    pwm1=150;
    pwm2=150;
  }   
  else if ((angle > 5) && (angle <= 15))
    target_Angle = angle * 18;
  else if ((angle > 15) && (angle <= 30))
    target_Angle = angle * 17.5;
  else if ((angle > 30) && (angle <= 45))
    target_Angle = angle * 14.55;

  //////////////////
  else if ((angle > 45) && (angle <= 90))   
    // target_Angle = angle * 17.85;
    // target_Angle = angle * 17.84;   
    target_Angle = angle * 17.81;    
  ////////////////

  else  
    target_Angle = angle * 18.2;

  while(1){
    LeftPosition =  motor1_encoder; 
    RightPosition =  motor2_encoder; 

    if((LeftPosition >= target_Angle - 70)&&(angle > 8)){  //used to be rightPosition
      md.setBrakes(400, 400);
      delay(100);
      md.setBrakes(0, 0);
      break;
    }

    if((LeftPosition >= target_Angle - 5)&&(angle < 9)){
      md.setBrakes(400, 400);
      delay(100);
      md.setBrakes(0, 0);
      break;
    }

    //  output = pidControlLeftAndRight(motor1_encoder,motor2_encoder); 
    output = pidControlLeftAndRight(LeftPosition,RightPosition); //hardcoded  

    //  md.setSpeeds(-(pwm1+output), pwm2-output);  
    md.setSpeeds((pwm1+output), -(pwm2-output)); //hardcoded

  } 

  if (angle == 90 && getSensor ==1)
    exportSensors();
}

int turnRight(int angle){
  motor1_encoder=0;
  motor2_encoder=0;  
  int pwm1=300, pwm2=300, output=0,LeftPosition,RightPosition;
  int target_Angle;
  // int target_Angle = angle * 18.0555;

  if (angle <= 15){
    target_Angle = angle * 17;  // 15 BASICALLY OKAY
    pwm1=150;
    pwm2=150;
  }
  else if ((angle > 15) && (angle <= 30))
    target_Angle = angle * 17;  // 30 BASICALLY OKAY
  else if ((angle > 30) && (angle <= 45))
    target_Angle = angle * 14.55;


  ///////////////////////
  else if ((angle > 45) && (angle <= 90))  
    //target_Angle = angle * 17.715;  // 90 OKAY on old arena
    // target_Angle = angle * 17.8;      // fully charged
    // target_Angle = angle * 17.82;
    target_Angle = angle * 17.74;

  //////////////////////
  else if ((angle > 90) && (angle <= 360))  
    target_Angle = angle * 17.9;
  else if ((angle > 360) && (angle < 720))  
    target_Angle = angle * 18.0;
  else if ((angle >= 720) && (angle <= 1080)) //OKAY
    target_Angle = angle * 18.1;
  else  
    target_Angle = angle * 18.1;

  while(1){

    LeftPosition =  motor1_encoder;
    RightPosition =  motor2_encoder;  

    //  output = pidControlLeftAndRight(motor1_encoder,motor2_encoder);
    output = pidControlLeftAndRight(LeftPosition,RightPosition); //hardcoded


    //   md.setSpeeds(pwm1-output, -(pwm2+output));
    md.setSpeeds(-(pwm1-output), pwm2+output);

    if((RightPosition >= target_Angle - 70)&&(angle > 8)){ //used to be left position
      md.setBrakes(400, 400);
      delay(100);
      md.setBrakes(0, 0);
      break;
    }


    if((RightPosition >= target_Angle - 5)&&(angle < 9)){
      md.setBrakes(400, 400);
      delay(100);
      md.setBrakes(0, 0);
      break;
    }
  }

  if (angle == 90 && getSensor == 1)
    exportSensors(); 
}


int turnRightExtension(){
  //  Serial.print("turning");
  md.setSpeeds(100,400);
  delay(900);
  md.setSpeeds(400, 400);
  delay(100);
}



/* ---------------------------------- PID Control ----------------------------------*/


int pidControlForward(int LeftPosition, int RightPosition){
  int error,prev_error,pwm1=255,pwm2=255;
  float integral,derivative,output;
  float Kp = 0.75;  //0-0.1
  float Kd = 1.65;  //1-2
  float Ki = 0.65;  //0.5-1

  error = LeftPosition - RightPosition;
  integral += error;
  derivative = (error - prev_error);
  output = Kp*error + Ki * integral + Kd * derivative;
  prev_error = error;

  pwm1=output;
  return pwm1;
}


int pidControlBack(int LeftPosition, int RightPosition){
  int error,prev_error,pwm1=255,pwm2=255;
  float integral,derivative,output;
  float Kp = 0.75;  //0-0.1
  float Kd = 1.65;  //1-2
  float Ki = 0.65;  //0.5-1

  error = RightPosition - LeftPosition;
  integral += error;
  derivative = (error - prev_error);
  output = Kp*error + Ki * integral + Kd * derivative;
  prev_error = error;


  pwm1=output; 
  return pwm1;
}

int pidControlLeftAndRight(int LeftPosition, int RightPosition){
  int error,prev_error,pwm1=255,pwm2=255;
  float integral,derivative,output;
  float Kp = 0.75;  //0-0.1
  float Kd = 1.65;  //1-2
  float Ki = 0.6;  //0.5-1

  error = RightPosition + LeftPosition;
  integral += error;
  derivative = (error - prev_error);
  output = Kp*error + Ki * integral + Kd * derivative;
  prev_error = error;

  pwm1=output;
  return pwm1;
}


/* ---------------------------------- Sensor Functions ----------------------------------*/
int irSensorFeedback (int sensorIndex){
  return analogRead(sensorIndex);
}

float calculateDistance(int sensorIndex){
  int numLoop = 5;
  int adc = averageFeedback(30,15,sensorIndex);
  float distance;
  float voltFromRaw;

  switch(sensorIndex){
  case LF: 
    distance = 6228.0 / (adc +15.5) - 3;
    break;
  case F:
    PWM_Mode_Setup();
    distance = 1;
    for(int i=0; i<4; i++) {
      int j = PWM_Mode();
      if(j > 10) {
        distance = -1;
        //distance = j;
        break;
      } 
    }
    md.init();
    break;

  case 100:
    distance = PWM_Mode();       
    break;

  case 200:
    PWM_Mode_Setup();
    distance = PWM_Mode();
    md.init(); 
    break;

  case RF: 
    distance = 5528.0 / (adc +21.0) - 1.0;
    break;
  case L: 
    distance = 6088 / (adc  + 7) - 1;
    break;
  case RL: 
    voltFromRaw = map(adc, 0, 1023, 0, 5000);
    if(adc >= 208) // or 303
      distance=61.573*pow(voltFromRaw/1000, -1.1068) + 1;
    else if(adc < 208 && adc >= 165)
      distance=61.573*pow(voltFromRaw/1000, -1.1068) + 1;
    else
      distance = -1;
    break;
  case RS: 
    distance = 6088.0 / (adc +7) - 2;
    break;
  }
  return distance;
}

void quickSort(int x[32],int first,int last){
  int pivot,j,temp,i;
  if(first<last){
    pivot=first;
    i=first;
    j=last;

    while(i<j){
      while(x[i]<=x[pivot]&&i<last)
        i++;
      while(x[j]>x[pivot])
        j--;
      if(i<j){
        temp=x[i];
        x[i]=x[j];
        x[j]=temp;
      }
    }

    temp=x[pivot];
    x[pivot]=x[j];
    x[j]=temp;
    quickSort(x,first,j-1);
    quickSort(x,j+1,last);

  }
}

int averageFeedback(int in, int out, int pin){
  int x[in];
  int i;
  int sum = 0;
  int start = (in - out)/2;
  int average;
  for(i=0;i<in;i++){
    x[i] = irSensorFeedback(pin);
  }
  quickSort(x, 0, in-1);
  for(i = start; i < start+out; i++){
    sum = sum + x[i];
  }
  average = sum/out;
  return average;
}

/* ---------------------------------- Sending Out Sensor Data ----------------------------------*/

void exportSensors(){
  Serial.print(calculateDistance(L));
  Serial.print(",");
  Serial.print(calculateDistance(LF));  
  Serial.print(",");
  Serial.print(calculateDistance(F));
  Serial.print(",");
  Serial.print(calculateDistance(RF));
  Serial.print(",");
  Serial.print(calculateDistance(RL));
  Serial.print(",");
  Serial.print(calculateDistance(RS));
  Serial.print("|");
}


/* ---------------------------------- Left Wall Alignment ----------------------------------*/
void wallAlignment(){  
  alignAngel();
  delay(300);
  alignDistance();
  delay(300);
  alignAngel();
  delay(300);
}

void alignAngel(){
  int offset = 0;
  int frontLeftFeedback = averageFeedback( 30, 15, LF);
  int frontRightFeedback = averageFeedback(30, 15, RF);
  int difference = frontLeftFeedback - frontRightFeedback - offset;

  //  Serial.print(frontLeftFeedback);
  //  Serial.print("   ");
  //  Serial.print(frontRightFeedback);
  //  Serial.print("   ");
  //  Serial.print(difference);
  //  Serial.print("   ");
  //  Serial.println();

  while((difference > 5)||(difference < -5)){
    if (difference > 0)
      turnLeft(2);
    //md.setSpeeds(-150, 150);
    else if (difference < 0)
      turnRight(2);
    //md.setSpeeds(150, -150);

    frontLeftFeedback = averageFeedback( 30, 15, LF);
    frontRightFeedback = averageFeedback( 30, 15, RF);
    difference = frontLeftFeedback - frontRightFeedback - offset;
  }
  //md.setBrakes(150, 150);
}


void alignDistance(){
  //  boolean near = 0;
  //  while((calculateDistance(200) !=6)&&(calculateDistance(200)>0)){
  // // Serial.println(calculateDistance(200));
  //  if (calculateDistance(200) > 6){
  //     md.setSpeeds(200,200);
  //     delay(50);
  //     md.setBrakes(400, 400);
  //     delay(100);
  //     md.setBrakes(0, 0);
  //  }
  //  else if (calculateDistance(200) < 6){
  //     md.setSpeeds(-200,-200);
  //     delay(50);
  //     md.setBrakes(400, 400);
  //     delay(100);
  //     md.setBrakes(0, 0);
  //     near = 1;
  //  }
  int near = 0;
  PWM_Mode_Setup();
  while(1) {
    if (calculateDistance(100) > 5)
      md.setSpeeds(70, 78);
    else if(calculateDistance(100) < 5){
      md.setSpeeds(-100,-100);
      delay(70);
      md.setBrakes(400, 400);
      delay(100);
      md.setBrakes(0, 0);
      near = 1;
    }
    else if(calculateDistance(100) == 5){
   //   Serial.println(calculateDistance(100));
      break;
    }
  }
 // Serial.println("break");
  if (near){
    md.setSpeeds(-70,-70);
    delay(200);
  }
  md.setBrakes(400, 400);
  delay(50);
  md.setBrakes(0, 0);
  md.init();
}

/* ------- Ultrasonic --------*/
int URPWM = 12; // PWM Output 0√î¬∫√ß25000US√î¬∫√•Every 50US represent 1cm
int URTRIG= 6; // PWM trigger pin


uint8_t EnPwmCmd[4]={
  0x44,0x02,0xbb,0x01};    // distance measure command

void PWM_Mode_Setup(){ 
  pinMode(URTRIG,OUTPUT);                     // A low pull on pin COMP/TRIG
  digitalWrite(URTRIG,HIGH);                  // Set to HIGH

    pinMode(URPWM, INPUT);                      // Sending Enable PWM mode command
}

int PWM_Mode(){                              // a low pull on pin COMP/TRIG  triggering a sensor reading
  unsigned int Distance=0;
  digitalWrite(URTRIG, LOW);
  digitalWrite(URTRIG, HIGH);               // reading Pin PWM will output pulses

  unsigned long DistanceMeasured  = pulseIn(URPWM,LOW);//a /30;

  while(DistanceMeasured>=10000){              // the reading is invalid.
    digitalWrite(URTRIG, LOW);
    digitalWrite(URTRIG, HIGH);
    DistanceMeasured  = pulseIn(URPWM,LOW);
    //Distance = -1;    
  }

  Distance=DistanceMeasured/50;           // every 50us low level stands for 1cm

  /*Serial.print("Distance=");
   Serial.print(Distance);
   Serial.println("cm");*/
  return Distance;
}



