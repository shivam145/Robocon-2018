#include<QTRSensors1.h>
#include<QTRSensors2.h>
#include<QTRSensors3.h>
#include<QTRSensors4.h>

#define KP        0.09
#define KD        0
#define SPEED_BASE    140
#define SPEED_BASE_CROSS 90              //During the line crossing speed
#define SPEED_MIN   0 
#define SPEED_MAX   200
#define NUM_SENSORS   8
#define TIMEOUT     2500

#define Start     14
#define DIR1      10                     //7 //Front side Motor direction
#define DIR2      12                     //4 //Back side Motor direction
#define DIR3      8                      //2 //Left side Motor direction
#define DIR4      6                      //3 //Right side Motor direction

#define PWM1      9                      //10 //Front side Motor PWM
#define PWM2      11                     //11 //Back side Motor PWM
#define PWM3      7                      //5 //Left side Motor PWM
#define PWM4      5                      //6  //Right side Motor PWM

#define TZ_1_STPR1 18                    //30 //TZ1 stopper inp1
#define TZ_1_STPR2 19                    //31 //TZ1 stopper inp2
#define TZ_2_STPR1 3                     //3//32 //TZ2 stopper inp1
#define TZ_2_STPR2 4                     //4//33 //TZ2 stopper inp2
#define TZ_3_STPR1 42                    //42//34 //TZ3 stopper inp1
#define TZ_3_STPR2 44                    //44//35 //TZ3 stopper inp2

#define TZ_1_ply_PMW 55                  //36 //TZ1 Pulley PWM
#define TZ_2_ply_PMW 58                  //37 //TZ2 Pulley PWM
#define TZ_3_ply_PMW 61                  //38 //TZ3 Pulley PWM

#define TZ_1_ply_dir 56                  //39 //TZ1 Pulley direction
#define TZ_2_ply_dir 59                  //40 //TZ2 Pulley direction
#define TZ_3_ply_dir 60                  //41 //TZ3 Pulley direction

#define FORWARD_DIR1  1                  //Left side Motor forward direction
#define FORWARD_DIR2  1                  //Right side Motor forward direction
#define FORWARD_DIR3  1                  //Front side Motor forward direction
#define FORWARD_DIR4  1                  //Back side Motor forward direction
#define BACKWARD_DIR1 0                  //Left side Motor reverse direction
#define BACKWARD_DIR2 0                  //Right side Motor reverse direction
#define BACKWARD_DIR3 0                  //Front side Motor reverse direction
#define BACKWARD_DIR4 0                  //Back side Motor reverse direction

#define LED_STATUS    13
#define VALUE_WHITE_MAX 180

#define TZ_1      1
#define TZ_2      2
#define TZ_3      3

#define TZ_1_sensor 54                   //14 //TZ1 IR sensor for shuttle coke sensing
#define TZ_2_sensor 57                   //15 //TZ2 IR sensor for shuttle coke sensing
#define TZ_3_sensor 52                   //16 //TZ3 IR sensor for shuttle coke sensing

#define LM1 46                           //17 //TZ1 Limit switch
#define LM2 48                           //18 //TZ2 Limit switch
#define LM3 50                           //19 //TZ3 Limit switch

#define DIR_12      1                    //Robot movement from front to back
 #define DIR_21     2                    //Robot movement from back to front
#define DIR_34      3                    //Robot movement from Left to right
#define DIR_43      4                    //Robot movement from Right to Left
#define STOP        5                    //Robot movement steady
#define OBSTACLE_NO 3                    //No of IR sensor for sensing the shuttle coke

//uncomment to use the extra sensor in  case of emergency
//QTRSensorsRC1 qtrrc1((unsigned char[]){} ,NUM_SENSORS, TIEOUT, QTR_NO_EMITTER_PIN);//23-25
QTRSensorsRC3 qtrrc3((unsigned char[]){22,24,26,28,30,32,34,36},NUM_SENSORS, TIMEOUT, QTR_NO_EMITTER_PIN);
QTRSensorsRC2 qtrrc2((unsigned char[]){69,68,67,66,65,64,63,62},NUM_SENSORS, TIMEOUT, QTR_NO_EMITTER_PIN);
QTRSensorsRC4 qtrrc4((unsigned char[]){39,41,43,45,47,49,51,53},NUM_SENSORS, TIMEOUT, QTR_NO_EMITTER_PIN);

uint8_t obstacle_pin;                    //IR pair sensor pin variable
volatile int error = 0;
volatile int last_error = 0;
volatile int position1;                  //Sensor1 Position
volatile int position2;                  //Sensor2 Position
volatile int position3;                  //Sensor3 Position
volatile int position4;                  //Sensor4 Position
int speed1 = 0;                          //Motor speed
int speed2 = 0;                          //Motor speed
int speed_base = SPEED_BASE;

volatile uint8_t is_line_crossed1 = 0;
volatile uint8_t is_line_crossed2 = 0;
volatile uint8_t is_line_crossed3 = 0;
volatile uint8_t is_line_crossed4 = 0;
volatile uint8_t is_line_detected = 0;
volatile uint8_t detectedZone = 0; 

int motor_speed_1 = SPEED_BASE;
int motor_speed_2 = SPEED_BASE;
int sensor_values1[NUM_SENSORS];
int sensor_values2[NUM_SENSORS];
int sensor_values3[NUM_SENSORS];
int sensor_values4[NUM_SENSORS];

int position_count1=0,position_count2=0;
void robotMove(uint8_t);                  //Robot movement routine
int detectThrowingZone(int);              //Throwing zone detection routine
void handleThrow(uint8_t);                //Throwing routine
void throwTZ(unsigned char);              //Shuttle coke throwing routine
void unclamping(unsigned char);           //Stopper unclamping routine
void clamping(unsigned char);             //Stopper clamping routine
void arm_wound(unsigned char);            //Arm wounding routine
void arm_unwound(unsigned char);          //Arm unwounding routine
//void sensor1_calibrate(void);           //uncomment in case of using sensor1
void sensor2_calibrate(void);
void sensor3_calibrate(void);
void sensor4_calibrate(void);
void setup(void)
{
  Serial.begin(9600);
  Serial.println("Start :)");

  pinMode(LED_STATUS,OUTPUT);

  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(DIR3, OUTPUT);
  pinMode(DIR4, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(PWM3, OUTPUT);
  pinMode(PWM4, OUTPUT);

  pinMode(TZ_1_STPR1, OUTPUT);
  pinMode(TZ_1_STPR2, OUTPUT);
  pinMode(TZ_2_STPR1, OUTPUT);
  pinMode(TZ_2_STPR2, OUTPUT);
  pinMode(TZ_3_STPR1, OUTPUT);
  pinMode(TZ_3_STPR2, OUTPUT);

  pinMode(TZ_1_ply_PMW, OUTPUT);
  pinMode(TZ_2_ply_PMW, OUTPUT);
  pinMode(TZ_3_ply_PMW, OUTPUT);

  pinMode(TZ_1_ply_dir, OUTPUT);
  pinMode(TZ_2_ply_dir, OUTPUT);
  pinMode(TZ_3_ply_dir, OUTPUT);

  pinMode(Start,INPUT_PULLUP);
  pinMode(TZ_1_sensor, INPUT_PULLUP);
  pinMode(TZ_2_sensor, INPUT_PULLUP);
  pinMode(TZ_3_sensor, INPUT_PULLUP);
  
  pinMode(LM1, INPUT_PULLUP);
  pinMode(LM2, INPUT_PULLUP);
  pinMode(LM3, INPUT_PULLUP);

  digitalWrite(DIR1, FORWARD_DIR1);
  digitalWrite(DIR2, FORWARD_DIR2);
  digitalWrite(DIR3, FORWARD_DIR3);
  digitalWrite(DIR4, FORWARD_DIR4);
  digitalWrite(PWM1, LOW);
  digitalWrite(PWM2, LOW);
  digitalWrite(PWM3, LOW);
  digitalWrite(PWM4, LOW);
  //sensor1_calibrate();                        //uncomment in case of using sensor1
  
  sensor3_calibrate();
  //delay(2000);
  
  sensor4_calibrate();
  sensor2_calibrate();
  //delay(10000);
       
    //uncomment in case of using sensor1
    //Debug the Max min sensor values on Serial monitor
    /*for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc1.calibratedMinimumOn[i]);
      Serial.print(' ');
    }
    Serial.println();

    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc1.calibratedMaximumOn[i]);
      Serial.print(' ');
    }
    Serial.println();
   */
    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc2.calibratedMinimumOn[i]);
      Serial.print(' ');
    }
    Serial.println();

    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc2.calibratedMaximumOn[i]);
      Serial.print(' ');
    }
    Serial.println();

    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc3.calibratedMinimumOn[i]);
      Serial.print(' ');
    }
    Serial.println();

    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc3.calibratedMaximumOn[i]);
      Serial.print(' ');
    }
    Serial.println();
   
    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc4.calibratedMinimumOn[i]);
      Serial.print(' ');
    }
    Serial.println();

    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc4.calibratedMaximumOn[i]);
      Serial.print(' ');
    }
    Serial.println();

    analogWrite(PWM1,0);
    analogWrite(PWM2,0);
    analogWrite(PWM3,0);
    analogWrite(PWM4,0);
    while(digitalRead(Start));
}
//---------------------------------------------------------------------
//Main loop
void loop() {  
      if(position_count1 == 0 && position_count2 == 0) {
        while(!is_line_detected) {
          robotMove(DIR_43);

          //uncomment to debug robots position
          // Serial.print("Count 1: ");Serial.print(position_count1);
          //Serial.print("     Count 2: ");Serial.println(position_count2);
        }
        is_line_detected=0;
        
        do {      
                //robotMove_1(DIR_43);
                digitalWrite(DIR1, FORWARD_DIR3);
                digitalWrite(DIR2, FORWARD_DIR4);
                analogWrite(PWM1, 70);
                analogWrite(PWM2, 70);
                analogWrite(PWM3, 0);
                analogWrite(PWM4, 0);
                //uncomment to debug robots position
                //Serial.print("Count 1: ");Serial.print(position_count1);
                //Serial.print("     Count 2: ");Serial.println(position_count2);
                qtrrc2.readLine(sensor_values2);
         }while(!(sensor_values2[0] < VALUE_WHITE_MAX ||sensor_values2[1] < VALUE_WHITE_MAX || sensor_values2[7]< VALUE_WHITE_MAX));
 
          is_line_crossed2 = 0;
          robotMove(STOP);
          delay(1000);
       }
         
        if(position_count1 == 0 && position_count2 == 1) {
                 
                 while(!is_line_detected) {
                    robotMove(DIR_12);
                    //uncomment to debug robots position
                    //Serial.print("Count 1: ");Serial.print(position_count1);
                    //Serial.print("      Count 2: ");Serial.println(position_count2);
                 }
                 is_line_detected=0;
                 
                 do {
                    digitalWrite(DIR3,FORWARD_DIR3);
                    digitalWrite(DIR4,FORWARD_DIR4);
                    analogWrite(PWM3,40);
                    analogWrite(PWM4,40);
                    //uncomment to debug robots position
                    //Serial.print("Count 1: ");Serial.print(position_count1);
                    //Serial.print("     Count 2: ");Serial.println(position_count2);
                    //robotMove_1(DIR_12);
                      qtrrc4.readLine(sensor_values4);
                 }while(!(sensor_values4[0] < VALUE_WHITE_MAX||sensor_values4[1] < VALUE_WHITE_MAX || sensor_values4[6] < VALUE_WHITE_MAX||sensor_values4[7]< VALUE_WHITE_MAX ));
                
                is_line_crossed4 = 0;
                robotMove(STOP);
                delay(1000);
        }
     
      
        detectedZone = detectThrowingZone();
        handleThrow(detectedZone);

}
//End main loop
//---------------------------------------------------------------------

//uncomment when using sensor1
/*
void sensor1_calibrate(void)
{
  digitalWrite(LED_STATUS,HIGH);
  for (int i = 0; i < 250; i++) // calibrate for sometime by sliding the sensorValues across the line, or you may use auto-calibration instead 
    { 
       qtrrc1.calibrate();   
       delay(20); 
    }
    digitalWrite(LED_STATUS,LOW);
    delay(2000); // wait for 2s to position the bot before entering the main loop 
}
*/
void sensor2_calibrate(void)
{
  digitalWrite(LED_STATUS,HIGH);
    for (int i = 0; i < 250; i++) // calibrate for sometime by sliding the sensorValues across the line, or you may use auto-calibration instead 
    {
       qtrrc2.calibrate();   
       delay(20); 
    }
    digitalWrite(LED_STATUS,LOW);
    delay(2000); // wait for 2s to position the bot before entering the main loop 
}

void sensor3_calibrate(void)
{
  digitalWrite(LED_STATUS,HIGH);
    for (int i = 0; i < 250; i++) // calibrate for sometime by sliding the sensorValues across the line, or you may use auto-calibration instead 
    { 
       qtrrc3.calibrate();
       
       delay(20); 
    }
    digitalWrite(LED_STATUS,LOW);
    delay(2000); // wait for 2s to position the bot before entering the main loop 
}

void sensor4_calibrate(void) {
  digitalWrite(LED_STATUS,HIGH);
    for (int i = 0; i < 250; i++) {// calibrate for sometime by sliding the sensorValues across the line, or you may use auto-calibration instead 
       qtrrc4.calibrate();   
       delay(20); 
    }
  digitalWrite(LED_STATUS,LOW);
  delay(2000);
}

//Calmping the stopper
void calmping(unsigned char TZ) {
  switch(TZ)
  {
    case TZ_1:
      digitalWrite(TZ_1_STPR1,HIGH);
      digitalWrite(TZ_1_STPR2,LOW);
      delay(800);
      digitalWrite(TZ_1_ply_PMW,LOW);
      digitalWrite(TZ_1_STPR1,LOW);
    break;

    case TZ_2:
      digitalWrite(TZ_2_STPR1,HIGH);
      digitalWrite(TZ_2_STPR2,LOW);
      delay(800);
      digitalWrite(TZ_2_ply_PMW,LOW);
      digitalWrite(TZ_2_STPR1,LOW);
    
    break;

    case TZ_3:
      digitalWrite(TZ_3_STPR1,HIGH);
      digitalWrite(TZ_3_STPR2,LOW);
      delay(800);
      digitalWrite(TZ_3_ply_PMW,LOW);
      digitalWrite(TZ_3_STPR1,LOW);

    break;
  }
}
//Unclamping the stopper
void unclamping(unsigned char TZ) {
  switch(TZ) {
    case TZ_1:
      digitalWrite(TZ_1_STPR1,LOW);
      digitalWrite(TZ_1_STPR2,HIGH);
      delay(800);
      digitalWrite(TZ_1_STPR2,LOW);
    break;

    case TZ_2:
      digitalWrite(TZ_2_STPR1,LOW);
      digitalWrite(TZ_2_STPR2,HIGH);
      delay(800);
      digitalWrite(TZ_2_STPR2,LOW);
    break;

    case TZ_3:
      digitalWrite(TZ_3_STPR1,LOW);
      digitalWrite(TZ_3_STPR2,HIGH);
      delay(800);
      digitalWrite(TZ_3_STPR2,LOW);
    break;
  }
}


//ARM wounding
void arm_wound(unsigned char TZ) {
  switch(TZ) {
    case TZ_1:
    while(digitalRead(LM1)) {
      digitalWrite(TZ_1_ply_dir,HIGH);
      digitalWrite(TZ_1_ply_PMW,HIGH); 
    }
    break;

    case TZ_2:
    while(digitalRead(LM2)) {
      digitalWrite(TZ_2_ply_dir,HIGH);
      digitalWrite(TZ_2_ply_PMW,HIGH); 
    }
   
    
    break;

    case TZ_3:    
    while(digitalRead(LM3)) {
      digitalWrite(TZ_3_ply_dir,HIGH);
      digitalWrite(TZ_3_ply_PMW,HIGH); 
    }
    
    break;
  }
}

//ARM unwounding
void arm_unwound(unsigned char TZ) {
  switch(TZ) {
    case TZ_1:
      digitalWrite(TZ_1_ply_dir,LOW);
      digitalWrite(TZ_1_ply_PMW,HIGH);  
      delay(1050);
      digitalWrite(TZ_1_ply_PMW,LOW);  
    break;

    case TZ_2:
      digitalWrite(TZ_2_ply_dir,LOW);
      digitalWrite(TZ_2_ply_PMW,HIGH);  
      delay(750);
      digitalWrite(TZ_2_ply_PMW,LOW); 
    break;

    case TZ_3:
      digitalWrite(TZ_3_ply_dir,LOW);
      digitalWrite(TZ_3_ply_PMW,HIGH);  
      delay(750);
      digitalWrite(TZ_3_ply_PMW,LOW); 
    break;
  }
}
//Shuttle cock throwing action
void throwTZ(unsigned char TZ)
{
  switch(TZ)
  {
    case TZ_1:
      analogWrite(PWM1,0);
      analogWrite(PWM2,0);
      analogWrite(PWM3,0);
      analogWrite(PWM4,0);
    //delay(2000);
    //Serial.print("throwing started:");
      unclamping(TZ_1);
      arm_wound(TZ_1);
      calmping(TZ_1);
      arm_unwound(TZ_1);
    //Serial.print("throwing done:");  
    break;
  
    case TZ_2:
      analogWrite(PWM1,0);
      analogWrite(PWM2,0);
      analogWrite(PWM3,0);
      analogWrite(PWM4,0);
    //delay(2000);
    //Serial.print("throwing started:");
      unclamping(TZ_2);
      arm_wound(TZ_2);
      calmping(TZ_2);
      arm_unwound(TZ_2);
    //Serial.print("throwing done:");
    break;
  
    case TZ_3:
      analogWrite(PWM1,0);
      analogWrite(PWM2,0);
      analogWrite(PWM3,0);
      analogWrite(PWM4,0);
      delay(2000);
      //Serial.print("throwing started:");
      unclamping(TZ_3);
      arm_wound(TZ_3);
      calmping(TZ_3);
      arm_unwound(TZ_3);
      //Serial.print("throwing done:");
    break;
  }
}

//Throw handling
void handleThrow(uint8_t zone) {
  switch(zone) {
      //case 0:  break;
      case TZ_1:
        if(position_count1 == 1 && position_count2 == 1) {
            while(!is_line_detected) {
              robotMove(DIR_43);
              //uncomment to debug robots position
              //Serial.print("Count 1: ");Serial.print(position_count1);
              //Serial.print("Count 2: ");Serial.println(position_count2);
            }
            is_line_detected=0;
        }
        if(position_count1 == 1 && position_count2 == 2) {
                while(!is_line_detected) {
                  robotMove(DIR_43);
                  //uncomment to debug robots position
                  //Serial.print("Count 1: ");Serial.print(position_count1);
                  //Serial.print("Count 2: ");Serial.println(position_count2);
                }
                is_line_detected=0;
                do {
                  //uncomment to debug robots position
                  //Serial.print("Count 1: ");Serial.print(position_count1);
                  //Serial.print("Count 2: ");Serial.println(position_count2);
                  robotMove_1(DIR_43);
                  qtrrc2.readLine(sensor_values2);
                }while(!(sensor_values2[0] < VALUE_WHITE_MAX || sensor_values2[1] < VALUE_WHITE_MAX || sensor_values2[2] < VALUE_WHITE_MAX || sensor_values2[6] < VALUE_WHITE_MAX||  sensor_values2[7]< VALUE_WHITE_MAX));

                is_line_crossed2 = 0;
                robotMove(STOP);
                delay(1000);
        }
        
             
        if(position_count1 == 1 && position_count2 == 3) { 
            // Serial.println("throwing done:");
            throwTZ(1);
        } 
      
    break;

    case TZ_2:
      if(position_count1 == 1 && position_count2 == 1) {
        while(!is_line_detected) {
          robotMove(DIR_12);
          //uncomment to debug robots position
          //Serial.print("Count 1: ");Serial.print(position_count1);
          //Serial.print("     Count 2: ");Serial.println(position_count2);
        }
        is_line_detected=0;
        do {
          //uncomment to debug robots position
          //Serial.print("Count 1: ");Serial.print(position_count1);
          //Serial.print("     Count 2: ");Serial.println(position_count2);
          digitalWrite(DIR3, FORWARD_DIR3);
          digitalWrite(DIR4, FORWARD_DIR4); 
          analogWrite(PWM3,35);
          analogWrite(PWM4,35);
          
        qtrrc4.readLine(sensor_values4);
        }while(!(sensor_values4[0] < VALUE_WHITE_MAX || sensor_values4[1] < VALUE_WHITE_MAX || sensor_values4[6] < VALUE_WHITE_MAX || sensor_values4[7]< VALUE_WHITE_MAX));
       
        is_line_crossed4 = 0;
        robotMove(STOP);
        delay(1000);
      }
      if(position_count1 == 2 && position_count2 == 1) {
          while(!is_line_detected) {

            robotMove(DIR_43);
            //uncomment to debug robots position
            //Serial.print("Count 1: ");Serial.print(position_count1);
            //Serial.print("     Count 2: ");Serial.println(position_count2);
          }
          is_line_detected=0;
          
      }
      if(position_count1 == 2 && position_count2 == 2) {
        while(!is_line_detected) {

          robotMove(DIR_43);
          //uncomment to debug robots position
          //Serial.print("Count 1: ");Serial.print(position_count1);
          //Serial.print("     Count 2: ");Serial.println(position_count2);
        }
        is_line_detected=0;
           do  {

              //uncomment to debug robots position
              //Serial.print("Count 1: ");Serial.print(position_count1);
              //Serial.print("     Count 2: ");Serial.println(position_count2);
            
              robotMove_1(DIR_43);
              qtrrc2.readLine(sensor_values2);
            } while(!(sensor_values2[0] < VALUE_WHITE_MAX || sensor_values2[1] < VALUE_WHITE_MAX || sensor_values2[6] < VALUE_WHITE_MAX || sensor_values2[7]< VALUE_WHITE_MAX));
            
            is_line_crossed2 = 0;
            robotMove(STOP);
            delay(1000);
        }
        if(position_count1 == 2 && position_count2 == 3) {
            //delay(2000);
            //Serial.println("throwing done:");
            throwTZ(2);
        }
    break;


    case TZ_3:
      if(position_count1 == 2 && position_count2 == 1)
      {
          while(!is_line_detected) {

            robotMove(DIR_43);
            //uncomment to debug robots position
            //Serial.print("Count 1: ");Serial.print(position_count1);
            //Serial.print("     Count 2: ");Serial.println(position_count2);     
          }
          is_line_detected=0;
          
          
      }

      //for calibration of the position
      for(int j=0;j<20;j++) {
        robotMove_1(DIR_43);
      }

      if(position_count1 == 2 && position_count2 == 2) {
          while(!is_line_detected) {
            //uncomment to debug robots position
            robotMove(DIR_43);
          
            //Serial.print("Count 1: ");Serial.print(position_count1);
            //Serial.print("     Count 2: ");Serial.println(position_count2);
          }
          is_line_detected=0;
     
      }
      
      //for calibration of the position
      for(int j=0;j<20;j++) {
        robotMove_1(DIR_43);
      }

      if(position_count1 == 2 && position_count2 == 3) {
          while(!is_line_detected) {
            robotMove(DIR_43);
            //uncomment to check the position
            //Serial.print("Count 1: ");Serial.print(position_count1);
            //Serial.print("     Count 2: ");Serial.println(position_count2);
          }
          is_line_detected=0;
      }
      
      //for calibration of the position
      for(int j=0;j<20;j++) {
        robotMove_1(DIR_43);
      }
      if(position_count1 == 2 && position_count2 == 4) {
          while(!is_line_detected) {robotMove(DIR_43);
            //uncomment to check the position
            //Serial.print("Count 1: ");Serial.print(position_count1);
            //Serial.print("     Count 2: ");Serial.println(position_count2);
          }
          is_line_detected=0;
      }
      
      //for calibration of the position
      for(int j=0;j<20;j++) {
        robotMove_1(DIR_43);
      }
      if(position_count1 == 2 && position_count2 == 5) {
          while(!is_line_detected) {

            robotMove(DIR_43);

            //uncomment to check the position
            //Serial.print("Count 1: ");Serial.print(position_count1);
            //Serial.print("     Count 2: ");Serial.println(position_count2);
          }
          is_line_detected=0;


          
          do {
                //uncomment to check the position
                // Serial.print("Count 1: ");Serial.print(position_count1);
                //Serial.print("     Count 2: ");Serial.println(position_count2);
          
               robotMove_1(DIR_43);
               qtrrc2.readLine(sensor_values2);
          }while(!(sensor_values2[0] < VALUE_WHITE_MAX || sensor_values2[1] < VALUE_WHITE_MAX || sensor_values2[6] < VALUE_WHITE_MAX|| sensor_values2[7]< VALUE_WHITE_MAX));
           is_line_crossed2 = 0;
           robotMove(STOP);
           delay(1000);
      }
            
      if(position_count1 == 2 && position_count2 == 6)
      {
          //  delay(2000);
           //Serial.println("throwing done:");
        throwTZ(3);
      }

      
    break;
   
    default: 

              if(position_count1 == 1 && position_count2 == 3) {
                   while(!is_line_detected) {robotMove(DIR_34);
                   //Serial.print("Count 1: ");Serial.print(position_count1);Serial.print("Count 2: ");Serial.println(position_count2);
                   }
                   is_line_detected=0;
                    
              }
              if(position_count1 == 1 && position_count2 == 2) {
                while(!is_line_detected) {robotMove(DIR_34);
                //Serial.print("Count 1: ");Serial.print(position_count1);Serial.print("Count 2: ");Serial.println(position_count2);
                }
                is_line_detected=0;
                     do 
                    {
                     // Serial.print("Count 1: ");Serial.print(position_count1);Serial.print("Count 2: ");Serial.println(position_count2);
                      robotMove_1(DIR_34);
                      qtrrc2.readLine(sensor_values2);
                    }  while(!(sensor_values2[0] < VALUE_WHITE_MAX || sensor_values2[1] < VALUE_WHITE_MAX || sensor_values2[6] < VALUE_WHITE_MAX || sensor_values2[7]< VALUE_WHITE_MAX));
                    is_line_crossed2 = 0;
                    robotMove(STOP);
                    delay(1000);
                
              }
              if(position_count1 == 2 && position_count2 == 3) {
      
                while(!is_line_detected) {robotMove(DIR_34);
                //Serial.print("Count 1: ");Serial.print(position_count1);Serial.print("     Count 2: ");Serial.println(position_count2);
                }
                is_line_detected=0;
                
                for(int j=0;j<20;j++)
                {
                      robotMove_1(DIR_34);
                }
              
                
              }
              if(position_count1 == 2 && position_count2 == 2) {
                while(!is_line_detected) {
                  robotMove(DIR_34);
                  //Serial.print("Count 1: ");Serial.print(position_count1);Serial.print("     Count 2: ");Serial.println(position_count2);
                }
                is_line_detected=0;
                do {
                  //   Serial.print("Count 1: ");Serial.print(position_count1);Serial.print("     Count 2: ");Serial.println(position_count2);
                  robotMove_1(DIR_34);
                  qtrrc2.readLine(sensor_values2);
                }while(!(sensor_values2[0] < VALUE_WHITE_MAX || sensor_values2[1] < VALUE_WHITE_MAX || sensor_values2[6] < VALUE_WHITE_MAX || sensor_values2[7]< VALUE_WHITE_MAX));
                is_line_crossed2 = 0;
                robotMove(STOP);
                delay(1000);
              }
              if(position_count1 == 2 && position_count2 == 6) {
                  while(!is_line_detected) {robotMove(DIR_34);
                  //Serial.print("Count 1: ");Serial.print(position_count1);Serial.print("     Count 2: ");Serial.println(position_count2);
                  }
                  is_line_detected=0;
                  
                  for(int j=0;j<20;j++) {
                    robotMove_1(DIR_34);
                  }
                  
              }
              if(position_count1 == 2 && position_count2 == 5) {
                  while(!is_line_detected) {robotMove(DIR_34);
                  //Serial.print("Count 1: ");Serial.print(position_count1);Serial.print("     Count 2: ");Serial.println(position_count2);
                  }
                  is_line_detected=0;
                                      
                  for(int j=0;j<20;j++) {
                    robotMove_1(DIR_34);
                  }
                  
              }
              if(position_count1 == 2 && position_count2 == 4) {
                  while(!is_line_detected) {robotMove(DIR_34);
                  //Serial.print("Count 1: ");Serial.print(position_count1);Serial.print("     Count 2: ");Serial.println(position_count2);
                  }
                  is_line_detected=0;
                  
                  for(int j=0;j<20;j++)
                  {
                    robotMove_1(DIR_34);
                    }
                              
              }
              if(position_count1 == 2 && position_count2 == 3) {
                  while(!is_line_detected) {robotMove(DIR_34);
                  //Serial.print("Count 1: ");Serial.print(position_count1);Serial.print("     Count 2: ");Serial.println(position_count2);
                  }
                  is_line_detected=0;
                              
                  for(int j=0;j<20;j++) {
                    robotMove_1(DIR_34);
                  }
                  
              }
             
              break;
    
  }
}

//Shuttle cock detection for perticular zone
int detectThrowingZone()
{     
    Serial.print("detection started:");
    int obstacle_tz1=digitalRead(TZ_1_sensor);  
    int obstacle_tz2=digitalRead(TZ_2_sensor);
    int obstacle_tz3=digitalRead(TZ_3_sensor);
           
          
           if(!obstacle_tz1)
           {
                delay(1500);
                return TZ_1;
           }
           else if(!obstacle_tz2)
           {
               delay(6000);
               return TZ_2;
           
           }
           else if(!obstacle_tz3)
           {
              delay(1500);
             return TZ_3;
           
           }
           else 
              return 0;   
        
}

//Robot movement fuction
void robotMove(uint8_t dir)
{
  switch(dir)
  {
    case DIR_21:
      digitalWrite(DIR3, BACKWARD_DIR3);
      digitalWrite(DIR4, BACKWARD_DIR4);
      //position1=qtrrc1.readLine(sensor_values1);
      calc_motor_speed(position1, sensor_values1);
      analogWrite(PWM3, motor_speed_1);
      analogWrite(PWM4, motor_speed_2);
      analogWrite(PWM1, 0);
      analogWrite(PWM2, 0);
      if(is_line_detected) position_count1 -= 1;
    break;
    case DIR_12:
      digitalWrite(DIR3, FORWARD_DIR3);
      digitalWrite(DIR4, FORWARD_DIR4);
      position2=qtrrc2.readLine(sensor_values2);
      //Serial.println(position2);
      calc_motor_speed(position2, sensor_values2);
      analogWrite(PWM4, motor_speed_1);
      analogWrite(PWM3, motor_speed_2);
      analogWrite(PWM1, 0);
      analogWrite(PWM2, 0);
      if(is_line_detected) position_count1 += 1;
    break;
    case DIR_43:
      digitalWrite(DIR1, FORWARD_DIR3);
      digitalWrite(DIR2, FORWARD_DIR4);
      position3=qtrrc3.readLine(sensor_values3);
      //Serial.println(position2);
      calc_motor_speed(position3, sensor_values3);
      analogWrite(PWM1, motor_speed_1);
      analogWrite(PWM2, motor_speed_2);
      analogWrite(PWM3, 0);
      analogWrite(PWM4, 0);
      if(is_line_detected) position_count2 += 1;
    break;
    case DIR_34:
      digitalWrite(DIR1, BACKWARD_DIR3);
      digitalWrite(DIR2, BACKWARD_DIR4);
      position4=qtrrc4.readLine(sensor_values4);
      //Serial.println(position2);
      calc_motor_speed(position4, sensor_values4);
      analogWrite(PWM1, motor_speed_1);
      analogWrite(PWM2, motor_speed_2);
      analogWrite(PWM3, 0);
      analogWrite(PWM4, 0);
      
      if(is_line_detected) position_count2 -= 1;
    break;
    default:
      speed_base = SPEED_BASE;
      digitalWrite(DIR1, FORWARD_DIR1);
      digitalWrite(DIR2, FORWARD_DIR2);
      digitalWrite(DIR3, FORWARD_DIR3);
      digitalWrite(DIR4, FORWARD_DIR4);
      analogWrite(PWM1, 0);
      analogWrite(PWM2, 0);
      analogWrite(PWM3, 0);
      analogWrite(PWM4, 0);
    break;
  }
}

void robotMove_1(uint8_t dir)
{
  switch(dir)
  {
    case DIR_21:
      digitalWrite(DIR3, BACKWARD_DIR3);
      digitalWrite(DIR4, BACKWARD_DIR4);
      //position1=qtrrc1.readLine(sensor_values1);
      calc_motor_speed_1(position1, sensor_values1);
      analogWrite(PWM3, motor_speed_1);
      analogWrite(PWM4, motor_speed_2);
      analogWrite(PWM1, 0);
      analogWrite(PWM2, 0);
      if(is_line_detected) position_count1 -= 1;
    break;
    case DIR_12:
      digitalWrite(DIR3, FORWARD_DIR3);
      digitalWrite(DIR4, FORWARD_DIR4);
      position2=qtrrc2.readLine(sensor_values2);
      //Serial.println(position2);
      calc_motor_speed_1(position2, sensor_values2);
      analogWrite(PWM4, motor_speed_1);
      analogWrite(PWM3, motor_speed_2);
      analogWrite(PWM1, 0);
      analogWrite(PWM2, 0);
      if(is_line_detected) position_count1 += 1;
    break;
    case DIR_43:
      digitalWrite(DIR1, FORWARD_DIR3);
      digitalWrite(DIR2, FORWARD_DIR4);
      position3=qtrrc3.readLine(sensor_values3);
      //Serial.println(position3);
      calc_motor_speed_1(position3, sensor_values3);
      analogWrite(PWM1, motor_speed_1);
      analogWrite(PWM2, motor_speed_2);
      analogWrite(PWM3, 0);
      analogWrite(PWM4, 0);
          if(is_line_detected) position_count2 += 1;
    break;
    case DIR_34:
      digitalWrite(DIR1, BACKWARD_DIR3);
      digitalWrite(DIR2, BACKWARD_DIR4);
      position4=qtrrc4.readLine(sensor_values4);
      //Serial.println(position4);
      calc_motor_speed_1(position4, sensor_values4);
      analogWrite(PWM1, motor_speed_1);
      analogWrite(PWM2, motor_speed_2);
      analogWrite(PWM3, 0);
      analogWrite(PWM4, 0);
      
      if(is_line_detected) position_count2 -= 1;
    break;
    default:
      speed_base = SPEED_BASE;
      digitalWrite(DIR1, FORWARD_DIR1);
      digitalWrite(DIR2, FORWARD_DIR2);
      digitalWrite(DIR3, FORWARD_DIR3);
      digitalWrite(DIR4, FORWARD_DIR4);
      analogWrite(PWM1, 0);
      analogWrite(PWM2, 0);
      analogWrite(PWM3, 0);
      analogWrite(PWM4, 0);
    break;
  }
}
//Robot movement fuction uncomment when using sensor1
/*void robotMove_3(uint8_t dir)
{
  switch(dir)
  {
    case DIR_21:
      digitalWrite(DIR3, BACKWARD_DIR3);
      digitalWrite(DIR4, BACKWARD_DIR4);
      //position1=qtrrc1.readLine(sensor_values1);
      calc_motor_speed(position1, sensor_values1);
      analogWrite(PWM3, motor_speed_1);
      analogWrite(PWM4, motor_speed_2);
      analogWrite(PWM1, 0);
      analogWrite(PWM2, 0);
      if(is_line_detected) position_count1 -= 1;
    break;
    case DIR_12:
      digitalWrite(DIR3, FORWARD_DIR3);
      digitalWrite(DIR4, FORWARD_DIR4);
      position2=qtrrc2.readLine(sensor_values2);
      //Serial.println(position2);
      calc_motor_speed(position2, sensor_values2);
      analogWrite(PWM4, motor_speed_1);
      analogWrite(PWM3, motor_speed_2);
      analogWrite(PWM1, 0);
      analogWrite(PWM2, 0);
      if(is_line_detected) position_count1 += 1;
    break;
    case DIR_43:
      digitalWrite(DIR1, FORWARD_DIR3);
      digitalWrite(DIR2, FORWARD_DIR4);
      position3=qtrrc3.readLine(sensor_values3);
      //Serial.println(position2);
      calc_motor_speed(position3, sensor_values3);
      analogWrite(PWM1, motor_speed_1);
      analogWrite(PWM2, motor_speed_2);
      analogWrite(PWM3, 0);
      analogWrite(PWM4, 0);
      if(is_line_detected) position_count2 += 1;
    break;
    case DIR_34:
      digitalWrite(DIR1, BACKWARD_DIR3);
      digitalWrite(DIR2, BACKWARD_DIR4);
      position4=qtrrc4.readLine(sensor_values4);
      //Serial.println(position2);
      calc_motor_speed(position4, sensor_values4);
      analogWrite(PWM1, motor_speed_1);
      analogWrite(PWM2, motor_speed_2);
      analogWrite(PWM3, 0);
      analogWrite(PWM4, 0);
      
      if(is_line_detected) position_count2 -= 1;
    break;
    default:
      speed_base = SPEED_BASE;
      digitalWrite(DIR1, FORWARD_DIR1);
      digitalWrite(DIR2, FORWARD_DIR2);
      digitalWrite(DIR3, FORWARD_DIR3);
      digitalWrite(DIR4, FORWARD_DIR4);
      analogWrite(PWM1, 0);
      analogWrite(PWM2, 0);
      analogWrite(PWM3, 0);
      analogWrite(PWM4, 0);
    break;
  }
}*/


//Speed calculation of Motor 
void calc_motor_speed(int position, int sensor_values[8])
{

            if(position_count1 == 0 && position_count2 == 0) {
                  if(sensor_values[0] < VALUE_WHITE_MAX && sensor_values[1] < VALUE_WHITE_MAX && sensor_values[2] < VALUE_WHITE_MAX ) {
                    is_line_detected = 1;
                    if(position_count1 == 1 && position_count2 == 2)
                      speed_base = SPEED_BASE;
                    else if(position_count1 == 2 && position_count2 == 2)
                      speed_base = SPEED_BASE;
                    else if(position_count1 == 2 && position_count2 == 4)
                      speed_base = SPEED_BASE;
                    else if(position_count1 == 2 && position_count2 == 5)
                      speed_base = SPEED_BASE;
                    else
                      speed_base = SPEED_BASE_CROSS; 
                  
                  }
                  else if(sensor_values[5] < VALUE_WHITE_MAX && sensor_values[6] < VALUE_WHITE_MAX && sensor_values[7] < VALUE_WHITE_MAX ) {
                    is_line_detected = 1;
                    if(position_count1 == 1 && position_count2 == 2)
                    speed_base = SPEED_BASE;
                    else if(position_count1 == 2 && position_count2 == 2)
                    speed_base = SPEED_BASE;
                    else if(position_count1 == 2 && position_count2 == 4)
                    speed_base = SPEED_BASE;
                    else if(position_count1 == 2 && position_count2 == 5)
                    speed_base = SPEED_BASE;
                    else
                    speed_base = SPEED_BASE_CROSS; 
                  
                  }
              
            }
            else if((sensor_values[0] < VALUE_WHITE_MAX && sensor_values[1] < VALUE_WHITE_MAX) && (sensor_values[6] < VALUE_WHITE_MAX && sensor_values[7] < VALUE_WHITE_MAX)) {
            is_line_detected = 1;
            if(position_count1 == 1 && position_count2 == 2)
            speed_base = SPEED_BASE;
            else if(position_count1 == 2 && position_count2 == 2)
            speed_base = SPEED_BASE;
            else if(position_count1 == 2 && position_count2 == 4)
            speed_base = SPEED_BASE;
            else if(position_count1 == 2 && position_count2 == 5)
            speed_base = SPEED_BASE;
            else
            speed_base = SPEED_BASE_CROSS; 
          
          }
          else if(sensor_values[0] < VALUE_WHITE_MAX && sensor_values[1] < VALUE_WHITE_MAX && sensor_values[2] < VALUE_WHITE_MAX && sensor_values[3] < VALUE_WHITE_MAX && sensor_values[4] < VALUE_WHITE_MAX && sensor_values[5]< VALUE_WHITE_MAX && sensor_values[6]< VALUE_WHITE_MAX) {
            is_line_detected = 1;
            if(position_count1 == 1 && position_count2 == 2)
            speed_base = SPEED_BASE;
            else if(position_count1 == 2 && position_count2 == 2)
            speed_base = SPEED_BASE;
            else if(position_count1 == 2 && position_count2 == 4)
            speed_base = SPEED_BASE;
            else if(position_count1 == 2 && position_count2 == 5)
            speed_base = SPEED_BASE;
            else
            speed_base = SPEED_BASE_CROSS; 
          
          }
          else if(sensor_values[1] < VALUE_WHITE_MAX && sensor_values[2] < VALUE_WHITE_MAX && sensor_values[3] < VALUE_WHITE_MAX && sensor_values[4] < VALUE_WHITE_MAX && sensor_values[5] < VALUE_WHITE_MAX && sensor_values[6]< VALUE_WHITE_MAX && sensor_values[7]< VALUE_WHITE_MAX) {
            is_line_detected = 1;
            if(position_count1 == 1 && position_count2 == 2)
            speed_base = SPEED_BASE;
            else if(position_count1 == 2 && position_count2 == 2)
            speed_base = SPEED_BASE;
            else if(position_count1 == 2 && position_count2 == 4)
            speed_base = SPEED_BASE;
            else if(position_count1 == 2 && position_count2 == 5)
            speed_base = SPEED_BASE;
            else
            speed_base = SPEED_BASE_CROSS; 
          
          }
          else if(sensor_values[1] < VALUE_WHITE_MAX && sensor_values[2] < VALUE_WHITE_MAX && sensor_values[3] < VALUE_WHITE_MAX && sensor_values[4] < VALUE_WHITE_MAX && sensor_values[5] < VALUE_WHITE_MAX && sensor_values[6]< VALUE_WHITE_MAX) {
            is_line_detected = 1;
            if(position_count1 == 1 && position_count2 == 2)
            speed_base = SPEED_BASE;
            
            else if(position_count1 == 2 && position_count2 == 2)
            speed_base = SPEED_BASE;
            else if(position_count1 == 2 && position_count2 == 4)
            speed_base = SPEED_BASE;
            else if(position_count1 == 2 && position_count2 == 5)
            speed_base = SPEED_BASE;
            else
            speed_base = SPEED_BASE_CROSS; 
          
          }
    
  else is_line_detected = 0;

  error = position - 3500;
  speed1 = KP * error + KD * (error - last_error);
  last_error = error;

  //Serial.print(error);
  //Serial.print(",");
  //Serial.print(speed1);
  //Serial.print(",");
  motor_speed_1 = (speed_base + speed1);
  motor_speed_2 = (speed_base - speed1);
  //Serial.print(motor_speed_1);
  //Serial.print(",");
  //Serial.print(motor_speed_2);
  //Serial.print(",");

  if(motor_speed_1 <= SPEED_MIN) motor_speed_1 = SPEED_MIN;
  if(motor_speed_2 <= SPEED_MIN) motor_speed_2 = SPEED_MIN;

  if(motor_speed_1 >= SPEED_MAX) motor_speed_1 = SPEED_MAX;
  if(motor_speed_2 >= SPEED_MAX) motor_speed_2 = SPEED_MAX;
}

void calc_motor_speed_1(int position, int sensor_values[8])
{
  
  error = position - 3500;
  speed1 = KP * error + KD * (error - last_error);
  last_error = error;
  
  //if(position_count1 == 1 && position_count2 == 1)
            // speed_base=0;
  if(position_count1 == 1 && position_count2 == 2)
        speed_base = SPEED_BASE;

  /*else if(position_count1 == 2 && position_count2 == 1)
        speed_base = 40;
        */
  else if(position_count1 == 2 && position_count2 == 2)
        speed_base = SPEED_BASE;
  else if(position_count1 == 2 && position_count2 == 4)
        speed_base = SPEED_BASE;
  else if(position_count1 == 2 && position_count2 == 5)
        speed_base = SPEED_BASE;
  else
        speed_base = SPEED_BASE_CROSS; 
  //Serial.print(error);
  //Serial.print(",");
  //Serial.print(speed1);
  //Serial.print(",");
  motor_speed_1 = (speed_base + speed1);
  motor_speed_2 = (speed_base - speed1);
  //Serial.print(motor_speed_1);
  //Serial.print(",");
  //Serial.print(motor_speed_2);
  //Serial.print(",");
  
  if(motor_speed_1 <= SPEED_MIN) motor_speed_1 = SPEED_MIN;
  if(motor_speed_2 <= SPEED_MIN) motor_speed_2 = SPEED_MIN;
  
  if(motor_speed_1 >= SPEED_MAX) motor_speed_1 = SPEED_MAX;
  if(motor_speed_2 >= SPEED_MAX) motor_speed_2 = SPEED_MAX;
}


// uncomment when using sensor 1
/*void calc_motor_speed_3(int position, int sensor_values[8])
{
  
  error = position - 3500;
  speed1 = KP * error + KD * (error - last_error);
  last_error = error;
 
  if(position_count1 == 2 && position_count2 == 4)
        speed_base = SPEED_BASE_CROSS;
  else
        speed_base = SPEED_BASE; 
  //Serial.print(error);
  //Serial.print(",");
  //Serial.print(speed1);
  //Serial.print(",");
  motor_speed_1 = (speed_base + speed1);
  motor_speed_2 = (speed_base - speed1);
  //Serial.print(motor_speed_1);
  //Serial.print(",");
  //Serial.print(motor_speed_2);
  //Serial.print(",");
  
  if(motor_speed_1 <= SPEED_MIN) motor_speed_1 = SPEED_MIN;
  if(motor_speed_2 <= SPEED_MIN) motor_speed_2 = SPEED_MIN;
  
  if(motor_speed_1 >= SPEED_MAX) motor_speed_1 = SPEED_MAX;
  if(motor_speed_2 >= SPEED_MAX) motor_speed_2 = SPEED_MAX;
}*/
