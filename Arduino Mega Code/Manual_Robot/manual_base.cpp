 #include <PS2X_lib.h>  

/******************************************************************
 * set pins connected to PS2 controller:
 *   - 1e column: original 
 *   - 2e colmun: Stef?
 * replace pin numbers by the ones you use
 ******************************************************************/
#define PS2_DAT        13  //14    
#define PS2_CMD        12  //15
#define PS2_SEL        11  //16
#define PS2_CLK        10  //17
#define joystick_x 127
#define joystick_y 127
#define variation_x 40
#define variation_y 40
#define joyminup_x joystick_x+variation_x
#define joymindown_x joystick_x-variation_x
#define joyminup_y joystick_y+variation_y
#define joymindown_y joystick_y-variation_y

/******************************************************************
 * select modes of PS2 controller:
 *   - pressures = analog reading of push-butttons 
 *   - rumble    = motor rumbling
 * uncomment 1 of the lines for each mode selection  
 ******************************************************************/
//#define pressures   true
#define pressures   false
//#define rumble      true
#define rumble      false

PS2X ps2x; // create PS2 Controller Class


  int error = 0;
  byte type = 0;
  byte vibrate = 0;
  int dirr1=5;                 //5
  int dirr2=3;                 //3
  int dirf1=9;                 //9
  int dirf2=7;                 //7
  int conveyf=29;              //pwm
  int conveyb=27;              //dir
  int linkf=23;                //dir
  int linkb=25;                //pwm

void setup() {
 
  Serial.begin(57600);
  pinMode(dirf2,OUTPUT);
  pinMode(dirf1,OUTPUT);
  pinMode(dirr2,OUTPUT);
  pinMode(dirr1,OUTPUT);
  pinMode(conveyf,OUTPUT);
  pinMode(conveyb,OUTPUT);
  pinMode(linkf,OUTPUT);
  pinMode(linkb,OUTPUT);
  delay(300);  //added delay to give wireless ps2 module some time to startup, before configuring it
   
  //CHANGES for v1.6 HERE!!! **************PAY ATTENTION*************
  
  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  
  if(error == 0) {
      Serial.print("Found Controller, configured successful ");
      Serial.print("pressures = ");

      if (pressures)
        Serial.println("true ");
      else
        Serial.println("false");
      Serial.print("rumble = ");
      if (rumble)
        Serial.println("true)");
      else 
        Serial.println("false");

      Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
      Serial.println("holding L1 or R1 will print out the analog stick values.");

  }  
  else if(error == 1)
    Serial.println("No controller found, check wiring ");
   
  else if(error == 2)
    Serial.println("Controller found but not accepting commands");

  else if(error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it.");

  
  type = ps2x.readType(); 

  switch(type) {
    case 0:
      Serial.print("Unknown Controller type found ");
      break;
    case 1:
      Serial.print("DualShock Controller found ");
      break;
    case 2:
      Serial.print("GuitarHero Controller found ");
      break;
    case 3:
      Serial.print("Wireless Sony DualShock Controller found ");
      break;
   }

}

void loop() {
   
  if(error == 1) //skip loop if no controller found
    return; 
  
  if(type == 2) { //Guitar Hero Controller
    ps2x.read_gamepad();          //read controller 
   
    if(ps2x.ButtonPressed(GREEN_FRET))
      Serial.println("Green Fret Pressed");
    if(ps2x.ButtonPressed(RED_FRET))
      Serial.println("Red Fret Pressed");
    if(ps2x.ButtonPressed(YELLOW_FRET))
      Serial.println("Yellow Fret Pressed");
    if(ps2x.ButtonPressed(BLUE_FRET))
      Serial.println("Blue Fret Pressed");
    if(ps2x.ButtonPressed(ORANGE_FRET))
      Serial.println("Orange Fret Pressed"); 

    if(ps2x.ButtonPressed(STAR_POWER))
      Serial.println("Star Power Command");
    
    if(ps2x.Button(UP_STRUM))          //will be TRUE as long as button is pressed
      Serial.println("Up Strum");
    if(ps2x.Button(DOWN_STRUM))
      Serial.println("DOWN Strum");
 
    if(ps2x.Button(PSB_START))         //will be TRUE as long as button is pressed
      Serial.println("Start is being held");
    if(ps2x.Button(PSB_SELECT))
      Serial.println("Select is being held");
    
    if(ps2x.Button(ORANGE_FRET)) {     // print stick value IF TRUE
      Serial.print("Wammy Bar Position:");
      Serial.println(ps2x.Analog(WHAMMY_BAR), DEC); 
    } 
  }

  else {  //DualShock Controller
    ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
    
    if(ps2x.Button(PSB_START))         //will be TRUE as long as button is pressed
      Serial.println("Start is being held");
    if(ps2x.Button(PSB_SELECT))
      Serial.println("Select is being held");      

    if(ps2x.Button(PSB_PAD_UP)) {      //will be TRUE as long as button is pressed
      Serial.print("Up held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
      digitalWrite(conveyf,HIGH);
      digitalWrite(conveyb,LOW);
      
    }
    else {
      digitalWrite(conveyf,LOW);
    }

    if(ps2x.Button(PSB_PAD_RIGHT)) {
      Serial.print("Right held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
      digitalWrite(linkf,LOW);
      digitalWrite(linkb,HIGH);
    }
    else {
      digitalWrite(linkb,LOW);
    }

    if(ps2x.Button(PSB_PAD_LEFT)) {
      Serial.print("LEFT held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
      digitalWrite(linkf,LOW);
      digitalWrite(linkb,HIGH);
      
    }
    else {
      digitalWrite(linkb,LOW);
    }
    
    if(ps2x.Button(PSB_PAD_DOWN) {
      Serial.print("DOWN held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
      digitalWrite(conveyf,HIGH);
      digitalWrite(conveyb,HIGH);
    }   
    else {
      digitalWrite(conveyf,LOW);
    }


    
    //this will set the large motor vibrate speed based on how hard you press the blue (X) button

    vibrate = ps2x.Analog(PSAB_CROSS);  
    if (ps2x.NewButtonState()) {        //will be TRUE if any button changes state (on to off, or off to on)
      if(ps2x.Button(PSB_L3))
        Serial.println("L3 pressed");

      if(ps2x.Button(PSB_R3))
        Serial.println("R3 pressed");

      if(ps2x.Button(PSB_L2))
        Serial.println("L2 pressed");

      if(ps2x.Button(PSB_R2))
        Serial.println("R2 pressed");

      if(ps2x.Button(PSB_TRIANGLE))
        Serial.println("Triangle pressed");        
    }

    if(ps2x.ButtonPressed(PSB_CIRCLE)) {
      
      //will be TRUE if button was JUST pressed
      Serial.println("Circle just pressed");
     
    }
        
    if(ps2x.NewButtonState(PSB_CROSS))               //will be TRUE if button was JUST pressed OR released
      Serial.println("X just changed");
    if(ps2x.ButtonReleased(PSB_SQUARE))              //will be TRUE if button was JUST released
      Serial.println("Square just released");     

      
    if(ps2x.Button(PSB_L1)) {


      int pwm_yup= map(ps2x.Analog(PSS_LY), 0,joymindown_y , 195, 50);
      int pwm_yup_1= map(ps2x.Analog(PSS_LY), 0,joymindown_y , 255, 50);
      int pwm_ydown= map(ps2x.Analog(PSS_LY),joyminup_y,255,50,255);
       
      int pwm_xleft = map(ps2x.Analog(PSS_LX),0,joymindown_x,255,50);
      int pwm_xright = map(ps2x.Analog(PSS_LX),joyminup_x,255,50,255);
       
      if((ps2x.Analog(PSS_LX)>=joymindown_x && ps2x.Analog(PSS_LX)<=joyminup_x ) && (ps2x.Analog(PSS_LY)>=joymindown_y && ps2x.Analog(PSS_LY)<=joyminup_y) ) {
         
          delay(10);
          analogWrite(4,0);
          analogWrite(2,0);
          analogWrite(6,0);
          analogWrite(8,0);

      }
      else if(ps2x.Analog(PSS_LX)>=0 && ps2x.Analog(PSS_LX)<joymindown_x && (ps2x.Analog(PSS_LY)>=joymindown_y && ps2x.Analog(PSS_LY)<=joyminup_y)) {

          delay(10);
          digitalWrite(dirf1,LOW);
          digitalWrite(dirf2,LOW);
          digitalWrite(dirr1,LOW);
          digitalWrite(dirr2,LOW);
          analogWrite(4,pwm_xleft);
          analogWrite(2,pwm_xleft);
          analogWrite(8,0);
          analogWrite(6,0);
      }
      else if(ps2x.Analog(PSS_LX)>joyminup_x && ps2x.Analog(PSS_LX)<=255 && (ps2x.Analog(PSS_LY)>=joymindown_y && ps2x.Analog(PSS_LY)<=joyminup_y)) {
          delay(10);
          digitalWrite(dirf1,HIGH);
          digitalWrite(dirf2,HIGH);
          
          digitalWrite(dirr1,HIGH);
          digitalWrite(dirr2,HIGH);
          analogWrite(4,pwm_xright);
          analogWrite(2,pwm_xright);
          analogWrite(8,0);
          analogWrite(6,0);
      }
      
      else if((ps2x.Analog(PSS_LX)>=joymindown_x && ps2x.Analog(PSS_LX)<=joyminup_x ) && (ps2x.Analog(PSS_LY)>=0 && ps2x.Analog(PSS_LY)<joymindown_y)) {
           delay(10);
           digitalWrite(dirf1,LOW);
           digitalWrite(dirf2,LOW);
           analogWrite(8,pwm_yup_1);
           analogWrite(6,pwm_yup);
           analogWrite(2,0);
           analogWrite(4,0);
      }
        else if((ps2x.Analog(PSS_LX)>=joymindown_x && ps2x.Analog(PSS_LX)<=joyminup_x ) && (ps2x.Analog(PSS_LY)>joyminup_y && ps2x.Analog(PSS_LY)<=255)) {
          delay(10);
          digitalWrite(dirf1,HIGH);
          digitalWrite(dirf2,HIGH);
          analogWrite(8,pwm_ydown);
          analogWrite(6,pwm_ydown);
          analogWrite(4,0);
          analogWrite(2,0);
      }
       
      else {
          delay(10);
          analogWrite(4,0);
          analogWrite(2,0);
          analogWrite(6,0);
          analogWrite(8,0);

      }
       
    }
    else if(ps2x.Button(PSB_R1)) {
      int pwm_turnright=map(ps2x.Analog(PSS_RX),0,joymindown_x,100,50);
      int pwm_turnleft=map(ps2x.Analog(PSS_RX),joyminup_x,255,50,100);
      if((ps2x.Analog(PSS_RX)>=0 && (ps2x.Analog(PSS_RX)<=joymindown_x))) {
          delay(10);
          digitalWrite(dirf1,HIGH);        //3
          digitalWrite(dirf2,LOW);         //high 4
          digitalWrite(dirr1,LOW);         //low
          digitalWrite(dirr2,HIGH);
          analogWrite(4,pwm_turnright);
          analogWrite(2,pwm_turnright);
          analogWrite(6,pwm_turnright);
          analogWrite(8,pwm_turnright);
        
      }
      else if((ps2x.Analog(PSS_RX)>=joyminup_x && (ps2x.Analog(PSS_RX)<=255))) {
        delay(10);
        digitalWrite(dirf1,LOW);
        digitalWrite(dirf2,HIGH);
        digitalWrite(dirr1,HIGH);
        digitalWrite(dirr2,LOW);
        analogWrite(4,pwm_turnleft);
        analogWrite(2,pwm_turnleft);
        analogWrite(6,pwm_turnleft);
        analogWrite(8,pwm_turnleft);
      
      }
      else {
        delay(10);
        analogWrite(4,0);
        analogWrite(2,0);
        analogWrite(6,0);
        analogWrite(8,0);
      }
        
  }

  delay(50);  
  }
}
