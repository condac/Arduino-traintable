

/*
Arduino DCC automatic train controller
by Burns, Juli 2012

This program sends pulses 0v-5v on DCC_PIN that needs to be amplified by a booster
works well with LMD18200 Booster 
And with Märklin Delta 6604 66045

Some parts of the program is originaly from Michael Blank and is released under GNU license

This program is free software; you can redistribute it and/or modify it under the terms of the 
GNU General Public License as published by the Free Software Foundation; 
either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
See the GNU General Public License for more details.
*/

//All pins
/*
0 serial
1 serial
2 Man/auto switch
3 Start/Stop switch
4 DCC signal
5 DCC PWM
6 Station 1
7 Station 2
8 Pre station 1
9 Pre station 2
10
11 Park 1
12 Park 2
13 LED

Analog
0 Gas 1
1 Gas 2
2 Current sense
3
4
5 Overload LED

*/
#define OVER_LED   13

#define MAN_SW     2  // MAN/AUTO Switch
#define START_SW     3  // Start switch
#define SENSOR1    8  // Pre station sensor1
#define SENSOR2    9  // Pre station sensor1
#define PARK1     11  // Park sensor 1
#define PARK2     12  //Park sensor 2
#define DCC_PIN    4  // Arduino pin for DCC out 
                      // this pin is connected to "DIRECTION" of LMD18200
#define DCC_PWM    5  // must be HIGH for signal out
                      // connected to "PWM in" of LMD18200
//#define DCC_THERM  0  // thermal warning PIN //never used
#define AN_SPEED1   A0  // analog reading for Speed Poti
#define AN_SPEED2   A1  // analog reading for Speed Poti
#define CURRENT_PIN A2 // Current sense from lmd18200


// never used #define AN_CURRENT 0  // analog input for current sense reading
#define TRAIN2_BTN 2  // ON-OFF train 2

//Timer frequency is 2MHz for ( /8 prescale from 16MHz )
#define TIMER_SHORT 0x8D  // 58usec pulse length 
#define TIMER_LONG  0x1B  // 116usec pulse length 

unsigned char last_timer=TIMER_SHORT;  // store last timer value
   
unsigned char flag=0;  // used for short or long pulse
unsigned char every_second_isr = 0;  // pulse up or down

// definitions for state machine 
#define PREAMBLE 0    
#define SEPERATOR 1
#define SENDBYTE  2







unsigned char state= PREAMBLE;
unsigned char preamble_count = 16;
unsigned char outbyte = 0;
unsigned char cbit = 0x80;


boolean firststart = true;
int maxcurrent = 0;
int current;
int overcurrent = 36; //was 36
boolean currentprotection = true;
boolean trackOn = false;
int hertz = 10;
int wait = 1000/hertz;


//variables for speed messure
int timespeed = 0;

// variables for throttle
int locoSpeed=0;
int dir = 1;
int last_locoSpeed=0;
int last_dir;
int dirPin = 3;
int FPin[] = { 3,3,3,3};
int maxF =3;
int locoAdr=3;   // this is the (fixed) address of the loco
int runtime = 0;
int stoptime = 0;
boolean stopping = false;
int locomaxspeed = 40;
int loco1parkspeed = 30;

boolean parked1 = false;
int laptime1 = 30*hertz;

// Train nr2
int STATIONTIME = hertz*10; // 100ms loop 5s = 50
int runtime2 = 0;
int stoptime2 = 0;
int locoSpeed2 = 0;
int dir2 = 1;
int last_locoSpeed2=0;

int last_dir2;
int dirPin2 = 3;


int FPin2[] = { 3,3,3,3};
int maxF2 = 3;
int locoAdr2 = 1;   // this is the (fixed) address of the loco
boolean stopping2 = false;
int loco2maxspeed = 120;
int loco2parkspeed = 100;
boolean parked2 = false;
int laptime2 = 30*hertz;

// buffer for command
struct Message {
   unsigned char data[7];
   unsigned char len;
} ;

#define MAXMSG  5
// for the time being, use only two messages - the idle msg and the loco Speed msg

struct Message msg[MAXMSG] = { 
    { { 0xFF,     0, 0xFF, 0, 0, 0, 0}, 3},   // idle msg
    { { locoAdr, 0x3F,  0, 0, 0, 0, 0}, 4},    // locoMsg with 128 speed steps
    { { locoAdr2, 0x3F,  0, 0, 0, 0, 0}, 4},
    { { locoAdr, 0x90,  0, 0, 0, 0, 0}, 4}, // 0x90 means funktion packet and light on
    { { locoAdr2, 0x90,  0, 0, 0, 0, 0}, 4}
  };               // loco msg must be filled later with speed and XOR data byte
                                
int msgIndex=0;  
int byteIndex=0;


//Setup Timer2.
//Configures the 8-Bit Timer2 to generate an interrupt at the specified frequency.
//Returns the time load value which must be loaded into TCNT2 inside your ISR routine.
void SetupTimer2(){
 
  //Timer2 Settings: Timer Prescaler /8, mode 0
  //Timmer clock = 16MHz/8 = 2MHz oder 0,5usec
  TCCR2A = 0;
  TCCR2B = 0<<CS22 | 1<<CS21 | 0<<CS20; 

  //Timer2 Overflow Interrupt Enable   
  TIMSK2 = 1<<TOIE2;

  //load the timer for its first cycle
  TCNT2=TIMER_SHORT; 

}

//Timer2 overflow interrupt vector handler
ISR(TIMER2_OVF_vect) {
  //Capture the current timer value TCTN2. This is how much error we have
  //due to interrupt latency and the work in this function
  //Reload the timer and correct for latency.  
  // for more info, see http://www.uchobby.com/index.php/2007/11/24/arduino-interrupts/
  unsigned char latency;
  
  // for every second interupt just toggle signal
  if (every_second_isr)  {
     digitalWrite(DCC_PIN,1);
     every_second_isr = 0;    
     
     // set timer to last value
     latency=TCNT2;
     TCNT2=latency+last_timer; 
     
  }  else  {  // != every second interrupt, advance bit or state
     digitalWrite(DCC_PIN,0);
     every_second_isr = 1; 
     
     switch(state)  {
       case PREAMBLE:
           flag=1; // short pulse
           preamble_count--;
           if (preamble_count == 0)  {  // advance to next state
              state = SEPERATOR;
              // get next message
              msgIndex++;
              if (msgIndex >= MAXMSG)  {  msgIndex = 0; }  
              byteIndex = 0; //start msg with byte 0
           }
           break;
        case SEPERATOR:
           flag=0; // long pulse
           // then advance to next state
           state = SENDBYTE;
           // goto next byte ...
           cbit = 0x80;  // send this bit next time first         
           outbyte = msg[msgIndex].data[byteIndex];
           break;
        case SENDBYTE:
           if (outbyte & cbit)  { 
              flag = 1;  // send short pulse
           }  else  {
              flag = 0;  // send long pulse
           }
           cbit = cbit >> 1;
           if (cbit == 0)  {  // last bit sent, is there a next byte?
              byteIndex++;
              if (byteIndex >= msg[msgIndex].len)  {
                 // this was already the XOR byte then advance to preamble
                 state = PREAMBLE;
                 preamble_count = 16;
              }  else  {
                 // send separtor and advance to next byte
                 state = SEPERATOR ;
              }
           }
           break;
     }   
 
     if (flag)  {  // if data==1 then short pulse
        latency=TCNT2;
        TCNT2=latency+TIMER_SHORT;
        last_timer=TIMER_SHORT;
     }  else  {   // long pulse
        latency=TCNT2;
        TCNT2=latency+TIMER_LONG; 
        last_timer=TIMER_LONG;
     }  
  }

}

void setup(void) {
  pinMode(DCC_PWM,OUTPUT);   // will be kept high when power on rail is on, PWM pin
  digitalWrite(DCC_PWM,LOW);
  Serial.begin(9600); 
  //Set the pins for DCC to "output".
  pinMode(DCC_PIN,OUTPUT);   // this is for the DCC Signal
  pinMode(13,OUTPUT);



  pinMode(OVER_LED,OUTPUT);   // this is for the DCC Signal
  
  //pinMode(DCC_THERM, INPUT);
  //digitalWrite(DCC_THERM,1); //enable pull up

  pinMode(MAN_SW,INPUT);
  digitalWrite(MAN_SW,1); //enable pull up
  
  pinMode(START_SW,INPUT);
  digitalWrite(START_SW,1); //enable pull up
  
  pinMode(dirPin, INPUT);
  digitalWrite(dirPin, 1);  //enable pull-up resistor !!
  
  pinMode(SENSOR2, INPUT);
  digitalWrite(SENSOR2, 1);  //enable pull-up resistor
  
  pinMode(SENSOR1, INPUT);
  digitalWrite(SENSOR1, 1);  //enable pull-up resistor
  
  pinMode(PARK1, INPUT);
  digitalWrite(PARK1, 1);  //enable pull-up resistor
  
  pinMode(PARK2, INPUT);
  digitalWrite(PARK2, 1);  //enable pull-up resistor
  
  for (int i=0 ; i<=maxF; i++){ //not used
     pinMode(FPin[i], INPUT);
     digitalWrite(FPin[i], 1);  //enable pull-up resistor
  }  
 
  //read_locoSpeed_etc();
  assemble_dcc_msg();
  assemble_dcc_msg2light();
  assemble_dcc_msglight();
  //Start the timer 
  SetupTimer2();   

  digitalWrite(OVER_LED,LOW);
        Serial.println("Setup completed");

}

void loop(void) {
  if (currentprotection) {
    if (trackOn) {
      digitalWrite(DCC_PWM,HIGH);
    }
  //delay(200);
  for (int i=0; i<wait; i++) {
    current = analogRead(CURRENT_PIN);
    if (current > maxcurrent) {
      maxcurrent=current;
    }
    if (current >overcurrent) {
      digitalWrite(DCC_PWM,LOW);
      digitalWrite(OVER_LED,HIGH);
      currentprotection = false;
      Serial.println("HIGH CURRENT");
      
    }
    delay(1);
  }
  
  //speed messure
  timespeed++;
  if (!digitalRead(SENSOR2) ) {
    if (timespeed > hertz*4) {
      int fart = (806*hertz)/timespeed;
      Serial.print("Speed: ");
      Serial.print(fart);

      
      Serial.println("km/h");
      timespeed = 0;
      
      
    }
    
  }
  current = analogRead(CURRENT_PIN);
  Serial.print("Current: ");
  Serial.print(current);  
  Serial.print("max current: ");
  Serial.println(maxcurrent);

  if ( digitalRead(START_SW) && parked1 && parked2 ) {
    //if power button is off and all trains are at the parking area
      digitalWrite(DCC_PWM,LOW);
  }
  else if ( digitalRead(START_SW) ) {
    Serial.println("Start OFF" ); 
    //if switch is in OFF state
    if (!digitalRead(PARK1) || parked1) {
      locoSpeed = 1;
      assemble_dcc_msg();
      parked1= true;
    }
    else {
        locoSpeed = loco1parkspeed;
        assemble_dcc_msg();
    }
    if (!digitalRead(PARK2) || parked2) {
      locoSpeed2 = 1;
      assemble_dcc_msg2();
      parked2= true;
    }
    else {
        locoSpeed2 = loco2parkspeed;
        assemble_dcc_msg2();
    }
      Serial.print("Stopping : " ); 
      Serial.print(locoSpeed); 
      Serial.print(" : " ); 
      Serial.print(locoSpeed2); 
  }
  else {
    Serial.println("Start ON" ); 
    //else powerswitch is in ON state
    //digitalWrite(DCC_PWM,HIGH);
    trackOn = true;
    parked1= false;
    parked2= false;
    if ( digitalRead(MAN_SW) ) {
      if (man_loco1())  {
       // some reading changed
       // make new dcc message
       assemble_dcc_msg();
      }
      if (man_loco2())  {
       // some reading changed
       // make new dcc message
       assemble_dcc_msg2();
      }
      Serial.print("Manual : " ); 
      Serial.print(locoSpeed); 
      Serial.print(" : " ); 
      Serial.print(locoSpeed2); 
      sensortest();
    }
    else {
      if (auto_loco2())  {
       // some reading changed
       // make new dcc message
       assemble_dcc_msg2();
      }
      if (auto_loco1())  {
       // some reading changed
       // make new dcc message
       assemble_dcc_msg();
      } 
      Serial.print("Auto : " ); 
      Serial.print(locoSpeed); 
      Serial.print(" : " ); 
      Serial.println(locoSpeed2); 
    }
  }
  }
}

/*
boolean read_locoSpeed_etc()  {
   boolean changed = false;
   // read the analog input into a variable:
   
   // limit range to 0..127
   locoSpeed = (127L * analogRead(AN_SPEED1))/1023;
   
   if (locoSpeed != last_locoSpeed) { 
      changed = true;  
      last_locoSpeed = locoSpeed;
   }
   
   dir = digitalRead(dirPin);
   
   if (dir != last_dir)  {  
      changed = true;  
      last_dir = dir;
   }

   return changed;
}
*/
void assemble_dcc_msg() {
   int i;
   unsigned char data, xdata;
   
   if (locoSpeed == 1)  {  // this would result in emergency stop
      locoSpeed = 0;
   }
   
   // direction info first
   if (dir)  {  // forward
      data = 0x80;
   }  else  {
      data = 0;
   }
   
   data |=  locoSpeed;
     
   // add XOR byte 
   xdata = (msg[1].data[0] ^ msg[1].data[1]) ^ data;
   
   noInterrupts();  // make sure that only "matching" parts of the message are used in ISR
   msg[1].data[2] = data;
   msg[1].data[3] = xdata;
   interrupts();

}

boolean auto_loco1()  {
   boolean changed = false;
   // read the analog input into a variable:
   
   // limit range to 0..127
   //locoSpeed = (127L * analogRead(AN_SPEED))/1023;
   if (!digitalRead(SENSOR1)) {
     if (runtime >= laptime1) {
       stopping = true;
     }
     
   }
   else {
     
   }
   if (stopping) {
     locoSpeed--;
     locoSpeed--;
   }
   else {
     locoSpeed++;
     runtime++;
   }
   if (locoSpeed >=locomaxspeed) {
     locoSpeed = locomaxspeed;
   }
   if (locoSpeed <=0) {
     locoSpeed = 0;
     stoptime++;
   }
    if (stoptime >= STATIONTIME) {
      stopping = false;
      stoptime = 0;
      runtime = 0;
    }
   if (locoSpeed == 1)  {  // this would result in emergency stop
      locoSpeed = 2;
   }
   if (locoSpeed != last_locoSpeed) { 
      changed = true;  
      last_locoSpeed = locoSpeed;
   }

   return changed;
}



boolean auto_loco2()  {
   boolean changed = false;
   // read the analog input into a variable:
   
   // limit range to 0..127
   //locoSpeed = (127L * analogRead(AN_SPEED))/1023;
   if (!digitalRead(SENSOR2)) {
     if (runtime2 >= laptime2) {
       stopping2 = true;
     }
     digitalWrite(13,1);
   }
   else {
     digitalWrite(13,0);
   }
   if (stopping2) {
     //locoSpeed2--;
     locoSpeed2--;
     if (locoSpeed2 == 2) {
       locoSpeed2 = 0;
     }
   }
   else {
     locoSpeed2++;
     runtime2++;
   }
   if (locoSpeed2 >=loco2maxspeed) {
     locoSpeed2 = loco2maxspeed;
   }
   if (locoSpeed2 <=0) {
     locoSpeed2 = 0;
     stoptime2++;
   }
    if (stoptime2 >= STATIONTIME) {
      stopping2 = false;
      stoptime2 = 0;
      runtime2 = 0;
    }
   if (locoSpeed2 == 1)  {  // this would result in emergency stop
      locoSpeed2 = 2;
   }
   if (locoSpeed2 != last_locoSpeed2) { 
      changed = true;  
      last_locoSpeed2 = locoSpeed2;
   }

   
   if (dir2 != last_dir2)  {  
      changed = true;  
      last_dir2 = dir2;
   }

   return changed;
}

boolean man_loco2()  {
   boolean changed = false;
   // read the analog input into a variable:
   
   // limit range to 0..127
   locoSpeed2 = (127L * analogRead(AN_SPEED2))/1023;
   if (locoSpeed2 == 1)  {  // this would result in emergency stop
      locoSpeed2 = 0;
   }
   if (locoSpeed2 != last_locoSpeed2) { 
      changed = true;  
      last_locoSpeed2 = locoSpeed2;
   }
   
   /*dir = digitalRead(dirPin);
   
   if (dir != last_dir)  {  
      changed = true;  
      last_dir = dir;
   }*/

   return changed;
}

boolean man_loco1()  {
   boolean changed = false;
   // read the analog input into a variable:
   
   // limit range to 0..127
   locoSpeed = (127L * analogRead(AN_SPEED1))/1023;
   if (locoSpeed == 1)  {  // this would result in emergency stop
      locoSpeed = 0;
   }
   if (locoSpeed != last_locoSpeed) { 
      changed = true;  
      last_locoSpeed = locoSpeed;
   }
   
   /*dir = digitalRead(dirPin);
   
   if (dir != last_dir)  {  
      changed = true;  
      last_dir = dir;
   }*/

   return changed;
}

void assemble_dcc_msg2() {
   int i;
   unsigned char data, xdata;
   
   if (locoSpeed2 == 1)  {  // this would result in emergency stop
      locoSpeed2 = 0;
   }
   
   // direction info first
   if (dir2)  {  // forward
      data = 0x80;
   }  else  {
      data = 0;
   }
   
   data |=  locoSpeed2;
     
   // add XOR byte 
   xdata = (msg[2].data[0] ^ msg[2].data[1]) ^ data;
   
   noInterrupts();  // make sure that only "matching" parts of the message are used in ISR
   msg[2].data[2] = data;
   msg[2].data[3] = xdata;
   interrupts();

}
void assemble_dcc_msg2light() {
   int i;
   unsigned char data, xdata;
   

   
   data = 0;
   
   // add XOR byte 
   xdata = (msg[4].data[0] ^ msg[4].data[1]) ^ data;
   
   noInterrupts();  // make sure that only "matching" parts of the message are used in ISR
   msg[4].data[2] = data;
   msg[4].data[3] = xdata;
   interrupts();

}
void assemble_dcc_msglight() {
   int i;
   unsigned char data, xdata;
   
   data = 0;
   
   // add XOR byte 
   xdata = (msg[3].data[0] ^ msg[3].data[1]) ^ data;
   
   noInterrupts();  // make sure that only "matching" parts of the message are used in ISR
   msg[3].data[2] = data;
   msg[3].data[3] = xdata;
   interrupts();

}
void sensortest() {
   /* #define MAN_SW     2  // MAN/AUTO Switch
#define START_SW     3  // Start switch
#define SENSOR1    8  // Pre station sensor1
#define SENSOR2    9  // Pre station sensor1
#define PARK1     22  // Park sensor 1
#define PARK2     7  //Park sensor 2
*/
    Serial.print("Sensortest:");
    Serial.print("MAN_SW:");
    Serial.print(digitalRead(MAN_SW));  
    Serial.print(" START_SW:");
    Serial.print(digitalRead(START_SW));
        Serial.print(" SENSOR1:");
    Serial.print(digitalRead(SENSOR1));
        Serial.print(" SENSOR2:");
    Serial.print(digitalRead(SENSOR2));
        Serial.print(" PARK1:");
    Serial.print(digitalRead(PARK1));
        Serial.print(" PARK2:");
    Serial.print(digitalRead(PARK2));
}
