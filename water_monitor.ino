/*
 * Calibrate water pump before use it in Xmas tree watering
 * 
 *
 */

/*
 * Board pin layout
 * 
 */
// constants won't change. They're used here to set pin numbers:
const int BUTTON_PIN = 13;  // the number of the pushbutton pin
const int MOTOR_PIN =  8;   // LED lit when motor is on
const int SENSOR_PIN = A5; // int Sensor = Analog 5.
const int SIMULATOR_PIN = 11; // HIGH if simulation

#include <LiquidCrystal.h>
// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 2, en = 3, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

/******************
 * Constants to be calibrated 
 * 
 * **************/
int S[5] = {25,500,750,900,1000}; // Sensor value at 0,25,50,75,100%
float motor_mlps = 20.0; // ml/s
int alarm_level = 50; // minimum level to start pumping in 0-99 (%)

/******************
 * Simulation constants 
 * 
 * ****************/
bool  isSimulation = false;
const float timeToFill = 10; // seconds to fill from 0-100%
const float timeToEmpty = 50; // seconds to empty from 100-0%
const float slang_volume = 28.3 ; // slang volume in ml PI*0.3cm^2*100cm=28cm^3

/******************
 * global variables 
 * 
 * ***************/
bool calibrated = false; // true if calibration was already done

unsigned long motor_started_ms = 0; // last time motor was started
unsigned long motor_stoped_ms = 0;  // last time motor was started
bool motor_is_on = false; // true if motor is on

unsigned long motor_worked_s; // sec motor worked last time
unsigned long motor_idle_s; // ms motor is idle
unsigned long simulator_extra_ms = 0; // ms which correcponds to level in simulation to adjust motor pump

int sensor_value; // sensor value 0-1024
int sensor_level; // sensor mapped to 0-99

/****
 * ????
 * ***/

//unsigned long start_level = 0;

//int buttonState = 0;   // variable for reading the pushbutton status

//unsigned long t0;

//float elapsed_time_idle;

/******************
  SETUP
******************/

void setup() {
 
  // initialize the LED pin as an output:
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  // initialize the pushbutton pin as an pull-up input:
  // the pull-up input pin will be HIGH when the switch is open and LOW when the switch is closed.
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  // the pull-up input pin will be HIGH when the switch is open and LOW when the switch is closed.
  pinMode(SIMULATOR_PIN, INPUT_PULLUP); //short to Ground for LOW
  
  
  
  // open a serial connection to display values
  Serial.begin(9600);

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  lcd.clear();
  
  // check if Simulation 
  isSimulation = digitalRead(SIMULATOR_PIN)==LOW;
  if(isSimulation) {
    Serial.println("!!! Simulation !!!");
    calibrated = true;
  }
}
/*********************
start main loop here
**********************/

void loop() {
  
  unsigned long tmp;

  digitalWrite(LED_BUILTIN, HIGH); // turn on LED
  
  if(calibrated==false) calibration();
    
  // read the state of the pushbutton value:
  //buttonState = digitalRead(BUTTON_PIN);
  
  // control LED according to the state of button
  /*if(buttonState == LOW)   {      // If button is pressing
     
    while(digitalRead(BUTTON_PIN) == LOW) {delay(10);}
  */
  
  // read sensor
  //sensor_value = sensor_read();
  //sensor_level = map_sensor_level(sensor_value);
  
  simulator_extra_ms = get_sim_time_for_level(sensor_value);
  
  if( sensor_read() < alarm_level ) {
      
    digitalWrite(LED_BUILTIN, LOW);  // turn off LED

   //Serial.println(sr);
    //Serial.println(sl);
    //Serial.println(start_level);
    motor_start();
    
    /* wait until water reach sensor
    while( sensor_read() < 0) {
      //Serial.print(".");
      delay(200);
    }
    //Serial.println(); */
   
   // time since last water
   //elapsed_time_idle = motor_started_time - motor_stoped_time;
   //Serial.println(elapsed_time_idle/1000.0);
   
    // start timer
    //tmp = millis();
    while ( sensor_read() < 99 ) {
      //Serial.print("~");
      delay(500);
      update_screen(); // todo! change to update_info
    }
    motor_stop();
  
    Serial.print("motor worked ");
    Serial.print(motor_worked_s);
    Serial.print("s ");
    
    Serial.print( motor_worked_s*motor_mlps);
    Serial.println("ml added");
    
    sensor_read();
    simulator_extra_ms = get_sim_time_for_level(sensor_value);
  }

  /*Serial.print(sensor_value);
  Serial.print(" ");
  Serial.print(sensor_level);
  Serial.print("%");
  Serial.print(" ");
  Serial.print(simulator_extra_ms/1000.0);
  Serial.print("s extra");
  Serial.print(" ");
  Serial.print((timeToFill*sensor_level)/100.0);
  Serial.print("s extra from timetofilll");
  Serial.println();*/

  // update screen
  update_screen();

  delay(500);
}
/*****************
  Motor module
***************/
void  motor_start()
{
  digitalWrite(MOTOR_PIN, HIGH);
  motor_started_ms = millis();
  motor_is_on = true;
  Serial.println("motor started");
  motor_idle_s = (motor_started_ms - motor_stoped_ms)/1000; // time motor stood idle = drinking mode
}
void  motor_stop()
{
  digitalWrite(MOTOR_PIN, LOW);
  motor_stoped_ms = millis();
  motor_worked_s = (motor_stoped_ms - motor_started_ms)/1000; // time motor worked. = filling moden
  motor_is_on = false;
  Serial.println("motor stoped");
}

/**********************************
  Sensor module
*/

// always return value 0-99
int sensor_read()
{
  /*/ switch on 5V
  int value = analogRead(SENSOR_PIN);
  // switch off
  */
     
  if( isSimulation ) 
    sensor_value = sensor_simulate(); 
  else 
    sensor_value = analogRead(A5);
  
  sensor_level = map_sensor_level(sensor_value);  
  return sensor_level;
}


// Map sensor value to 0-99 based on calibration 
int map_sensor_level(int v)
{
  int ret;
  
  if (v < S[0]) { 
      ret = 0;
   
  } else if(v < S[1]) { 
    ret = 100*(0.25*(v-S[0]))/(S[1]-S[0]);
 
  } else if(v < S[2]) { 
    ret = 100*((0.25*(v-S[1]))/(S[2]-S[1])+0.25);
  
  } else if(v < S[3]) { 
    ret = 100*((0.25*(v-S[2]))/(S[3]-S[2])+0.5);
  
  } else if(v < S[4]) { 
    ret = 100*((0.25*(v-S[3]))/(S[4]-S[3])+0.75);
  }   
  else {
    ret = 100;
  }  
  return ret;
}
// ***** Sensor simulation *************
// 
// return int in range Smin-Smax
int sensor_simulate()
{
  int ret;
  unsigned long t4;
  
  if( motor_is_on) { // pump mode
  
    
    unsigned long elapsed_time=(millis()-motor_started_ms) + simulator_extra_ms;
    t4 = timeToFill*1000/4;
    
    if (elapsed_time < t4) { // 0-25% filled up
      ret = S[0] + ((elapsed_time - t4*0)*(S[1]-S[0]))/t4;
   
    } else if(elapsed_time < t4*2) { //25-50% filled up
      ret = S[1] + ((elapsed_time - t4*1)*(S[2]-S[1]))/t4;
   
    } else if(elapsed_time < t4*3) { //50-75% filled up
      ret = S[2] + ((elapsed_time - t4*2)*(S[3]-S[2]))/t4;
    
    } else { //75-100% filled up
      ret = S[3] + ((elapsed_time - t4*3)*(S[4]-S[3]))/t4;
      
      if( ret > S[4]) ret = S[4];
    }
  } else { // drink mode
      ret = S[4] - ((S[4]-S[0])/timeToEmpty)*(millis()-motor_stoped_ms)/1000.0; 
      if(ret < S[0]) ret = S[0];
  }
  return ret;
}
// retuen how much time motor shold have been working to reach this level
// used in simulation to start filling water from same lavel as alarm_level 
unsigned long get_sim_time_for_level(int v)
{
  unsigned long ret;
  float t4 = timeToFill*1000.0/4.0;
  
  if (v < S[0]) { 
      ret = 0;
      
  } else if(v < S[1]) { 
    ret = (t4*(v-S[0]))/(S[1]-S[0]);

  } else if(v < S[2]) { 
    ret = (t4*(v-S[1]))/(S[2]-S[1])+t4;

  } else if(v < S[3]) { 
    ret = (t4*(v-S[2]))/(S[3]-S[2])+t4*2;

  } else if(v < S[4]) { 
    ret = (t4*(v-S[3]))/(S[4]-S[3]) + t4*3;

  }   
  else {
    ret = t4*4;
  }  
  
  return ret;
}

// manual measurements
// view results on screen

void calibration()
{
  int Smin=1024;
  int Smax=0;
  char text[17];
  int v=0;
  unsigned long t0;

  Serial.println("in calibration");
    
  for( int i; i<5; i++)
  {  
    lcd.clear();
    
    lcd.setCursor(0,0);
    sprintf(text,"Calibrate %d%%", 100*i/4);
    lcd.print(text);
    
    lcd.setCursor(0,1);
    sprintf(text,"was %d", S[i]);
    lcd.print(text);
    
    do {    
      v = analogRead(A5);
      lcd.setCursor(12,1);
      sprintf(text,"%4d",v);
      lcd.print(text);
      delay(100);
    } while( digitalRead(BUTTON_PIN) == HIGH );
  
    t0 = millis();
    while( digitalRead(BUTTON_PIN) == LOW ) { 
      delay(10); 
      if( millis()-t0>2000) goto onexit;
      
    } 
    
    
    S[i]=v;
    
    Serial.print(i*100/4);
    Serial.print("% - ");
    Serial.println(S[i]);
    
    for(int j=0;j<2;j++){
      lcd.noDisplay();
      delay(500);
      lcd.display();
      delay(500);
    }
    
    //delay(1000);
  }
  
onexit: 
  lcd.clear();
  lcd.print("  Calibration");
  lcd.setCursor(0,1);
  lcd.print("   completed");


  //for(int j=0;j<1;j++){
      lcd.noDisplay();
      delay(500);
      lcd.display();
      delay(500);
  //}
  while( digitalRead(BUTTON_PIN) == HIGH ){delay(50);}
  
  lcd.clear();
  calibrated = true;
  Serial.println("out calibration");
  
}

/*********************
 * 
 *  Update screen info 
 * 
 * 
 * 
0123456789012345
1024.100%.3000ml - sensor value, sensor range, last push
00d00h...250ml/h - up time, consumption speed
         
*/
void update_screen()
{
  char txt[17] ;
  //int v = sensor_read();
  unsigned long msec = millis();
  
  int ml = motor_mlps * (int)motor_worked_s; //consump since last motor stop
  //int last_idle_time_sec = (elapsed_time_idle)/1000; // last time motor stoped
  float drink_speed;
  
  if( !motor_is_on) drink_speed = ml * (100.0 - sensor_level)/(100.0-alarm_level)/int((msec-motor_stoped_ms)/1000);
  
  Serial.print(sensor_level);
  Serial.print("% ");
  Serial.print(100 - sensor_level);
  Serial.print("% gone in ");
  Serial.print((msec-motor_stoped_ms)/1000.0);
  Serial.print("s ");
  Serial.print(drink_speed);
  Serial.print("ml/m ");
  Serial.print(ml);
  Serial.print("ml ");
  Serial.println();
  
  lcd.clear();
  
  // row 1
  lcd.setCursor(0, 0);
  // print the number of seconds since reset:
  sprintf(txt,"%3d%% %4d %4dml", sensor_level, sensor_value, ml );
  lcd.print(txt);
  // row 2
  //lcd.setCursor(0, 1);
  //lcd.print(sec_to_time(s));
  
  lcd.setCursor(0, 1);
  //sprintf(txt, "%5dml", (int)ml ); //"%02d:%02d:%02d:%02d %d", s/86400,(s/3600) % 24,(s/60) % 60,(s) % 60, s );
  //sprintf(txt, "%02d:%02d:%02d %5dml", (int)(s/3600),(int)(s/60)%60, (int)(s%60), (int)ml ); //"%02d:%02d:%02d:%02d %d", s/86400,(s/3600) % 24,(s/60) % 60,(s) % 60, s );
  
  unsigned long sec = msec/1000;
  if(sec < 3600 ){ // under 1h: 00m00s
    sprintf(txt, "%2dm%02ds %5dmls", int(sec/60), int(sec%60), (int)drink_speed );
  }else if (sec < 86400 ){ // under 24h: 00h00m
    sprintf(txt, "%2dh%02dm %5dmls", int(sec/3600), int((sec/60)%60), (int)drink_speed );
  }else{ // 00d00h
    sprintf(txt, "%2dd%02dh %5dmls", int(sec/86400), int((sec/3600)%24), (int)drink_speed );
    
  }
  lcd.print(txt);
}
