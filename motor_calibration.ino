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
const int MOTOR_PIN =  10;   // LED lit when motor is on
const int SENSOR_PIN = A5; // int Sensor = Analog 5.
const int SIMULATOR_PIN = 11; // LOW = simulation
const int SENSOR_ON = 8; // pin to turn sensor on

#include <LiquidCrystal.h>
// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 2, en = 3, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

/******************
 * Constants to be calibrated 
 * 
 * **************/
int S[5] = {170,350,400,450,480}; // Sensor value at 0,25,50,75,100%
float motor_mlps = 20.0; // ml/s
int Lmax = 75; // minimum level to start pumping in 0-99 (%)
int Lmin = 10; // minimum level to start pumping in 0-99 (%)
int L0 = 5;    // minimum level to start motor to fill 

/******************
 * Simulation constants 
 * 
 * ****************/
bool  isSimulation = false;
const float timeToFill = 5; // seconds to fill from 0-100%
const float timeToEmpty = 30; // seconds to empty from 100-0%
const float timeToBase = 10;
//const float slang_volume = 28.3 ; // slang volume in ml PI*0.3cm^2*100cm=28cm^3

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
//unsigned long simulator_extra_ms = 0; // ms which correcponds to level in simulation to adjust motor pump

int sensor_value; // sensor value 0-1024
int sensor_level; // sensor mapped to 0-99

float  drink_total = 0;
float  drink_speed = 0;

int prev_sensor_level = 0;
int update_interval_s = 1; 
int next_mesurement_in_s = 0;

// ********************
//  Setup
// ********************

void setup() {
 
  // initialize the LED pin as an output:
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(SENSOR_ON, OUTPUT);
  
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
  isSimulation = false; // digitalRead(SIMULATOR_PIN)==LOW;
  
  if(isSimulation) {
    Serial.println("!!! Simulation !!!");
    calibrated = true;  // skip calibration, use the default values
    motor_worked_s = timeToFill * L0/100; //set water level to L0
  }
}
// ********************
//  Main loop
// ********************
void loop() {
  
  unsigned long tmp;

  digitalWrite(LED_BUILTIN, HIGH); // turn on LED
  
  if(calibrated==false) calibration();
  
  // read sensor  
  if( sensor_read() < Lmin ) 
  {
    
    // if lever is lower than Lmin 
    // turn off LED
    digitalWrite(LED_BUILTIN, LOW);  
    
    // keep pumping water while level is below Lmax
    while ( sensor_read() < Lmax ) 
    {
      update_screen();
      motor_timer(200); // start motor
      //update_screen();
      delay(200);
    }
    
    // Level is above Lmax now
    // initiate variables
    update_interval_s = 1; 
    //update_screen();
  }
    
  // calculatedelay for next update
  if( prev_sensor_level == sensor_level )
  {
    if( update_interval_s < 3 ) update_interval_s *= 2; // 1,2,4,8,16,32,1m4s,2m,4m,
  }
  if( prev_sensor_level > sensor_level + 10 )
  {
    if( update_interval_s >= 2 ) update_interval_s /= 2; 
  }
  prev_sensor_level = sensor_level;
  for(next_mesurement_in_s=update_interval_s; next_mesurement_in_s>0; next_mesurement_in_s--){
    update_screen();
    delay(1000);
  }
}
// ********************
//  Motor module
// ********************
void  motor_start()
{  
  digitalWrite(MOTOR_PIN, HIGH);
  motor_started_ms = millis();
  motor_is_on = true;
  //Serial.println("motor started");
  motor_idle_s = (motor_started_ms - motor_stoped_ms)/1000; // time motor stood idle = drinking mode
}
void  motor_stop()
{
  digitalWrite(MOTOR_PIN, LOW);
  motor_stoped_ms = millis();
  motor_worked_s = (motor_stoped_ms - motor_started_ms)/1000; // time motor worked. = filling moden
  motor_is_on = false;
  //Serial.print("motor stoped after ");
  //Serial.print(motor_worked_s);
  //Serial.println(" s");
}

void motor_timer(unsigned int timer)
{
  motor_start();
  delay(timer);
  motor_stop();
}

// ********************
//  Sensor module
// return value 0-100
// ********************
int sensor_read()
{
     
  if( isSimulation ) 
    sensor_level = sensor_simulate_level(); 
  else { 
    // sensor.on(); 
    int val=0; int x=1;

  digitalWrite(SENSOR_ON, HIGH);
  for(int i=0;i<x; i++){
    delay(10);
    val += analogRead(A5); 
  }
  digitalWrite(SENSOR_ON, LOW);
  
  
    sensor_value = val/x; //analogRead(A5);
    // sensor.off; 
    // get level 0-100
    sensor_level = map_sensor_level(sensor_value);  
  }
  return sensor_level;
}

// sensor value to 0-99 based on calibration
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

// ********************
//  Simulation module
//
// return int in range Smin-Smax
// ********************

int sensor_simulate_level()
{
  unsigned long elapsed_time;

  if( motor_is_on) { // pump mode
    elapsed_time = (millis()-motor_started_ms);
    sensor_level = Lmin + (int)(Lmax-L0)*elapsed_time/(timeToFill*1000);
    if( sensor_level > 100 ) sensor_level = 100;
    
  } else { // drink mode
    elapsed_time = (millis()-motor_stoped_ms);
    sensor_level = Lmax - (int)(Lmax-L0)*elapsed_time/(timeToEmpty*1000);
    if( sensor_level < 0 ) sensor_level = 0;
  }
  return sensor_level;
}

// ********************
//  Calibration module
//
// Sets S[0]-S[4] for mapping sensor value to level
// ********************
void calibration()
{
  int Smin=1024;
  int Smax=0;
  char text[17];
  int v=0;
  unsigned long t0;
/*
  Serial.println("motor calibration");
  
  // wait until button is pressed
  while( digitalRead(BUTTON_PIN) == HIGH ){ Serial.print(".");delay(100);}
  
  // when button is pressed
  motor_start();
  while( millis() - motor_started_ms <= 5000 ){Serial.print(".");delay(100);}
  // start motor for 10 sec
  // stop motor
  motor_stop();
  
  // wait until button is pressed
  while( digitalRead(BUTTON_PIN) == HIGH );
  
  
  // print water level 
  for(int i=0; i<10; i++) 
  {
    v=sensor_read();
    Serial.print(sensor_level);
    Serial.print("% ");
    Serial.print(sensor_value);
    Serial.println(" ");
    delay(200);
  }
 */
  //goto onexit;
  
  Serial.println("sensor calibration");
   
  // calibrate LOW
  
  // calibrate HIGH
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
      v = sensor_read();
      lcd.setCursor(12,1);
      sprintf(text,"%4d",sensor_value);
      lcd.print(text);
      delay(1000);
    } while( digitalRead(BUTTON_PIN) == HIGH );
  
    t0 = millis();
    while( digitalRead(BUTTON_PIN) == LOW ) { 
      delay(10); 
      if( millis()-t0>2000) goto onexit;
      
    } 
    
    
    S[i]=sensor_value;
    
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
----------------
100%.1020.3000ml - sensor value, sensor range, last push
00d00h...100ml/d - up time, consumption speed
----------------
*/
void update_screen()
{
  char txt[17] ;
  float vol ;
  unsigned long msec = millis();
  unsigned long tdown = 0;
  
  // get time string
  char up_time[10];  
  ms_to_string(up_time, msec/1000);
   
  // calculate 
  if( motor_is_on) {
    drink_speed = -motor_mlps;
  }
  else {
    // vol drank so far
    vol = motor_worked_s * motor_mlps;
    drink_total = (vol * float(Lmax - sensor_level)) / (Lmax-Lmin);
    drink_speed =  drink_total / float( (msec-motor_stoped_ms)/1000.0);
    tdown =  (msec-motor_stoped_ms)/100;
  }

  // print to serial
  // Line 1
  Serial.print(sensor_level);
  Serial.print("% ");
  Serial.print(sensor_value);
  Serial.print(" ");
  /*Serial.print(drink_total);
  Serial.print("ml gone in ");
  Serial.print((msec-motor_stoped_ms)/1000.0);
  Serial.print("s "); 
  Serial.print(up_time);
  Serial.print("  ");*/
  Serial.println("");
  
  // row 1
  // print the number of seconds since reset:
  sprintf(txt,"%3d%% %3d  (%3d%%)", sensor_level, sensor_value, prev_sensor_level ); // int((unsigned long)(Lmax-sensor_level)*1000/(msec-motor_stoped_ms)) );
  lcd.setCursor(0, 0);
  lcd.print(txt);
  
  // row 2
  //lcd.setCursor(0, 1);
  //lcd.print(sec_to_time(s));
  
  //   1s/2s
  // 30m/64m
  lcd.setCursor(0, 1);
  lcd.print(up_time);
  lcd.setCursor(7, 1);
  if( update_interval_s < 60 )
    sprintf(txt,"%4d/%ds ", next_mesurement_in_s, update_interval_s);
  else
    if( next_mesurement_in_s < 60 )
      sprintf(txt,"%4d/%dm ", (int)next_mesurement_in_s/60, update_interval_s/60);
    else
      sprintf(txt,"%4d/%dm ", (int)next_mesurement_in_s, update_interval_s/60);
  lcd.print(txt);
}

char * ms_to_string(char *txt, unsigned long sec)
{
  
  if(sec < 3600 ){ 
    // 00m00s
    sprintf(txt, "%2dm%02ds\0", int(sec/60), int(sec%60) );
    
  } else if (sec < 86400 ){ 
    // 00h00m
    sprintf(txt, "%2dh%02dm\0", int(sec/3600), int((sec/60)%60) );
    
  } else { 
    // 00d00h
    sprintf(txt, "%2dd%02dh\0", int(sec/86400), int((sec/3600)%24) );
    
  }
  return (char*)txt;
}

