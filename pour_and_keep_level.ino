/*

Simple Xmas tree watering whish uses minimum level for water sensor 
Starts working as soon as the water level is above MAX level.
Mearsures water level periodically. When the level is below the MIN level
it starts water pump and keeps pumping until level is over MAX. 

Update History:

19/12/21 v1 - initial prototype that works
11/12/21 v2 
 - screen is off when motor is working to save load
 - motor stops after 30s if max is not reached (likely empty watertank)
 - fixed format on screen
13/12/23 v3 
 - fixed #4 - restart after power outage
 - changed min interval to 30s
 - changed initial screen to Merry Xmas and version 
 - changed initial screen to show countdown at start 
 - power off protection: motor start pumping after 30s at start! 
  
*/
/******************
 * Global constants 
 * 
 * **************/
const int ver=3;  // product version!
int S[3] = {100,120,250}; // Sensor value at 0,50,100%


/******************
 * Board pin layout
 * 
 * **************/
// constants won't change. They're used here to set pin numbers:
const int BUTTON_PIN = 13;  // the number of the pushbutton pin
const int MOTOR_PIN =  10;   // LED lit when motor is on
const int SENSOR_PIN = A5; // int Sensor = Analog 5.
//const int SIMULATOR_PIN = 11; // LOW = simulation
const int SENSOR_ON = 8; // pin to turn sensor on
const int SCREEN_ON = 12; // pin to turn screen on

#include <LiquidCrystal.h>
// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 2, en = 3, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

/******************
 * Global variables 
 * 
 * ***************/
bool calibrated = false; // true if calibration was already done

unsigned long motor_started_ms = 0; // last time motor was started
unsigned long motor_stoped_ms = 0;  // last time motor was started
bool motor_is_on = false; // true if motor is on

unsigned long motor_worked_s; // sec motor worked last time
unsigned long motor_idle_s; // ms motor is idle
//unsigned long simulator_extra_ms = 0; // ms which correcponds to level in simulation to adjust motor pump
unsigned long last_drink_s = 0; // time since last drink

int sensor_value; // sensor value 0-1024

float  drink_total = 0;
float  drink_speed = 0;
float pSpeed = 0;


int prev_sensor_value = 0;
int update_interval_s = 30; 
int next_mesurement_in_s = 0;
int set_max_min = 1; // set _max min value first time 
int s2go = 0; // sec to go to min
int startv, diffv; //sensor value before motor start

// ********************
//  Setup
// ********************

void setup() {
 
  // initialize the LED pin as an output:
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(SENSOR_ON, OUTPUT);
  pinMode(SCREEN_ON, OUTPUT);
  
  // initialize the pushbutton pin as an pull-up input:
  // the pull-up input pin will be HIGH when the switch is open and LOW when the switch is closed.
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // the pull-up input pin will be HIGH when the switch is open and LOW when the switch is closed.
  //pinMode(SIMULATOR_PIN, INPUT_PULLUP); //short to Ground for LOW
  
  // open a serial connection to display values
  Serial.begin(9600);

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  lcd.clear();
  
  // switch screen on
  digitalWrite(SCREEN_ON, HIGH);
  lcd.setCursor(0,0);

  // initial message max/min
  print_start_screen();
  delay(3000);

  // wait 30s until sensor is in water, but not more than 30s
  int  start_delay = 30; // countdown at start before motor starts if sensor is dry
  while( sensor_read() < S[2] and start_delay > 0)
  {
    print_delay_screen(sensor_value, S[2], start_delay);
    delay(1000);
    start_delay-- ;
  }
 
 // initialize variables 
  prev_sensor_value = sensor_value;
  set_max_min = 1;
  update_interval_s = 5; // set short interval to see how it works 
  startv = S[1];
  lcd.clear();
  digitalWrite(LED_BUILTIN, LOW);
  
  
}
// ********************
//  Main loop
// ********************
void loop() {
  
  unsigned long tmp;
  char timestr[10];
  
   // print to serial
  // Line 1
  Serial.print(sensor_value);
  Serial.print(" "); // 
  Serial.print(sensor_value - prev_sensor_value); //
  Serial.print(" "); // 
  Serial.print(pSpeed); //
  Serial.print(" "); // 
  Serial.println(update_interval_s);
  
  // keep screen on during countdown
  digitalWrite(SCREEN_ON, HIGH);
  
  // wait interval
  for(next_mesurement_in_s=update_interval_s; next_mesurement_in_s>0; next_mesurement_in_s--)
  {
    update_screen();
    delay(1000);
  }
  // switch screen off 
  digitalWrite(SCREEN_ON, LOW);
  
  
  prev_sensor_value = sensor_value;
  
  // read sensor 
  sensor_read();
  
  // check if we below Min level
  if( sensor_value <= S[1] ) 
  {
    // if lever is lower than Lmin 
    startv = sensor_value; // value before motor start
    diffv = 0; // difference
    Serial.print(sensor_value);
    Serial.print(" -> motor started after ");
    
    last_drink_s = (millis()-motor_stoped_ms)/1000 ;
    ms_to_string(timestr,last_drink_s );
    Serial.println(timestr);  
    
    // keep pumping water while level is below Lmax
    motor_start();
    while ( sensor_read() + diffv < S[2] ) 
    {
      diffv = max(diffv, startv-sensor_value);
      Serial.print(sensor_value);
      Serial.print(" + ");
      Serial.print(diffv);
      Serial.print(" = ");
      Serial.println(sensor_value + diffv);
      update_screen();
      //motor_timer(1000); // start motor
      //update_screen();
      delay(200);
      
      // TODO:add check that motor works too long - 30s? 
      if ( millis() - motor_started_ms > 30000 ) 
      { 
        // set threshold to 0 to stop further attempt
        S[1]=0;
        break;
      }
    }
    motor_stop();
    //sensor_value += diffv; 
    Serial.print(sensor_value + diffv);
    Serial.println(" max reached ");
    
    // Level is above Lmax now
    // initiate variables
    update_interval_s = 60; //min(16,update_interval_s); 
    //update_screen();
    prev_sensor_value = sensor_value;
    pSpeed = 0;
  }
  else
  {
    // calculate delay for next update
    // current speed
    pSpeed = float(sensor_value - S[1])/float(prev_sensor_value - S[1]);
    
    if( pSpeed > 0.8 ) 
    {
      if( update_interval_s <= 1800 ) update_interval_s *= 2; // max period 1h=3600s
    }
    else if ( pSpeed < 0.4)
    {
      if( update_interval_s >= 60 ) update_interval_s /= 4; // min period 15s
    }
    else if ( pSpeed < 0.6)
    {
      if( update_interval_s >= 30 ) update_interval_s /= 2; // min period 15s
    }
    
    // new function for next delay
    //
    // prev_value  next_value  min_value
    //   |          |           |
    //   v          v           v
    // --+----------+-----------+----
    //              |<---P1---->|
    //   |<------P0------------>|
    //
    // if P1/P0 
    //    > 0.8 -> too slow change -> interval *=2
    //    < 0.6 -> too fast change -> interval /=2
    //    within 0.6-0.8 -> ok -> keep interval
    // 
    
    
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
// return value 0-1024 
// ********************
int sensor_read()
{
    digitalWrite(SENSOR_ON, HIGH);
    delay(10);
    sensor_value = analogRead(A5);
    delay(10);
    sensor_value = analogRead(A5);
    digitalWrite(SENSOR_ON, LOW);
 
  return sensor_value;
}

// ********************
//  Stabilize sensor by wating until reading is the same X times 
//
// ********************

void sensor_stabilize()
{
  sensor_stabilize(3);
}
void sensor_stabilize(int times)
{
  int p=1,cnt=0;
  int v1,v2;
  
  Serial.println("stabilize start");
  
  v1 = sensor_value;
  do 
  {
    delay(2000);
    v2 = sensor_read();
    if( v2==v1) 
    {
      p++;
    }
    else 
    {
      p =1;
      v1 = v2;
    }
    print_calibr_value(v2, p);
    cnt++;
  } while (p<times and cnt<100);
  
  Serial.println("stabilize complete");
}
void print_calibr_value(int v2, int p)
{
    char text[10];
    
      Serial.print(v2);
      Serial.print(" ");
      Serial.println(p);
      lcd.setCursor(10,1);
      sprintf(text,"%1d %4d",p, v2);
      lcd.print(text);
}

/*********************
 * 
 *  Print start screen 
 * 
0123456789012345
----------------
Merry Xmass v2.0   
max=000, min=000
----------------
0123456789012345
*/
void print_start_screen()
{
    char text[17];
    
      // line 1
      lcd.setCursor(0,0);
      sprintf(text,"Merry Xmas v%d",ver);
      lcd.print(text);

      // Line 2
      lcd.setCursor(0,1);
      sprintf(text,"min=%3d, max=%3d", S[1], S[2]);
      lcd.print(text);
}

/*********************
 * 
 *  Print delay screen 
 * 
0123456789012345
----------------
start.in.00s.or.
push.to.000:.000
----------------
0123456789012345
*/
void print_delay_screen(int now, int max, int time)
{
    char text[17];
    
      // line 1
      lcd.setCursor(0,0);
      sprintf(text,"start in %2ds or ", time);
      lcd.print(text);

      // Line 2
      lcd.setCursor(0,1);
      sprintf(text,"push to %3d: %3d", max, now);
      lcd.print(text);
}

/*********************
 * 
 *  Update screen info 
 * 
 * 
 * 
0123456789012345
----------------
AAA BBbBBb CCdCC 
DDdDDd EEE FFFFF 
----------------
0123456789012345

AAA - last measured sensor value
BBB - last time before refill
CCC - time since motor stopped 
DDD - up time  
EEE - sensor value when  motor started
FFF - time (s) to next measurement 
*/
void update_screen()
{
  char txt[17] ;
  float vol ;
  unsigned long msec = millis();

  // get time string
  char up_time[10]; 
  char last_time[10]; 
  

  // row 1
  // print the number of seconds since reset:
  ms_to_string(up_time, (msec - motor_stoped_ms)/1000);
  ms_to_string(last_time, last_drink_s );
  sprintf(txt,"%3d %6s %s ", sensor_value, last_time, up_time ); // int((unsigned long)(Lmax-sensor_level)*1000/(msec-motor_stoped_ms)) );
  lcd.setCursor(0, 0);
  lcd.print(txt);
  
  // row 2
  //lcd.setCursor(0, 1);
  //lcd.print(sec_to_time(s));
  
  //   1s/2s
  // 30m/64m
  ms_to_string(up_time, msec/1000);
  lcd.setCursor(0, 1);
  lcd.print( up_time );
  
  // diffv
  lcd.setCursor(7, 1);
  sprintf(txt,"%3d ", startv);
  lcd.print( txt );
  
  // print remaining time until measurement
  lcd.setCursor(11, 1);
  //sprintf(txt,"%4d", next_mesurement_in_s);
  
  
  if( next_mesurement_in_s < 60 )
    sprintf(txt,"%5ds ", next_mesurement_in_s); 
  else if( next_mesurement_in_s < 3600 )
    sprintf(txt,"%2dm%02d", next_mesurement_in_s/60,next_mesurement_in_s%60);
  else
    sprintf(txt,">%dh   ", next_mesurement_in_s/3600);
  //txt[5]='\0';
  // 00000s 
  // 00m00s
  // >0h...   
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

