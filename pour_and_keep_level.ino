/*
Simple Xmas tree watering whish uses minimum level for water sensor 

Starts working as soon as the water level is above MAX level.
Mearsures water level periodically. When the level is below the MIN level
it starts water pump and keeps pumping until level is over MAX. 

Update History:
19/12/21
v1.0 - initial prototype that works

 */

/*
 * Board pin layout
 * 
 */
// constants won't change. They're used here to set pin numbers:
const int BUTTON_PIN = 13;  // the number of the pushbutton pin
const int MOTOR_PIN =  10;   // LED lit when motor is on
const int SENSOR_PIN = A5; // int Sensor = Analog 5.
//const int SIMULATOR_PIN = 11; // LOW = simulation
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
int S[3] = {100,120,250}; // Sensor value at 0,50,100%
char* S_name[3] = { "Empty", "Low  ", "High " };
//int Lmax = 450; // volume is full
//int Lmin = 300; // volume is low
//int L0 = 180;   // volume is 0 

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
unsigned long last_drink_s = 0; // time since last drink

int sensor_value; // sensor value 0-1024

float  drink_total = 0;
float  drink_speed = 0;

int prev_sensor_value = 0;
int update_interval_s = 32; 
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
/*
  // while sensor value is less than min water level print sensor level
  Serial.print("Empty level: ");
  Serial.println(S[0]);
  
  lcd.setCursor(0,0);
  //"1234567890123456"
  //"Fill to max  000"
  //"Empty level...  "
  lcd.print("Empty level...  ");
  lcd.noDisplay();
  // fill to the max and stabilize sensor 
  sensor_stabilize(3);
  lcd.display();
  
  S[0]=sensor_value;
  Serial.print("Min level: ");
  Serial.print(S[0]);
  Serial.print("  in ");
  Serial.print( millis()/1000);
  Serial.println("s");
*/
  lcd.setCursor(0,0);
  //lcd.print("Push to max     ");
  //lcd.setCursor(13,0); lcd.print(S[0]);
  lcd.print("Push to water  ");
  lcd.setCursor(0,1); 
  lcd.print("over ");
  lcd.print(S[2]);

  //lcd.setCursor(13,0); lcd.print(S[2]);
  
  while( sensor_read() < S[2] )
  {
    print_calibr_value(sensor_value, 0);
    delay(1000);
  }
  /* 
  // fill to the max and stabilize sensor 
  unsigned long tmp = millis();
  lcd.noDisplay();
  sensor_stabilize(3); 
  lcd.display();
  
  // set max value to stabilized value
  //S[2] = sensor_value;
  Serial.print("Max level: ");
  Serial.print(S[2]);
  Serial.print("  in ");
  Serial.print( (millis() - tmp)/1000);
  Serial.println("s");
  
  S[1] = S[0] + (S[2]-S[0])*0.2; 
  Serial.print("Mid level: ");
  Serial.println(S[1]);
 
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Levels:"); 
  lcd.setCursor(0,1); lcd.print(S[0]);
  lcd.setCursor(5,1); lcd.print(S[1]);
  lcd.setCursor(10,1); lcd.print(S[2]); 
  delay(3000);
 */
  prev_sensor_value = sensor_value;
  set_max_min = 1;
  update_interval_s = 4;
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
  Serial.println(update_interval_s);
  
  // wait interval
  for(next_mesurement_in_s=update_interval_s; next_mesurement_in_s>0; next_mesurement_in_s--)
  {
    update_screen();
    delay(1000);
  }
  
  prev_sensor_value = sensor_value;
  
  // read sensor 
  sensor_read();
  
  //if( sensor_value < S[2] ) 
  //{
    
      // current speed
    //s2go = update_interval_s*(sensor_value - S[1])/(prev_sensor_value - sensor_value);
    
    // calculate delay for next update
    if( sensor_value > int(prev_sensor_value * 0.95) )
    {
      if( update_interval_s < 1000 ) update_interval_s *= 2; // 1,2,4,8,16,32,1m4s,2m,4m, 
    }
    if( sensor_value < int(prev_sensor_value * 0.8) )
    {
      if( update_interval_s >= 8 ) update_interval_s /= 2; 
    } 
  //}
  
  /*if( set_max_min == 1 )
  {
    S[2] = sensor_value;
    S[1] = S[2]*0.9;
    Serial.print("Max2 level: ");
  Serial.println(S[2]);
     Serial.print("Min2 level: ");
  Serial.println(S[1]);
  
    set_max_min = 0;
  }  */
  
  if( sensor_value < S[1] ) 
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
    }
    motor_stop();
    //sensor_value += diffv; 
    Serial.print(sensor_value + diffv);
    Serial.println(" max reached ");
    
    // Level is above Lmax now
    // initiate variables
    update_interval_s = min(16,update_interval_s); 
    //update_screen();
    prev_sensor_value = sensor_value;
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
 *  Update screen info 
 * 
 * 
 * 
0123456789012345
----------------
AAA BBbBBb CCdCC - sensor value, , last push
DDdDDd EEEE FFFF - ....01m12s - up time, consumption speed
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
  sprintf(txt,"%3d %s %s ", sensor_value, last_time, up_time ); // int((unsigned long)(Lmax-sensor_level)*1000/(msec-motor_stoped_ms)) );
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
  lcd.setCursor(12, 1);
  sprintf(txt,"%4d", next_mesurement_in_s);
  
  /*
  if( next_mesurement_in_s < 100 )
    sprintf(txt,"%2ds ", next_mesurement_in_s);
  else if( next_mesurement_in_s < 3600 )
    sprintf(txt,"%2dm ", next_mesurement_in_s/60);
  else
    sprintf(txt,"%2dh ", next_mesurement_in_s/3600);
  */
  lcd.print(txt);
}

char * ms_to_string(char *txt, unsigned long sec)
{
  
  if(sec < 3600 ){ 
    // 00m00s
    sprintf(txt, "%dm%02ds \0", int(sec/60), int(sec%60) );
    
  } else if (sec < 86400 ){ 
    // 00h00m
    sprintf(txt, "%dh%02dm \0", int(sec/3600), int((sec/60)%60) );
    
  } else { 
    // 00d00h
    sprintf(txt, "%dd%02dh \0", int(sec/86400), int((sec/3600)%24) );
    
  }
  return (char*)txt;
}
