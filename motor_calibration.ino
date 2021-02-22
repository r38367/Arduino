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
int S[3] = {180,350,460}; // Sensor value at 0,25,50,75,100%
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
int update_interval_s = 16; 
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
  
}
// ********************
//  Main loop
// ********************
void loop() {
  
  unsigned long tmp;
  char timestr[10];

  digitalWrite(LED_BUILTIN, HIGH); // turn on LED
  
  if(calibrated==false) calibrate_high() ; //calibration();
  
  // read sensor  
  if( sensor_read() < S[1] ) 
  {
    // if lever is lower than Lmin 
    // turn off LED
    digitalWrite(LED_BUILTIN, LOW);  
    
    //ms_to_string(timestr, millis()/1000);
    Serial.print(sensor_value);
    Serial.print(" -> motor started after ");
    //Serial.println(timestr);
    //Serial.print("    last drink ");
    
    last_drink_s = (millis()-motor_stoped_ms)/1000 ;
    ms_to_string(timestr,last_drink_s );
    Serial.println(timestr);  
    

    // if level below min fill in first to the min
     while ( sensor_read() < S[1] ) 
    {
      Serial.println(sensor_value);
      update_screen();
      motor_timer(500); // start motor
      //update_screen();
      //delay(100);
    }
    
    Serial.print(sensor_value);
    Serial.println(" min reached ");
    /*ms_to_string(timestr, millis()/1000);
    Serial.println(timestr);*/
    // keep pumping water while level is below Lmax
    while ( sensor_read() < S[2] ) 
    {
      Serial.println(sensor_value);
      update_screen();
      motor_timer(1000); // start motor
      //update_screen();
      delay(10000);
    }
    
    Serial.print(sensor_value);
    Serial.println(" max reached ");
    /*ms_to_string(timestr, millis()/1000);
    Serial.println(timestr);*/
    // Level is above Lmax now
    // initiate variables
    update_interval_s = 32; 
    //update_screen();
  }
  
   // print to serial
  // Line 1
  Serial.print(sensor_value);
  Serial.print(" "); // 
  Serial.print(sensor_value - prev_sensor_value); //
  Serial.print(" "); // 
  Serial.println(update_interval_s);
  
  // calculatedelay for next update
  if( prev_sensor_value <= sensor_value + 2 )
  {
    if( update_interval_s < 30 ) update_interval_s *= 2; // 1,2,4,8,16,32,1m4s,2m,4m,
  }
  if( prev_sensor_value > sensor_value + 20 )
  {
    if( update_interval_s >= 8 ) update_interval_s /= 2; 
  }
  prev_sensor_value = sensor_value;
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
// return value 0-1024 
// ********************
int sensor_read()
{
    digitalWrite(SENSOR_ON, HIGH);
    delay(50);
    sensor_value = analogRead(A5);
    digitalWrite(SENSOR_ON, LOW);
 
  return sensor_value;
}



// ********************
//  Calibration module
//
// Sets S[0]-S[3] for mapping sensor value to level
// ********************
void calibration()
{
  int Smin=1024;
  int Smax=0;
  char text[17];
  int v=0;
  unsigned long t0;
  

  
  
  Serial.println("sensor calibration");
  
  goto onexit;
  
  // calibrate low, min, high
  for( int i; i<3; i++)
  {  
    lcd.clear();
    
    lcd.setCursor(0,0);
    sprintf(text,"Calibrate %s", S_name[i]);
    lcd.print(text);
    
    lcd.setCursor(0,1);
    sprintf(text,"was %4d", S[i]);
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
    
    Serial.print(S_name[i]);
    Serial.print(" - ");
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
  Serial.println("calibration completed");
  
}

// ********************
//  Calibrate only HIGH
//S[0]-S[3] for mapping sensor value to level
// ********************
void calibrate_high()
{
  char text[17];
  int v=0;
  
  Serial.println("HIGH calibration");
  
    lcd.clear();
    
    lcd.setCursor(0,0);
    sprintf(text,"Calibrate %s", S_name[2]);
    lcd.print(text);
    
    lcd.setCursor(0,1);
    sprintf(text,"%3d", S[0]);
    lcd.print(text);
  
    while( digitalRead(BUTTON_PIN) == HIGH ) { 
      sensor_read();
      print_calibr_value();
       delay(200); 
    } 
    while( digitalRead(BUTTON_PIN) == LOW ) { 
      delay(10); 
    } 
    
    while ( sensor_read() < S[0] ) 
    {
      print_calibr_value();
      motor_timer(500); // start motor
      //update_screen();
      //delay(5000);
    }
    // stabilize
    print_calibr_value();
    sensor_stabilize();  
    
    lcd.setCursor(0,1);
    sprintf(text,"%3d-%3d", S[0], S[1]);
    lcd.print(text);
    
    while ( sensor_read() < S[1] ) 
    {
      print_calibr_value();
      motor_timer(1000); // start motor
      //update_screen();
      delay(200);
    }
    
    print_calibr_value();
    sensor_stabilize();
    
    lcd.setCursor(0,1);
    sprintf(text,"%3d-%3d-%3d", S[0], S[1], S[2]);
    lcd.print(text);
    
    int i=0;
    do {    
      v = sensor_read();
      print_calibr_value();
      if( i== 0 ) {
        motor_timer(1000); // start motor
        i=5;
      }
      delay(1000);
      i--;
    } while( digitalRead(BUTTON_PIN) == HIGH );
  
    while( digitalRead(BUTTON_PIN) == LOW ) { 
      delay(10); 
    } 
    
    print_calibr_value();
    sensor_stabilize();
    
    S[2] = sensor_value; //(v+sensor_read() ) /2;
    S[1] = S[2] - 100;
    
    Serial.print(S_name[1]);
    Serial.print(" - ");
    Serial.println(S[1]);
    
    Serial.print(S_name[2]);
    Serial.print(" - ");
    Serial.println(S[2]);
    
    for(int j=0;j<2;j++){
      lcd.noDisplay();
      delay(500);
      lcd.display();
      delay(500);
    }
    
  
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
  Serial.println("calibration completed");
  
}
void sensor_stabilize()
{
  int p=0;
  int arr[3] = {0,0,0};
  
  Serial.println("stabilize start");
  arr[0] = sensor_value;
  do {
    delay(2000);
    p++; 
    if( p==3) p=0;
    arr[p] = sensor_read();
    print_calibr_value();
    
  } while (arr[0]!=arr[1] or arr[0]!=arr[2]);
  Serial.println("stabilize complete");
}
void print_calibr_value()
{
    char text[10];
    
      Serial.println(sensor_value);
      lcd.setCursor(12,1);
      sprintf(text,"%4d",sensor_value);
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
100%.1020.3000ml - sensor value, sensor range, last push
00d00h...100ml/d - up time, consumption speed
----------------
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
  lcd.setCursor(7, 1);
  if( update_interval_s < 60 )
    sprintf(txt,"%4d/%ds  ", next_mesurement_in_s, update_interval_s); 
    //sprintf(txt,"%4ds\0", next_mesurement_in_s);
  else
    if( next_mesurement_in_s < 60 )
      sprintf(txt,"%4d/%dm  ", (int)next_mesurement_in_s/60, update_interval_s/60);
    else
      sprintf(txt,"%4d/%dm  ", (int)next_mesurement_in_s, update_interval_s/60);
  lcd.print(txt);
}

char * ms_to_string(char *txt, unsigned long sec)
{
  
  if(sec < 3600 ){ 
    // 00m00s
    sprintf(txt, "%dm%02ds\0", int(sec/60), int(sec%60) );
    
  } else if (sec < 86400 ){ 
    // 00h00m
    sprintf(txt, "%dh%02dm\0", int(sec/3600), int((sec/60)%60) );
    
  } else { 
    // 00d00h
    sprintf(txt, "%dd%02dh\0", int(sec/86400), int((sec/3600)%24) );
    
  }
  return (char*)txt;
}

