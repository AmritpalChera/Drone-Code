/*Amritpal Chera
 * 2019-06-03
 * This is the main code that control the quadcopter during its flight
 * It is meant to do critically time dependant calculations and send data to ESCs to control the motors
 * It also reads real time data from both the gyro and the receiver to calculate the optimal output for the ESCs
 * Special Credit to Joop Brokking** for his code; this code is influenced by his code on a similar quadcopter.
 * Major changes include that this code is specifically meant for hardware that is in current use and is not generalized for all hardware
 * PID controller is directly influenced by Joop Brooking as PID is a very complex algorithm to program, however, a full understanding of PID was established.
 */
//Include the libraries 
#include <Wire.h>
#include <EEPROM.h>

//PID gain and limits

//Roll: 3 seperate variables must be made for proportional, integral and derivative controller

float pid_pgain_roll =1;  //increase this in steps of 0.2 till overcompensate and then lower by 50%
float pid_dgain_roll =1;  //increase by 0.01 
float pid_igain_roll =3;  //increase till it becomes restless and lower by 25%
int pid_max_roll = 400;  //Max output from the pid

//Pitch: 3 seperate variables must be made for proportional, integral and derivative controller

float pid_pgain_pitch =pid_pgain_roll;  
float pid_dgain_pitch =pid_dgain_roll;
float pid_igain_pitch =pid_igain_roll;
int pid_max_pitch = 400; //Maximum pitch output from pid

//YAW: 3 seperate variables must be made for proportional, integral and derivative controller

float pid_pgain_yaw =3;  
float pid_dgain_yaw =0.02;
float pid_igain_yaw =0;
int pid_max_yaw = 400; //Maximum output from pid for yaw 

//GENERAL PID CALCULATIONS
float pidrolli, pidpitchi, pidyawi;
float pidrollerrord, pidpitcherrord, pidyawerrord;
float pidlastrollerrord, pidlastpitcherrord, pidlastyawerrord;
float pidrollsetpoint, pidpitchsetpoint, pidyawsetpoint;
float pidrollinput, pidpitchinput, pidyawinput; //stores the calculated data for the pids, this data is the change in angle/sec
float pidpitchoutput, pidrolloutput, pidyawoutput;

//calulating error for PID
float pid_error_temp;


boolean autolevel = true; //true indicates drone should autolevel, else...user has all the power


//NOTE: FOR ANY ARRAY, INDEX 0 IS NOT USED FOR CONVENIENCE, THEREFORE ARRAY WITH 5 INDICIES WILL STORE 4 THINGS ONLY 

//RECEIVER
byte lch1, lch2, lch3, lch4; //previous state of data from the receiver
volatile int ch1, ch2, ch3, ch4; //current state of data from receiver
int receiverinput[5];//the actual data from the receiver 
int countch1, countch2, countch3, countch4;


//ARDUINO 
byte highByte, lowByte; //retreives the MSB or LSB from an 8-bit number 
byte eeprom_data[36]; //holds the data from EEPROM of arduino  

//ESC
int esc1, esc2, esc3, esc4; //the four esc connected to Arduino
int throttle; 

//Gyro
float grollcalc, gpitchcalc, gyawcalc; //calculations done using gyro data
double groll, gpitch, gyaw; //overall calculation for angular data 

//first index = roll, second index = pitch, third index = yaw
int gaxiscalc[4]; //used for setup of gyro and initial calculations
int accaxiscalc[4]; //used for setup of accelerometer and inital calculation in terms of g force
int gaxis[4], accaxis[4];// stores the data calculated by accelerometer and gyro calculations

int temperature; //used to store the surrounding temp
float rolladjust, pitchadjust; //to alter and adjust the offsets of the gyro
int gyro_address;

float pitchangle, rollangle;// stores angles at which the quadcopter is

float pitchangleacc, rollangleacc; //stores same thing as pitchangle and pitchroll, except in units of g
long accx, accy, accz, acctotalvector; //accelerometer movement of the drone (in terms of gravity)

float pitchadjustment, rolladjustment;
boolean gyroset; //indicates if gyro is booted properly

//Counter for for loop
int a=0;


//indicates when the drone is on and off
int start;

//BATTERY VOLTAGE 
int batteryvolts;

//USER PREFRECNE
boolean auto_level=true;

//TIMERS
unsigned long looptimer; //keeps track of the time
//timers to generate pulse for each channel
unsigned long timerch1, timerch2, timerch3, timerch4; 
unsigned long esc_looptimer; //used to determing the falling edge of the pulse
unsigned long currenttime; //time to keep track of how long it has been since last input 
//timers to keep track of at what time the pulse went high for each channel
unsigned long timer1, timer2, timer3, timer4; 

void setup() {
  //load the data from EEPROM of Arduino to allow for fast acccess
  for (a=0;a<36;a++){
    eeprom_data[a]= EEPROM.read(a);
  }
  a=0; 
  gyro_address = eeprom_data[32]; //this is where the address of the gyro is stored
  Wire.begin(); //enable I2C

  //change clock speed to 400kHZ; value calculated using an equation
  TWBR = 12; 

  //All pins declared as inputs by default
  //Set required pins to output using DDR register
  DDRD |=B11110000; //setting pins 4-7 as outputs
  DDRB |=B00110000; //setting pins 12 and 13 as outputs

  //led for on and off indication connects to pin 12
  PORTD |= 1<<4;
  
  //give time for EEPROM to load...just in cade
  while(eeprom_data[33] != 'J' || eeprom_data[34] != 'M' || eeprom_data[35] != 'B')delay(10);


  //Set up MPU-6050 as the gyro 
  setupgyroregisters();


  //To begin sending power to the ESC, feed them with full current
  for (a=0;a<1250;a++){
    PORTD|=B11110000;//Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);
    PORTD &=B00001111;//Set digital poort 4, 5, 6 and 7 low.
    delayMicroseconds(3000);
  }

  //For most accurate data, take multiple samples
  for (a = 0; a < 2000 ; a ++){              //Take 2000 readings for calibration.
    if(a % 100 == 0){
      digitalWrite(12, !digitalRead(12));      //change state of LED every 100ms
    }
    adjustgyro();                                           //adjusts the gyro offsets once 2000 values have been read, otherwise just reads in the values
    gaxiscalc[1] += gaxis[1];                                //Add roll value to calculations of roll angle
    gaxiscalc[2] += gaxis[2];                              //Add roll value to calculations of pitch angle
    gaxiscalc[3] += gaxis[3];                                  //Add roll value to calculations of yaw angle

    //To prevent the ESC from beeping while gyro calibrates, send them with 1000microseconds pulse
    PORTD |= B11110000;  ////Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);//wait 1000 us
    PORTD &=B00001111;//set ports back to low
    delay(3);
  }

  //get the average of 2000 values to make it usuable data 
  grollcalc /=2000;
  gpitchcalc/=2000;
  gyawcalc /2000;

  //Enable pin-change interrupt and set the pins accordingly to the receiver
  PCICR |= (1 << PCIE0);                                   //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                   //PCINT0 =  (digital input 8) 
  PCMSK0 |= (1 << PCINT1);                                   // PCINT1 =(digital input 9)
  PCMSK0 |= (1 << PCINT2);                                   // PCINT2  = (digital input 10)
  PCMSK0 |= (1 << PCINT3);                                   //PCINT3 = (digital input 11)

  //Wait until receiver powers on and ensure throttle is in its lowest position
  //also ensure the turning sticks are in the centre
  //Throttle is indicated as channel 3
  while (ch3<990 || ch3>1020 || ch4<1400){
   //calls the function to convert the receiver signals to the output data for ESC
    ch3 = convertchdata(3);
    ch4 = convertchdata(4); 
    start++;

    //to stop the ESC from beeping, feed them with 1000us pulse
    PORTD |=B11110000;     //make ports 4,5,6,7 high
    delayMicroseconds(1000);
    PORTD &= B00001111; //make the ports low after 1000us
    delay(3);
    if (start == 125){  //once the loop repeats 125 times, change the state of the LED
      digitalWrite  (12, !digitalRead(12));
      start = 0;
    }  
  }
  start =0;

  //full battery is 12.6V
  //therefore at 12.6V, the arduino will read 1023
  //1260/1023 = 1.2317
  //volatge compensation for diode is 65

  batteryvolts = (analogRead(0)+65)*1.2317;
  looptimer = micros(); //function sets up the timer 

  //Turn of the LED once the setup is complete
  digitalWrite(12, LOW);
  
  
}

//Main Program 

void loop() {
  //for MPU6050, the angular motion of 1deg/sec is represented bu=y 65.5
  //therefore calculations can be made accrodingly to determine the total angle of rotation
  //for example, if drone turns at 6deg/sec, then it will complete 360deg in one minute
  
  //the following code determines the input to send in to PID in deg/sec of the angle
  pidrollinput = (pidrollinput*0.7)+((groll/65.5)*0.3);
  pidpitchinput = (pidpitchinput*0.7)+((gpitch/65.5)*0.3);
  pidyawinput = (pidyawinput*0.7)+(((gyaw/65.5)*0.3));

  //Calculate the gyro angle of rotation
  //Since the MPU refreshes at a rate of 250hz and a rotation of 1deg/sec= 65.5. 
  //to calculate the angle:
  pitchangle +=gpitch*0.0000611;
  rollangle +=groll* 0.0000611;

  //the following functions are used to calculate if the rotation is a pitch calculation of a roll calculation
  //if the drone has turned, the pitch angle remain relative to the earth, not relative to the front side of the drone
  
  pitchangle-=rollangle * sin (gyaw*0.000001066);
  rollangle +=pitchangle * sin(gyaw * 0.000001066);

  //calculate accelerometer angles
  //we are basically using 'dot product' to combine all the vectors into one singal value
  acctotalvector = sqrt ((accx*accx)+(accy*accy)+(accz*accz));

  //since sin inverse requires values to be <1, therefore, to calulate the value of y angle, yacc mist be less than total value of the vector
  //put if statemnet to prevent error
  //Math functions work in rads, therefore, multiple by 57.296 to convert to degrees
  if (abs(accy) < acctotalvector){
    pitchangleacc = asin((float)accy/acctotalvector)*57.296;  //calculate the pitch angle in degrees
  }

  //no abs necessary on acctotavector because it will always be +ve
  if (abs(accx)<acctotalvector){
    rollangleacc = asin ((float)accx/acctotalvector)*-57.296;  //calculate the roll angle
  }

  //for adjustments for the offsets of the gyro
  pitchangleacc-=0.0;
  rollangleacc-=0.0;

  //MPU tends to drift after a while, therefore, we must correct the drifit using the accelerometer data
  pitchangle = pitchangle*0.9996 + pitchangleacc*0.004;
  rollangle = rollangle*0.9996 + rollangleacc * 0.0004;

  //caculate the angle corrections being made for both pitch and roll
  pitchadjustment = pitchangle*15;
  rolladjustment  = rollangle *15;

  if (!auto_level){
    pitchadjustment =0;
    rolladjustment =0;
  }


  //CALCULATIONS HAVE BEEN MADE AND NOW TO START FLYING

  //to start motors; throttle low and yaw left 
   if (ch3 <1050 && ch4 <1050){
    start=1;
   }

   //return the yaw stick to the centre 
   if (start==1 && ch3<1050 && ch4>1450){
    start=2;
   }

   //set the gyro angles to accelerometers angles during startup
   pitchangle = pitchangleacc;
   rollangle = rollangleacc;
   gyroset = true;
   
  //RESET PID CONTROLS
  pidrolli =0;
  pidrollerrord =0;
  pidpitchi =0;
  pidpitcherrord =0;
  pidyawi =0;
  pidyawerrord=0;

  // TO STOP THE MOTORS: throttle lowest and yaw to right
  if (start==2 && ch3<1050 && ch4>1950){
    start =0;
  }

  //PID ROLL
  pidrollsetpoint =0;

  if (ch1>1508){
    pidrollsetpoint = ch1 -1508;    
  }
  else if (ch1<1492){
    pidrollsetpoint = ch1 -1492;
  }

  pidrollsetpoint -=rolladjustment; //angle adjustment that needs to be made due to gyro offsets
  pidrollsetpoint/=3.0;  //dividing the data by 3 gives uf the angle in degrees


  //PID PITCH
  
  pidpitchsetpoint =0;

  if (ch2>1508){
    pidpitchsetpoint = ch2 -1508;    
  }
  else if (ch2<1492){
    pidrollsetpoint = ch2 -1492;
  }

  pidpitchsetpoint -=pitchadjustment; //angle adjustment that needs to be made due to gyro offsets
  pidpitchsetpoint/=3.0;  //dividing the data by 3 gives uf the angle in degrees

  //PID YAW
  
  pidyawsetpoint =0;

  //don't do anything if throttle is in lowest position
  if (ch3>1050){
    if (ch4>1508){
      pidyawsetpoint = ch4 -1508;   
    }
    else if (ch4<1492){
      pidyawsetpoint = ch1 -1492;
    }
    pidyawsetpoint /=3; //dividing the data by 3 gives uf the angle in degrees
  }

  //AT THIS POINT, ALL PID INPUTS ARE CALCULATED, NOW CALCULATE PID OUTPUT
  calculatepid();

  //SINCE MOTOR SPEED WILL DECREASE AS BATTERY GETS LOW, WE MUST COMPENSATE FOR IT

  //the following calculation is made by taking in the input from analog pin 0 and multiplying it by 0.09853..which is just done to reduce noise 
  batteryvolts = batteryvolts*0.92 + (analogRead(0)+65)*0.09853;

  //make digitalpin 12 go high if battery voltahe too low
  if (batteryvolts <1000 && batteryvolts >600){
    digitalWrite(12,HIGH);
  }

  throttle = ch3;

  //CALCULATE PULSE FOR EACH OF TEH ESC
  //for this to work properly, we must ensure we got flexibility of our hands
  //if all the motors are running at full speed, then they wouldn't be able to spin faster in order to turn
  //therefore we must lower the speed of all motors instead
  if (start ==2){
    if (throttle>1800){
      throttle = 1800;
    }
    esc1 = throttle - pidpitchoutput + pidrolloutput - pidyawoutput; //Calculate the pulse for esc 1 (front-right - CCW)
    esc2 = throttle + pidpitchoutput + pidrolloutput + pidyawoutput; //Calculate the pulse for esc 2 (rear-right - CW)
    esc3 = throttle + pidpitchoutput - pidrolloutput - pidyawoutput; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc4 = throttle - pidpitchoutput - pidrolloutput + pidyawoutput; //Calculate the pulse for esc 4 (front-left - CW)

    //NOW ALTER THE OUTPUT IN ACCORDANCE WITH THE BATTERY VOLTAGE 
    if (batteryvolts< 1240 && batteryvolts>800){
    esc1 += esc1 * ((1240 - batteryvolts)/(float)3500);              
    esc2 += esc2 * ((1240 - batteryvolts)/(float)3500);             
    esc3 += esc3 * ((1240 - batteryvolts)/(float)3500);             
    esc4 += esc4 * ((1240 - batteryvolts)/(float)3500);              
    }
    //LIMIT THE MAX OUTPUT FOR THE ESCS

    //keep the motors running at low speed at minimum throttle
    if (esc1 < 1100) esc1 = 1100;                                         
    if (esc2 < 1100) esc2 = 1100;                                         
    if (esc3 < 1100) esc3 = 1100;                                         
    if (esc4 < 1100) esc4 = 1100;                                         
  
    //set the max output of the pid to 2000us
    if(esc1 > 2000)esc1 = 2000;                                           
    if(esc2 > 2000)esc2 = 2000;                                           
    if(esc3 > 2000)esc3 = 2000;                                           
    if(esc4 > 2000)esc4 = 2000;    
  }
  //if the drone is not yet started or turned off, feed escs with 1000us pulse
  else{
    esc1 = 1000;                                                          
    esc2 = 1000;                                                           
    esc3 = 1000;                                                           
    esc4 = 1000; 
  }

  //THE REFRESH RATE OF ESCs is 250hz therefore we must send a pulse every 4ms to ESC
  //set an internal timer for 4ms

  if (micros() - looptimer > 4050){
    digitalWrite(12,HIGH); //warning light indicating time exceeded the set timer
  }

  while (micros() - looptimer<4000){
   looptimer = micros();
  }

  PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.

  //calculate the time for the falling-edge for each esc
  timerch1 = esc1 + looptimer;                                     
  timerch2 = esc2 + looptimer;                                     
  timerch3 = esc3 + looptimer;                                    
  timerch4 = esc4 + looptimer;                                     
  
  //adjust gyro once more
  adjustgyro();

  //Send the output to the ESCs
  while(PORTD >= 16){                                                       //Stay in this loop until output 4,5,6 and 7 are low.
    esc_looptimer = micros();                                              //Read the current time.
    if(timerch1 <= esc_looptimer){
      PORTD &= B11101111;                //Set digital output 4 to low if the time is expired.
    }
    if(timerch2 <= esc_looptimer){
      PORTD &= B11011111;                //Set digital output 5 to low if the time is expired.
    }
    if(timerch3 <= esc_looptimer){
      PORTD &= B10111111;                //Set digital output 6 to low if the time is expired.
    }
    if(timerch4 <= esc_looptimer){
      PORTD &= B01111111;                //Set digital output 7 to low if the time is expired.
    }
  }
  

  
}
  
//Gyro code closely follows joop brokking tutorial as specific registers must be communicated with 
//sets up the register in gyro 
void setupgyroregisters(){
  //SETUP THE MPU-6050

  //Start communication with the gyro
  Wire.beginTransmission(gyro_address);                                      
  Wire.write(0x6B);                                                          //write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                                                          //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();                                                    //End the transmission with the gyro.

  //start transmission once more
  Wire.beginTransmission(gyro_address);                                      
  Wire.write(0x1B);                                                          //write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x08);                                                          //Set the register bits as 00001000 (500dps full scale)
  Wire.endTransmission();                                                    //End the transmission with the gyro

  Wire.beginTransmission(gyro_address);                                     
  Wire.write(0x1C);                                                          //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x10);                                                          //Set the register bits as 00010000 (+/- 8g full scale range)//gravity sensations (accelerometer
  Wire.endTransmission();                                                    //End the transmission with the gyro

  //check if the values are written correct
  Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search
  Wire.write(0x1B);                                                          //Start reading @ register 0x1B
  Wire.endTransmission();                                                    //End the transmission
  Wire.requestFrom(gyro_address, 1);                                         //Request 1 bytes from the gyro
  while(Wire.available() < 1);                                               //Wait until the 6 bytes are received

  //Address must be 0x08, else the program will not run
  if(Wire.read() != 0x08){                                                   //Check if the value is 0x08
    digitalWrite(12,HIGH);                                                   //Turn on the warning led
    while(1)delay(10);                                                       //Stay in this loop for ever
  }


  Wire.beginTransmission(gyro_address);                                      
  Wire.write(0x1A);                                                          //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                          //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz) //reduces noise 
  
  Wire.endTransmission();                                                    //End the transmission with the gyro    
}

//convert_receiver_channel
//convert receiver signals to signals between 1000 - 1500- 2000 us
int convertchdata(byte ch){
  byte channel, reverse;
  int low, centre, high, presentdata;
  int difference;

  //data retrevied from eeprom from setup
  channel = eeprom_data[ch + 23] & 0b00000111;                          

  if(eeprom_data[ch + 23] & 0b10000000){
    reverse = 1;        //Reverse channel when most significant bit is set
  }
  else {  //else the reverse setting is off
    reverse = 0;   
  }
  
  presentdata = receiverinput[ch];
  low = (eeprom_data[channel * 2 + 15] << 8) | eeprom_data[channel * 2 + 14];  //Store the low value for the specific receiver input channel
  centre = (eeprom_data[channel * 2 - 1] << 8) | eeprom_data[channel * 2 - 2]; //Store the center value for the specific receiver input channel
  high = (eeprom_data[channel * 2 + 7] << 8) | eeprom_data[channel * 2 + 6];   //Store the high value for the specific receiver input channel

  if (presentdata <centre){
    if (presentdata <low) presentdata = low;
    //sets the limit to th present data so it doesn't go lower than actual data. 
    //this line is based on what the current signal is and how far it is from the centre
    difference = (long(centre - presentdata))*(long)(500)/(centre-low); //this calculation scales the data to 1000-2000 values...which is sent in as a pulse to ESC 
    if (reverse==1){  //if the user has reversed the controller stick movement, decrease instead of increase and vice versa
      return 1500+difference;
    }
    else{
      return 1500 - difference; 
    } 
  }

  else if (presentdata > centre){
    if (presentdata>high){  //set a limit to what the highest data can be 
      presentdata = high; 
    }

    //perform the caluclation
    //in this case we know data will be somewhere above center
    //so find where it by dividing it by the total range between center and high 
    
    difference = ((long)(presentdata - centre) * (long)500) / (high-centre); 

    if (reverse ==1){
      return 1500 - difference;
    }
    else{
      return 1500+difference;
    }
    
  }

  
}

void adjustgyro(){
  //read MPU-6050
  if (eeprom_data[31] ==1){
    Wire.beginTransmission(gyro_address); //start communication with gyro
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(gyro_address, 14);  //request 14 bytes from the gyro
    
    
    ch1 = convertchdata(1); //pitch
    ch2 = convertchdata(2); //roll
    ch3 = convertchdata(3); //throttle
    ch4 = convertchdata(4); //yaw

    while (Wire.available() <14); //don't continue until gyro sends in 14 bytes

    accaxis[1] = Wire.read()<<8|Wire.read();                               //calulate acc_x data
    accaxis[2] = Wire.read()<<8|Wire.read();                               //calulate acc_y data
    accaxis[3] = Wire.read()<<8|Wire.read();                               //calulate acc_z data
    temperature = Wire.read()<<8|Wire.read();                               // temperature calculation from gyro.
    gaxis[1] = Wire.read()<<8|Wire.read();                              // angular data  - pitch.
    gaxis[2] = Wire.read()<<8|Wire.read();                              //angular data - roll.
    gaxis[3] = Wire.read()<<8|Wire.read();                              //angular data - yaw. 

  //after the calibration, you must compensate for drifiting data 
    if (a ==2000){
      gaxis[0] -=gaxiscalc[0]; 
      gaxis[1] -=gaxiscalc[1];
      gaxis[2] -=gaxiscalc[2];
    }

  groll = gaxis[eeprom_data[28] & 0b00000011];                      //Set gyro_roll to the correct axis 
  if(eeprom_data[28] & 0b10000000){
    groll *= -1;                          
  }
  gpitch = gaxis[eeprom_data[29] & 0b00000011];                     //Set gyro_pitch to the correct axis 
  if(eeprom_data[29] & 0b10000000){
    gpitch *= -1;                         
  }
  gyaw = gaxis[eeprom_data[30] & 0b00000011];                       //Set gyro_yaw to the correct axis 
  if(eeprom_data[30] & 0b10000000){
    gyaw *= -1;                        
  }

  accx = accaxis[eeprom_data[29] & 0b00000011];                           //Set acc_x to the correct axis 
  if(eeprom_data[29] & 0b10000000){
    accx *= -1;                             
  }
  accy = accaxis[eeprom_data[28] & 0b00000011];                           //Set acc_y to the correct axis 
  if(eeprom_data[28] & 0b10000000){
    accy *= -1;                            
  }
  accz = accaxis[eeprom_data[30] & 0b00000011];                           //Set acc_z to the correct axis 
  if(eeprom_data[30] & 0b10000000){
    accz *= -1;                              
  }
    
  }
}

//little is understood about such complex algorithm, therfore, this segment closely follows joop brokkin's tutorial
//this methods takes into consideration the present, past error and calculates the future error.
//this methods tries to make the input value reach the setpoint, however, since no system is perfect, there is always going to be a small error
//this specific PID works upon the calculations made upon error. 
void calculatepid(){
  //Roll calulations
  pid_error_temp = pidrollinput - pidrollsetpoint;
  pidrolli += pid_igain_roll * pid_error_temp;
  if(pidrolli > pid_max_roll)pidrolli = pid_max_roll;
  else if(pidrolli < pid_max_roll * -1)pidrolli = pid_max_roll * -1;

  pidrolloutput = pid_pgain_roll * pid_error_temp + pidrolli + pid_dgain_roll * (pid_error_temp - pidlastrollerrord);
  if(pidrolloutput > pid_max_roll)pidrolloutput = pid_max_roll;
  else if(pidrolloutput < pid_max_roll * -1)pidrolloutput = pid_max_roll * -1;

  pidlastrollerrord = pid_error_temp;

  //Pitch calculations
  pid_error_temp = pidpitchinput - pidpitchsetpoint;
  pidpitchi += pid_igain_pitch * pid_error_temp;
  if(pidpitchi > pid_max_pitch)pidpitchi = pid_max_pitch;
  else if(pidpitchi < pid_max_pitch * -1)pidpitchi = pid_max_pitch * -1;

  pidpitchoutput = pid_pgain_pitch * pid_error_temp + pidpitchi + pid_dgain_pitch * (pid_error_temp - pidlastpitcherrord);
  if(pidpitchoutput > pid_max_pitch)pidpitchoutput = pid_max_pitch;
  else if(pidpitchoutput < pid_max_pitch * -1)pidpitchoutput = pid_max_pitch * -1;

  pidlastpitcherrord = pid_error_temp;

  //Yaw calculations
  pid_error_temp = pidyawinput - pidyawsetpoint;
  pidyawi += pid_igain_yaw * pid_error_temp;
  if(pidyawi > pid_max_yaw)pidyawi = pid_max_yaw;
  else if(pidyawi < pid_max_yaw * -1)pidyawi = pid_max_yaw * -1;

  pidyawoutput = pid_pgain_yaw * pid_error_temp + pidyawi + pid_dgain_yaw * (pid_error_temp - pidlastyawerrord);
  if(pidyawoutput > pid_max_yaw)pidyawoutput = pid_max_yaw;
  else if(pidyawoutput < pid_max_yaw * -1)pidyawoutput = pid_max_yaw * -1;

  pidlastyawerrord = pid_error_temp;
} 

//pinchange interrupt
//gets triggered every time pins 8-11 change state
//receiver is connected to these pins
ISR (PCINT0_vect){
  //record current time
  currenttime = micros();
  //Channel 1=========================================
  if(PINB & B00000001){                                                     
    if(lch1 == 0){                                                //if Input 8 changed from 0 to 1.
      lch1 = 1;                                                   //Remember current input state.
      timer1 = currenttime;                                               //Set timer1 to currenttime.
    }
  }
  else if(lch1 == 1){                                             //Input 8 is not high and changed from 1 to 0.
    lch1 = 0;                                                     //Remember current input state.
    receiverinput[1] = currenttime - timer1;                             //Channel 1 is currenttime - timer1.
  }
  //Channel 2=========================================
  if(PINB & B00000010 ){                                                    
    if(lch2 == 0){                                                //if Input 9 changed from 0 to 1.
      lch2 = 1;                                                   //Remember current input state.
      timer2 = currenttime;                                               //Set timer2 to currenttime.
    }
  }
  else if(lch2 == 1){                                             //Input 9 is not high and changed from 1 to 0.
    lch2 = 0;                                                     //Remember current input state.
    receiverinput[2] = currenttime - timer2;                             
  }
  //Channel 3=========================================
  if(PINB & B00000100 ){                                                   
    if(lch3 == 0){                                                //if Input 10 changed from 0 to 1.
      lch3 = 1;                                                   //Remember current input state.
      timer3 = currenttime;                                               //Set timer3 to currenttime.
    }
  }
  else if(lch3 == 1){                                             //Input 10 is not high and changed from 1 to 0.
    lch3 = 0;                                                     //Remember current input state.
    receiverinput[3] = currenttime - timer3;                             

  }
  //Channel 4=========================================
  if(PINB & B00001000 ){                                                    
    if(lch4 == 0){                                                //if Input 11 changed from 0 to 1.
      lch4 = 1;                                                   //Remember current input state.
      timer4 = currenttime;                                               
    }
  }
  else if(lch4 == 1){                                             //Input 11 is not high and changed from 1 to 0.
    lch4 = 0;                                                     //Remember current input state.
    receiverinput[4] = currenttime - timer4;                          
  }
  
  
}
