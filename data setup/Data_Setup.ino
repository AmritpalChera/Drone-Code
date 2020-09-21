/*Amritpal Chera
 * 2019-05-28
 * This code is to setup the Arduino and upload all the data to its EEPROM for later use. 
 * It measures all the gyro offsets and records them for later use
 * It measures the reciver signals and records their values for easy access
 * Special Credit to Joop Brokking** for his code; this code is influenced by his code on a similar quadcopter.
 * Major changes include that this code is specifically meant for hardware that is in current use and is not generalized for all hardware
 */

 //Import libraries
#include <Wire.h>  //includes the wire library to allow communication with gyro
#include <EEPROM.h> //includes EEPROM library to allow us to stroe info on EEPROM 

//GLOBAL VARIIABLES
boolean success=true; //indicates if there was error or not
volatile int ch1, ch2, ch3, ch4; //the four channels of receiver 
//WE must also store the middle, max and min values for each channel 
int cench1, cench2, cench3, cench4; //center values for all channels
int maxch1, maxch2, maxch3, maxch4; //max values for each channel 
int minch1, minch2, minch3, minch4; //min values for each channel 
byte lastch1, lastch2, lastch3, lastch4; //stores the state of the four channels

//used to identify what channel caused the interrupt
byte ch1assign, ch2assign, ch3assign, ch4assign;

byte clkspeed_ok; //indicates if the I2C speed was set successfully
byte receiver_identify; //identifies what channel triggered the interrupt
byte gyroaddress; //gyroaddress of the gyro

//TIMERS
unsigned long timer; //used for while loops to give the user a specific amount of time to complete an action
unsigned long timer1, timer2, timer3, timer4; //timers for each of te channels, read in the pulse length when it is high
unsigned long currenttime; //used in comparison to channel timers to inidicate high pulse length


//GYRO
float grollcalc, gpitchcalc, gyawcalc; //calculations done using gyro data
float groll, gpitch, gyaw; //overall calculation
byte gyro_check_byte;//checks the input data from the gyro

//roll axies, pitch axis, yaw axes
byte raxis, paxis, yaxis; //used to indicate what movement corresponds to what angle
int a; //counter for 'for' loops

//Receiver
int receiverinput[5]; //the inputs from the receiver
 
void setup() {
  //set register for pin change interrupts using Arduino registers 
  //PCMSK0 is the register that allows us to use pin change interrupts
  //PCICR is pin change interrupt scanner and PCIE is the command to enable it 
  //We need this for the receiver 
  //The pins are set to inputs by default, so no need to declare that seperately
  PCICR |= (1<<PCIE0); //enable the scanner for digital pins 8 to 13
  //set pins 8-11 to trigger interrupt when the state of any of those pins is changed 
  PCMSK0 |= (1 << PCINT0);  // digital pin 8 
  PCMSK0 |= (1 << PCINT1);  // digital pin 9
  PCMSK0 |= (1 << PCINT2);  // digital pin 10
  PCMSK0 |= (1 << PCINT3);  //digital pin 11
  //AT THIS POINT, THOSE PINS ARE OUR INTERUPT PINS FOR RECEIVER

  //Wire library allows us to use pins A4 and A5 as SCL and SDA
  //SCL and SDA are pins used to read in serial data with clock pulse
  Wire.begin();
  Serial.begin(57600);
  delay(250); //time for gyro to begin its transmission to arduino

}

void loop() {
  //ALL THE PRINT STATEMENTS WILL BE WRITTEN TO FLASH MEMORY IN ORDER TO SAVE SPACE ON SRAM
  
  //set the I2C speed here
  Serial.println(F("Setting I2C speed to 400MHz"));
  delay(1000);
  TWBR = 12; //this number is gotten through using an equation

  if (F_CPU ==1600000L){ //Clock speed of Arduino
    clkspeed_ok = 1;
  }

  if (TWBR ==12 && clkspeed_ok){
    Serial.println(F("Speed set!"));
    success=true;
  }
  else {
    Serial.println(F("Operation failed"));
    success=false;
  }

  //check the validity of the receiver here
  if (success){
    Serial.println(F("RECEIVER SETUP"));
    delay(1000);
    Serial.println(F("Check for valid receiver signals"));
    //checkReceiverSignals(); //there might be an error while checking
    checkReceiverSignals();
  }

  //store the centre positions of receiver in this 
  if (success){
    delay(2000);
    Serial.println(F("Place all sticks in the center position"));
    //provide for 10 secs to complete the task 
    Serial.println(F("You have 10 Seconds"));
    for (int a=0;a<9;a++){
      Serial.println(a);
      delay(1000);
    }

    //store the center positions
    cench1 = ch1;
    cench2 = ch2;
    cench3 = ch3;
    cench4 = ch4;

    Serial.println(F("Centre positions stored"));
    Serial.print(F("Pin 8/ ch1: "));
    Serial.println(cench1);
    Serial.print(F("Pin 9/ ch 2"));
    Serial.println(cench2);
    Serial.print(F("Pin 10/ch 3"));
    Serial.println(cench3);
    Serial.print(F("Pin 11/ch 4"));
    Serial.println(cench4);
    Serial.print(F(""));   
  }
//*****************************************SOME CODE REMOVED FROM HERE

  //now we will establish what sticks on controller correspond to what
  if (success){
    Serial.print(F("Move Throttle to full and back to centre"));
    checkReceiverInput(1);
    waitsticksneutral();

    Serial.print(F("Move roll stick to simulate left wing up and then recentre"));
    checkReceiverInput(2);
    waitsticksneutral();

    Serial.print(F("Move pitch stick to simulate nose up and then recentre"));
    checkReceiverInput(3);
    waitsticksneutral();
    
    Serial.print(F("Move yaw stick to simulate nose moving right  and then recentre"));
    checkReceiverInput(4);
    waitsticksneutral();  
  }

  
  if (success){
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Gently move all the sticks simultaneously to their extends"));
    Serial.println(F("When ready put the sticks back in their center positions"));
    storeendpoint();
    Serial.println(F(""));
    Serial.println(F("High, low, center values stored "));
    Serial.println(F(""));

    //print out ch1 data
    Serial.println(F("Pin 8/ Channel 1: "));
    Serial.print(F("min: "));
    Serial.println(minch1);
    Serial.print(F("max: "));
    Serial.println(maxch1);
    Serial.print(F("center"));
    Serial.println(cench1);
    delay(3000);

    //print out ch2 data
    Serial.println(F("Pin 9/ Channel 2: "));
    Serial.print(F("min: "));
    Serial.println(minch2);
    Serial.print(F("max: "));
    Serial.println(maxch2);
    Serial.print(F("center"));
    Serial.println(cench2);
    //delay(3000);

    //print out ch3 data
    Serial.println(F("Pin 10/ Channel 3: "));
    Serial.print(F("min: "));
    Serial.println(minch3);
    Serial.print(F("max: "));
    Serial.println(maxch3);
    Serial.print(F("center"));
    Serial.println(cench3);
    //delay(3000);

    //print out ch1 data
    Serial.println(F("Pin 8/ Channel 4: "));
    Serial.print(F("min: "));
    Serial.println(minch4);
    Serial.print(F("max: "));
    Serial.println(maxch4);
    Serial.print(F("center"));
    Serial.println(cench4);
    //delay(3000);

    Serial.print(F("Move stick 'nose up' and back to center to continue"));
    continued();
    
    
  }

  if (success){
    //connect the gyro
    Serial.println(F(""));
    Serial.println(F("Gyro Search"));
    delay(2000);
    Serial.println(F("Search for MPU-6050"));
    //MPU6050 usually connects with the gyroaddress 0x68
    gyroaddress = 0x68; 

    delay(3000);
    Serial.println(F(""));
    Serial.println(F("Configure Gyro for startup"));
    begingyro();  //configuers the gyro-registers 
    Serial.println(F("Done"));
  }

  if (success){
    delay(3000);
    Serial.println(F("Gyro calibration"));
    Serial.println(F("Don't move the quadcopter. Calibration starts in 3 seconds"));
    delay(3000);
    Serial.println(F("Starting Calibration, wait 10 seconds"));

    //For most accurate data, take multiple samples
    for (a = 0; a < 2000 ; a ++){              //Take 2000 readings for calibration.
      if(a % 100 == 0){
        Serial.print(F("."));                //Print dot to indicate calibration.
      }
      adjustgyro();                                           //adjusts the gyro offsets once 2000 values have been read, otherwise just reads in the values
      grollcalc += groll;                                //Add roll value to calculations of roll angle
      gpitchcalc += gpitch;                              //Add roll value to calculations of pitch angle
      gyawcalc += gyaw;                                  //Add roll value to calculations of yaw angle
      delay(3);                                                  //Wait 3 milliseconds before the next loop.
    }

    //get the average of 2000 values to make it usuable data 
    grollcalc /=2000;
    gpitchcalc/=2000;
    gyawcalc /2000;

    //Print the results 
    Serial.println(F(""));
    Serial.print(F("Axis 1 offset="));
    Serial.println(grollcalc);
    Serial.print(F("Axis 2 offset="));
    Serial.println(gpitchcalc);
    Serial.print(F("Axis 3 offset="));
    Serial.println(gyawcalc);
    Serial.println(F(""));

  }
    Serial.println(F("Gyro axies configuration: "));
    delay(5000); //slow down the process a little bit

    //deal with left wing movement first 
    //left wing up is axis movement 1 (roll)
    //front nost up is axis movement 2 (pitch)
    //yaw axis / rotation axis is movement 3; this movement is only measured in rotation per unit time (angular motion)

    Serial.println(F("Lift the left side of the quadcopter to a 45 degree angle within 10 seconds"));
    check_gyro_axes(1);
    if(success){
       Serial.println(F("good"));
       Serial.println(F("Put the quadcopter back in its original position"));
       Serial.println(F("Move stick 'nose up' and back to center to continue"));
       continued();
    }

    Serial.println(F("Lift the nose of the quadcopter to a 45 degree angle within 10 seconds"));
    check_gyro_axes(2);
    if(success){
       Serial.println(F("good"));
       Serial.println(F("Put the quadcopter back in its original position"));
       Serial.println(F("Move stick 'nose up' and back to center to continue"));
       continued();
    }


    Serial.println(F("ROtate the nose of the quadcopter to a 45 degree angle within 10 seconds"));
    check_gyro_axes(3);
    if(success){
       Serial.println(F("good"));
       Serial.println(F("Put the quadcopter back in its original position"));
       Serial.println(F("Move stick 'nose up' and back to center to continue"));
       continued();
    }

    if (success){
      Serial.println(F("final setup check"));
      delay(1000);
      if(receiver_identify == 0b00001111){
      Serial.println(F("Receiver channels ok"));
      }
      else{
        Serial.println(F("Receiver channel verification failed!"));
        success=false;
      }
      delay(1000);
      if(gyro_check_byte == 0b00000111){
         Serial.println(F("Gyro axes ok"));
      }
      else{
        Serial.println(F("Gyro axes verification failed!"));
        success=false;
      }
      
    }

    //if no errors exists, write all data to EEPROM for fast access
    if (success){
      Serial.println(F("Writing EEPROM"));

      //THE NEXT PART CLOSELY FOLLOWS JOOP BROKKING WRITING CODE AS THE DATA WRITTEN IS USED DURING FLIGHT

      //THE EEPROM IS BASICALLY LIKE A HARD DRIVE OF ARDUINO
      //WE ARE BASICALLY WRITTING DATA TO EACH ONE OF ITS CELLS LIKE IN A CD
      //THESE CELLS WILL BE REFERRED TO AFTERWARDS TO RETREIVE STORED DATA
      
      EEPROM.write(0, cench1 & 0b11111111);
      EEPROM.write(1, cench1 >> 8);
      EEPROM.write(2, cench2 & 0b11111111);
      EEPROM.write(3, cench2 >> 8);
      EEPROM.write(4, cench3 & 0b11111111);
      EEPROM.write(5, cench3 >> 8);
      EEPROM.write(6, cench4 & 0b11111111);
      EEPROM.write(7, cench4 >> 8);
      EEPROM.write(8, maxch1 & 0b11111111);
      EEPROM.write(9, maxch1 >> 8);
      EEPROM.write(10, maxch2 & 0b11111111);
      EEPROM.write(11, maxch2 >> 8);
      EEPROM.write(12, maxch3 & 0b11111111);
      EEPROM.write(13, maxch3 >> 8);
      EEPROM.write(14, maxch4 & 0b11111111);
      EEPROM.write(15, maxch4 >> 8);
      EEPROM.write(16, minch1& 0b11111111);
      EEPROM.write(17, minch1>> 8);
      EEPROM.write(18, minch2 & 0b11111111);
      EEPROM.write(19, minch2 >> 8);
      EEPROM.write(20, minch3 & 0b11111111);
      EEPROM.write(21, minch3 >> 8);
      EEPROM.write(22, minch4 & 0b11111111);
      EEPROM.write(23, minch4 >> 8);
      EEPROM.write(24, ch1assign);
      EEPROM.write(25, ch2assign);
      EEPROM.write(26, ch3assign);
      EEPROM.write(27, ch4assign);
      EEPROM.write(28, raxis);
      EEPROM.write(29, paxis);
      EEPROM.write(30, yaxis);
      EEPROM.write(31, 1);
      EEPROM.write(32, gyroaddress);
      //Write the EEPROM signature
      EEPROM.write(33, 'J'); 
      EEPROM.write(34, 'M');
      EEPROM.write(35, 'B');
          
      
      //To make sure evrything is ok, verify the EEPROM data.
      //check if the data at a specific cells matches the data that was to be written
      //if not the case, a problem had occured, restart the setup
      Serial.println(F("Verify EEPROM data"));
      delay(1000);
      if(cench1 != ((EEPROM.read(1) << 8) | EEPROM.read(0)))success=false;
      if(cench2 != ((EEPROM.read(3) << 8) | EEPROM.read(2)))success=false;
      if(cench3 != ((EEPROM.read(5) << 8) | EEPROM.read(4)))success=false;
      if(cench4 != ((EEPROM.read(7) << 8) | EEPROM.read(6)))success=false;
      
      if(maxch1 != ((EEPROM.read(9) << 8) | EEPROM.read(8)))success=false;
      if(maxch2 != ((EEPROM.read(11) << 8) | EEPROM.read(10)))success=false;
      if(maxch3 != ((EEPROM.read(13) << 8) | EEPROM.read(12)))success=false;
      if(maxch4 != ((EEPROM.read(15) << 8) | EEPROM.read(14)))success=false;
      
      if(minch1!= ((EEPROM.read(17) << 8) | EEPROM.read(16)))success=false;
      if(minch2 != ((EEPROM.read(19) << 8) | EEPROM.read(18)))success=false;
      if(minch3 != ((EEPROM.read(21) << 8) | EEPROM.read(20)))success=false;
      if(minch4 != ((EEPROM.read(23) << 8) | EEPROM.read(22)))success=false;
      
      if(ch1assign != EEPROM.read(24))success=false;
      if(ch2assign != EEPROM.read(25))success=false;
      if(ch3assign != EEPROM.read(26))success=false;
      if(ch4assign != EEPROM.read(27))success=false;
      
      if(raxis != EEPROM.read(28))success=false;
      if(paxis != EEPROM.read(29))success=false;
      if(yaxis != EEPROM.read(30))success=false;
      if(gyroaddress != EEPROM.read(32))success=false;
      
      if('J' != EEPROM.read(33))success=false;
      if('M' != EEPROM.read(34))success=false;
      if('B' != EEPROM.read(35))success=false;
    
      if(success == false)Serial.println(F("EEPROM verification failed!"));
      else Serial.println(F("Verification done"));   
    }

    if (success){
      Serial.println(F("Setup is finished."));
  }

  
  
}

//checks if the signals from receiver are valid. 
//this means pulse should be within 1000 - 2000 milliseconds. 
void checkReceiverSignals(){
  byte value =0;
  timer = millis()+10000; //generates 10-seconds timer period when combined with the while loop below

  while (timer>millis() && value<15){
    //check if the time pulse is within 2100ms and 900ms
    //if it is, then it is good, otherwise indicate error
    if (ch1 <2100 && ch1 >900){
      value |= 0b00000001; //pin 8 in binary 
    }
    if (ch2 <2100 && ch2 >900){
      value |= 0b00000010; //pin 9 in binary 
    }
    if (ch3 <2100 && ch3 >900){
      value |= 0b00000100; //pin 10 in binary 
    }
    if (ch4 <2100 && ch4 >900){
      value |= 0b00000001; //pin 11 in binary 
    }
    delay(500);

    if (value ==0){
      success=false;
      Serial.println(F("Signals not valid from receiver"));
    }
    else Serial.println(F("Receiver Signals valid"));
  }
}


//to check the movement for each stick/ channel
//movement =1 for throttle
//movement = 2 for fly right (left side up)
//movement =3 for fly back (front side up)
//movement =4 for clockwise rotation in the air 
void checkReceiverInput(byte movement){
  Serial.print(F("Movement: "));
  Serial.print(movement);
  byte trigger =0;
  int pulse_length;
  timer = millis()+30000; //accounts for 30seconds when combined with the while loop
  while (timer >millis() && trigger ==0){
    delay (250);

   //if the receiver values are valid 
    if (ch1>1750 || ch1 <1250){
      trigger = 1; //interrupt triggered by ch1
      receiver_identify  |= 0b00000001; //what pin on arduino triggered interrupt
      pulse_length = ch1;
    }
    //channel 2 inputs 
    if (ch2>1750 || ch2 <1250){
      trigger = 2; //interrupt triggered by ch2
      receiver_identify  |= 0b00000010; //what pin on arduino triggered interrupt
      pulse_length = ch2;
    }

    
  //if channel 3 values are alright
    if (ch3>1750 || ch3 <1250){
      trigger = 3;  //interrupt triggered by ch3
      receiver_identify  |= 0b00000100;//what pin on arduino triggered interrupt
      pulse_length = ch3;
    }
    //if channel 4 inputs are valid
    if (ch4>1750 || ch4 <1250){
      trigger = 1; //interrupt triggered by ch4
      receiver_identify  |= 0b00001000; //what pin on arduino triggered interrupt
      pulse_length = ch4;
    }
  }


  //check for any error during indentifying what channel caused the trigger
  if (trigger==0){
    success=false;
    Serial.println(F("\nStick movement not detected"));
  }
  else{
    
    if(movement == 1){
      ch3assign = trigger;
      Serial.println(F(" is connected  to ch3"));
      if(pulse_length < 1250){
        ch3assign += 0b10000000;
      }
    }
    
    if(movement == 2){
      ch1assign = trigger;
      Serial.println(F(" is connected  to ch1"));
      if(pulse_length < 1250){
        ch1assign += 0b10000000;
      }
    }
 
    if(movement == 3){
      ch2assign = trigger;
      Serial.println(F(" is connected  to ch2"));
      if(pulse_length < 1250){
        ch2assign += 0b10000000;
      }
    }
    
    if(movement == 4){
      ch4assign = trigger;
      Serial.println(F(" is connected  to ch4"));
      if(pulse_length < 1250){
        ch4assign += 0b10000000;
      }
    }
  }   
}


//stores the end points of all channels
void storeendpoint(){
  byte zero=0;

  //set the currrent receiver inputs as low inputs
  minch1 = ch1;
  minch2 = ch2;
  minch3 = ch3;
  minch4 = ch4;
  Serial.println(F("Move channel 1 to center"));
  while (ch1<cench1+20 && ch1>cench1-20){
    delay(300);
  }
  Serial.println(F("Measuring EndGame of sticks"));
  //the variable is incremented using the OR operation
  //as we write numbers in binary, it is easy to tell what we are writing
  //when all four inputs are received, the total value of zero = 15. 
  while (zero<15){
    //if the sticks moved away from the centre, record momement
    //channel 1 movement
    if (ch1<cench1+20 && ch1>cench1-20){
      zero |= 0b00000001;
    }
    //channel 2 movement
    if (ch2<cench2+20 && ch2>cench2-20){
      zero |= 0b00000010;
    }
    //channel 3 movement
    if (ch3<cench3+20 && ch3>cench3-20){
      zero |= 0b00000100;
    }
    //channel 4 movement
    if (ch4<cench4+20 && ch4>cench4-20){
      zero |= 0b00001000;
    }

    //SETTING LOWEST VALUE FOR ALL CHANNELS

    //if lowest value for channel 1 is less than previously recorded value
    if(ch1 < minch1){
      minch1 = ch1;
    }
    if(ch2 < minch2){
      minch2 = ch2;
    }
    if(ch3 < minch3){
      minch3 = ch3;
    }
    if(ch4 < minch4){
      minch4 = ch4;
    }
    //SETTING HIGHEST VALUE FOR ALL CHANNELS 

    //if highest value for channel 1 is higher than previously recorded value
    if(ch1 > maxch1){
      maxch1 = ch1;
    }
    if(ch2 > maxch2){
      maxch2 = ch2;
    }
    if(ch3 > maxch3){
      maxch3 = ch3;
    }
    if(ch4 > maxch4){
      maxch4 = ch4;
    }
    delay(100);
    
  }
   
}

//**********code missing from here (continued function):

void continued(){
  byte con  =0;
  while (con==0){
    if (ch1>cench1+150 || ch1<cench1-150){
      con=1;
    }
    /*
    if (ch2>cench2+150 || ch2<cench2-150){
      con=1;
    }
    if (ch3>cench3+150 || ch3<cench3-150){
      con=1;
    }
    if (ch4>cench4+150 || ch4<cench4-150){
      con=1;
    }
    */
    delay(100);
  }
  waitsticksneutral(); //wait for the sticks to return to the centre
  
}

//function keeps running until sticks return to center
void waitsticksneutral(){
  Serial.println(F("Return sticks to center"));
  byte zero = 0;
  //once all the sticks are moved to centre, the while loop will end once zero =15;
  while(zero < 15){
    //if channel one is moved to center, OR zero value
    if(ch1 < cench1 + 20 && ch1 > cench1 - 20){
      zero |= 0b00000001;
    }
    //if channel two is moved to center, OR zero value
    if(ch2 < cench2 + 20 && ch2 > cench2 - 20){
      zero |= 0b00000010;
    }
    //if channel three is moved to center, OR zero value
    if(ch3 < cench3 + 20 && ch3 > cench3 - 20){
      zero |= 0b00000100;
    }
    //if channel four is moved to center, OR zero value
    if(ch4 < cench4 + 20 && ch4 > cench4 - 20){
      zero |= 0b00001000;
    }
    delay(100);
  }
}

/*
 * 
 * Continue here
 * 
 */

//starting process to configure the gyro
//specific process must be followed, therefore, closely follows YMFC code 
void begingyro(){
  Wire.beginTransmission(gyroaddress);  //Start communication with the gyro
  Wire.write(0x6B);  //congigure the register 
  Wire.write(0x00);
  Wire.endTransmission();  //end communication

  Wire.beginTransmission(gyroaddress);                             //Start communication with the gyro
  Wire.write(0x6B);                                            //this is gyroaddress of where the gyro is connected
  Wire.endTransmission();  //end communication
  Wire.requestFrom(gyroaddress, 1); //requesting one byte of data from gyro       

  while (Wire.available()<1); //don't continue unit gyro sends in atleast one bit of data 
  Serial.print(F("Register 0x6B is set to:"));

  /*
   * 
   * BIN NOT FULLY UNDERSTOOD
   * 
   */
  Serial.println(Wire.read(),BIN);
  Wire.beginTransmission(gyroaddress);    //begin the transmission again                         
  Wire.write(0x1B);                     //congigure the register 
  Wire.write(0x08);                    
  Wire.endTransmission();                 //End the transmission
    
  Wire.beginTransmission(gyroaddress);                             //Start communication with the gyro (gyroaddress 1101001)
  Wire.write(0x1B);                                            //Start readingfrom (0x1B) register
  Wire.endTransmission();                                      //End the transmission
  Wire.requestFrom(gyroaddress, 1);                                //Request 1 bytes from the gyro
  while(Wire.available() < 1);                                 //Wait until the 1 byte is received
  Serial.print(F("Register 0x1B is set to:"));
  Serial.println(Wire.read(),BIN);            //print the data 

}

//adjusts the gyro offsets based on the 2000 values recorded before
void adjustgyro(){
  Wire.beginTransmission(gyroaddress);  //start communication
  Wire.write(0x43);                 //write to register 0x43
  Wire.endTransmission();           //complete the write up 
  Wire.requestFrom(gyroaddress, 6);     //request 6 bits of data from gyro

  while (Wire.available()<6); //don't continue unless 6 bits of data is received 

  groll=Wire.read()<<8|Wire.read();                        //Read high and low part of the roll data
  if(a == 2000){                  
    groll -= grollcalc;               //adjusts and correct the error in roll measurement
  }
  gpitch=Wire.read()<<8|Wire.read();                       //Read high and low part of the pitch data
  if(a == 2000){
    gpitch -= gpitchcalc;             //adjusts and correct the error in pitch measurement
  }
  gyaw=Wire.read()<<8|Wire.read();                         //Read high and low part of the yaw data
  if(a == 2000){
    gyaw -= gyawcalc;                 //adjusts and correct the error in yaw measurement
  }
}

//This method closely follows joop-brokking's method due to the hardware aspecsts of gyro given on datasheet
//calcualtes the angular motion using the data from the gyro
//the calculations are made in radians
//the movement is just a label to what we want the motion to be
//In this case:
/*Movement 1 = roll
 * Movement 2 = pitch
 * Movement 3 = yaw
 */
void check_gyro_axes(byte movement){
  byte trigger_axis = 0;
  float gyro_angle_roll, gyro_angle_pitch, gyro_angle_yaw;
  //Reset all axes
  gyro_angle_roll = 0;
  gyro_angle_pitch = 0;
  gyro_angle_yaw = 0;
  adjustgyro();
  timer = millis() + 20000;    
  while(timer > millis() && gyro_angle_roll > -30 && gyro_angle_roll < 30 && gyro_angle_pitch > -30 && gyro_angle_pitch < 30 && gyro_angle_yaw > -30 && gyro_angle_yaw < 30){
    adjustgyro();

    //calculate angle from gyro
    gyro_angle_roll += groll * 0.0000611;          // 0.0000611 = 1 / 65.5 (LSB degr/s) / 250(Hz)
    gyro_angle_pitch += gpitch * 0.0000611;
    gyro_angle_yaw += gyaw * 0.0000611;
    
    
    delayMicroseconds(3700); //Loop is running @ 250Hz. +/-300us is used for communication with the gyro
  }
  //Assign the moved axis to the orresponding function (pitch, roll, yaw)
  if((gyro_angle_roll < -30 || gyro_angle_roll > 30) && gyro_angle_pitch > -30 && gyro_angle_pitch < 30 && gyro_angle_yaw > -30 && gyro_angle_yaw < 30){
    gyro_check_byte |= 0b00000001;
    if(gyro_angle_roll < 0)trigger_axis = 0b10000001;
    else trigger_axis = 0b00000001;
  }
  if((gyro_angle_pitch < -30 || gyro_angle_pitch > 30) && gyro_angle_roll > -30 && gyro_angle_roll < 30 && gyro_angle_yaw > -30 && gyro_angle_yaw < 30){
    gyro_check_byte |= 0b00000010;
    if(gyro_angle_pitch < 0)trigger_axis = 0b10000010;
    else trigger_axis = 0b00000010;
  }
  if((gyro_angle_yaw < -30 || gyro_angle_yaw > 30) && gyro_angle_roll > -30 && gyro_angle_roll < 30 && gyro_angle_pitch > -30 && gyro_angle_pitch < 30){
    gyro_check_byte |= 0b00000100;
    if(gyro_angle_yaw < 0)trigger_axis = 0b10000011;
    else trigger_axis = 0b00000011;
  }
  
  if(trigger_axis == 0){
    success=false;
    Serial.println(F("No angular motion is detected in the last 10 seconds!!! (ERROR 4)"));
  }
  else
  if(movement == 1)raxis = trigger_axis;
  if(movement == 2)paxis = trigger_axis;
  if(movement == 3)yaxis = trigger_axis;
}

//interrupt when a signal is received from the transmitter
ISR(PCINT0_vect){
  currenttime = micros();
  //Channel 1=========================================
  //if input 8 is high
  if(PINB & B00000001){                                   
    if(lastch1 == 0){           //Input 8 changed from 0 to 1
      lastch1 = 1;              //Remember current input state
      timer1 = currenttime;      //Set timer1 to currenttime
    }
  }
  else if(lastch1 == 1){      //Input 8 is not high and changed from 1 to 0
    lastch1 = 0;              //Remember current input state
    ch1 = currenttime - timer1;     //Channel 1 is currenttime - timer1
  }
  //Channel 2=========================================
  if(PINB & B00000010 ){   //if digital pin 9 went high
    if(lastch2 == 0){        //Input 9 changed from 0 to 1
      lastch2 = 1;            //Remember current input state
      timer2 = currenttime;    //Set timer2 to currenttime
    }
  }
  else if(lastch2 == 1){         //Input 9 is not high and changed from 1 to 0
    lastch2 = 0;                   //Remember current input state
    ch2 = currenttime - timer2;         //Channel 2 is currenttime - timer2
  }
  //Channel 3=========================================
  if(PINB & B00000100 ){         //if pin 10 went high, it changed state from 0 to 1
    if(lastch3 == 0){                                 
      lastch3 = 1;                                      //Remember current input state
      timer3 = currenttime;                                  //Set timer3 to currenttime
    }
  }
  else if(lastch3 == 1){                                //Input 10 is not high and changed from 1 to 0
    lastch3 = 0;                                        //Remember current input state
    ch3 = currenttime - timer3;         //Channel 3 is currenttime - timer3

  }
  //Channel 4=========================================
  if(PINB & B00001000 ){   //if pin 11 went high, it changed state from 0 to 1
    if(lastch4 == 0){                                   
      lastch4 = 1;                                      //Remember current input state
      timer4 = currenttime;                                  //Set timer4 to currenttime
    }
  }
  else if(lastch4 == 1){                                //Input 11 is not high and changed from 1 to 0
    lastch4 = 0;                                        //Remember current input state
    ch4 = currenttime - timer4;         //Channel 4 is currenttime - timer4
  }
}

/*Variables not declared 
 * 
 * 
 * 
 * 
 */


 /*Varibales not understood
  * 
  * 
  * BIN
  */
