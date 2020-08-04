#include <MCP7940.h>
#include <arduino-timer.h>
#include <Wire.h>
#include <avr/sleep.h>

#define EEPROM_ADR 0x50
#define page_size 4

const byte interruptPin = 2;
const byte ledpin = 13;

const uint8_t  SPRINTF_BUFFER_SIZE =     32;
char          inputBuffer[SPRINTF_BUFFER_SIZE]; 
volatile int isTouched = 0;
volatile int sleep = 0;

MCP7940_Class MCP7940;                                                        
auto timer = timer_create_default();
Timer <> t_timer;

void setup()
{
  Wire.begin();
  Wire.setClock(400000);

  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(interruptPin), touched, CHANGE);

  MCP7940Setup();             
  
  pinMode(ledpin, OUTPUT);
  digitalWrite(ledpin, HIGH);
  t_timer.in(10000, goToSleep);
}

void MCP7940Setup() {
  Serial.print(F(__VERSION__));                                               
  Serial.print(F(__DATE__));                                                  
  Serial.print(F(__TIME__));                                                  
  while (!MCP7940.begin()) {                                                  
    Serial.println(F("Unable to find MCP7940M. Checking again in 3s."));      
    delay(3000);                                                              
  }
  Serial.println(F("MCP7940 initialized."));                                  
  while (!MCP7940.deviceStatus()) {                                           // Turn oscillator on if necessary  //
    Serial.println(F("Oscillator is off, turning it on."));                   //                                  //
    bool deviceStatus = MCP7940.deviceStart();                                // Start oscillator and return state//
    if (!deviceStatus) {                                                      // If it didn't start               //
      Serial.println(F("Oscillator did not start, trying again."));           // Show error and                   //
      delay(1000);                                                            // wait for a second                //
    } // of if-then oscillator didn't start                                   //                                  //
  } // of while the oscillator is off                                         //                                  //
}

void readCommand() {                                                          //                                  //
  static uint8_t inputBytes = 0;                                              // Variable for buffer position     //
  while (Serial.available()) {                                                // Loop while incoming serial data  //
    inputBuffer[inputBytes] = Serial.read();                                  // Get the next byte of data        //
    if (inputBuffer[inputBytes]!='\n' && inputBytes<SPRINTF_BUFFER_SIZE)      // keep on reading until a newline  //
      inputBytes++;                                                           // shows up or the buffer is full   //
    else {                                                                    //                                  //
      inputBuffer[inputBytes] =  0;                                            // Add the termination character    //
      for (uint8_t i=0;i<inputBytes;i++)                                      // Convert the whole input buffer   //
        inputBuffer[i] = toupper(inputBuffer[i]);                             // to uppercase characters          //
      Serial.print(F("\nCommand \""));                                        //                                  //
      Serial.write(inputBuffer);                                              //                                  //
      Serial.print(F("\" received.\n"));                                      //                                  //
      /*************************************************************************************************************
      ** Parse the single-line command and perform the appropriate action. The current list of commands           **
      ** understood are as follows:                                                                               **
      **                                                                                                          **
      ** SETDATE      - Set the device time                                                                       **
      ** CALDATE      - Calibrate device time                                                                     **
      **                                                                                                          **
      *************************************************************************************************************/
      enum commands { SetDate, CalDate, reset, GetDates, Unknown_Command };                    // of commands enumerated type      //
      commands command;                                                       // declare enumerated type          //
      char workBuffer[10];                                                    // Buffer to hold string compare    //
      sscanf(inputBuffer,"%s %*s",workBuffer);                                // Parse the string for first word  //
      if      (!strcmp(workBuffer,"SETDATE" )) command = SetDate;             // Set command number when found    //
      else if (!strcmp(workBuffer,"CALDATE" )) command = CalDate;             // Set command number when found    //
      else if (!strcmp(workBuffer,"RESET" )) command = reset;             //clear eeprom
      else if (!strcmp(workBuffer,"GETDATES" )) command = GetDates;             //clear eeprom
      else command = Unknown_Command;                                         // Otherwise set to not found       //
      uint16_t tokens,year,month,day,hour,minute,second;                      // Variables to hold parsed dt/tm   //
      switch (command) {                                                      // Action depending upon command    //
        /***********************************************************************************************************
        ** Set the device time and date                                                                           **
        ***********************************************************************************************************/
        case SetDate:                                                         // Set the RTC date/time            //
          tokens = sscanf(inputBuffer,"%*s %hu-%hu-%hu %hu:%hu:%hu;",         // Use sscanf() to parse the date/  //
                          &year,&month,&day,&hour,&minute,&second);           // time into variables              //
          if (tokens!=6)                                                      // Check to see if it was parsed    //
            Serial.print(F("Unable to parse date/time\n"));                   //                                  //
          else {                                                              //                                  //
            MCP7940.adjust(DateTime(year,month,day,hour,minute,second));      // Adjust the RTC date/time         //
            Serial.print(F("Date has been set."));                            //                                  //
          } // of if-then-else the date could be parsed                       //                                  //
          break;                                                              //                                  //
        /***********************************************************************************************************
        ** Calibrate the RTC and reset the time                                                                   **
        ***********************************************************************************************************/
        case CalDate:                                                         // Calibrate the RTC                //
          tokens = sscanf(inputBuffer,"%*s %hu-%hu-%hu %hu:%hu:%hu;",         // Use sscanf() to parse the date/  //
                          &year,&month,&day,&hour,&minute,&second);           // time into variables              //
          if (tokens!=6)                                                      // Check to see if it was parsed    //
            Serial.print(F("Unable to parse date/time\n"));                   //                                  //
          else {                                                              //                                  //
            int8_t trim = MCP7940.calibrate(DateTime(year,month,day,          // Calibrate the crystal and return //
                                                     hour,minute,second));    // the new trim offset value        //
            Serial.print(F("Trim value set to "));                            //                                  //
            Serial.print(trim*2);                                             // Each trim tick is 2 cycles       //
            Serial.println(F(" clock cycles every minute"));                  //                                  //
          } // of if-then-else the date could be parsed                       //                                  //
          break;                                                              //                                  //
        /***********************************************************************************************************
        ** Unknown command                                                                                        **
        ***********************************************************************************************************/
        case reset:
          resetMemory();          
              
         
          break;
        case GetDates:
          readDates();
          break;
        case Unknown_Command:                                                 // Show options on bad command      //
        default:                                                              //                                  //
          Serial.println(F("Unknown command. Valid commands are:"));          //                                  //
          Serial.println(F("SETDATE yyyy-mm-dd hh:mm:ss"));                   //                                  //
          Serial.println(F("CALDATE yyyy-mm-dd hh:mm:ss"));                   //                                  //
      } // of switch statement to execute commands                            //                                  //
      inputBytes = 0; // reset the counter                                    //                                  //
    } // of if-then-else we've received full command                          //                                  //
  } // of if-then there is something in our input buffer                      //                                  //
} // of method readCommand  

void resetMemory() {
     Serial.print("erasing memory: ");                 
     short pos_l = getCurrentPosition();
       for(short m =0; m<pos_l+1; m++) {
           uint8_t empty[page_size] = {0,0,0,0};
           writeEEPROMPage(m*page_size, empty);
           if(m%16==0)
              Serial.print('.');
       }           
     Serial.println("done");
}

void readDates() {
    short s_pos = getCurrentPosition();
    Serial.print(s_pos);
    Serial.println(" dates recorded:");
    for(short p = 0; p < s_pos; p++) {
      uint8_t time_t[4];
      readEEPROM((p+1)*page_size, time_t);
      
      uint32_t buff[4] = {time_t[0], time_t[1], time_t[2], time_t[3]};
      uint32_t datetime = (buff[0]) | (buff[1] << 8) | (buff[2] << 16) | (buff[3] << 24);
      Serial.print(p + 1);
      Serial.print(' ');
   
      prettyPrint(datetime);
      Serial.println();
    }
}

bool goToSleep(void *) {
  sleep = 1;
  return true;
}

void loop()
{       
  t_timer.tick();
  if(isTouched == 1) {
    isTouched = 0;
    writeCurrentTime();
  }
  readCommand();
  
  if(sleep == 1) {
    digitalWrite(ledpin, LOW);
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);  
    sleep_enable();   
    sleep_mode();    
    sleep_disable();  
  }
}

 //convert to 4x 8 bit
void convertbyte(uint32_t input, uint8_t * output) {
   output[3] = (input & 0xff000000UL) >> 24;
   output[2] = (input & 0x00ff0000UL) >> 16;
   output[1] = (input & 0x0000ff00UL) >>  8;
   output[0] = (input & 0x000000ffUL)      ;   
}

//last stored date on eeprom
short getCurrentPosition() {
   uint8_t pos[4];   
   
   readEEPROM(0, pos);  
   short s_pos = pos[0] | pos[1] << 8;
   
   return s_pos;
}

void prettyPrint(uint32_t unixtime) {
   DateTime time_t = DateTime(unixtime);

   Serial.print(time_t.day());
   Serial.print("-");
   Serial.print(time_t.month());
   Serial.print("-");
   Serial.print(time_t.year());  
   Serial.print(".");
   Serial.print(time_t.hour());
   Serial.print(":");
   Serial.print(time_t.minute());
   Serial.print(":");
   Serial.print(time_t.second());
   
}

void writeCurrentTime() {
   uint8_t data[4];
   uint8_t unixtime = MCP7940.now().unixtime();       
  
   short s_pos = getCurrentPosition();
   s_pos++;   

   //change position field in eepromm, sector 0
   convertbyte(s_pos, data);
   writeEEPROMPage(0, data);
         
   //write date into eeprom, sector 1..n
   convertbyte(unixtime, data);
   short address = page_size*(s_pos+1);
   writeEEPROMPage(address, data);
}

void touched() {
  noInterrupts();

  isTouched = 1;
  
  interrupts();
}

void writeEEPROMPage(short eeAddress, uint8_t *data)
{
    Wire.beginTransmission(EEPROM_ADR);
  
    Wire.write((uint8_t)(eeAddress >> 8)); // MSB
    Wire.write((uint8_t)(eeAddress & 0xFF)); // LSB
  
    Wire.write(data, page_size); //Write the data
    delay(5);
  
    Wire.endTransmission(); //Send stop condition
}

void readEEPROM(short eeAddress, uint8_t *data)
{   
    Wire.beginTransmission(EEPROM_ADR);
  
    Wire.write((uint8_t)(eeAddress >> 8)); // MSB
    Wire.write((uint8_t)(eeAddress & 0xFF)); // LSB
    Wire.endTransmission();
  
    Wire.requestFrom(EEPROM_ADR, page_size);
    for(int x =0; x< page_size; x++) {
      data[x] = Wire.read();
    }    
}
