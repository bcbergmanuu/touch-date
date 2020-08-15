#include <MCP7940.h>
#include <arduino-timer.h>
#include <Wire.h>
#include <avr/sleep.h>

#define EEPROM_ADR 0x50
#define page_size 4

const byte interruptPin = 2;
char          inputBuffer[32]; 
volatile int isTouched = 0;
volatile int sleep = 0;

MCP7940_Class MCP7940;                                                        
auto timer = timer_create_default();
Timer <> t_timer;

void setup()
{
  Wire.begin();
  Wire.setClock(400000); // div 8

  Serial.begin(153600); // div 8

  attachInterrupt(digitalPinToInterrupt(interruptPin), touched, CHANGE);

  MCP7940Setup();             
  t_timer.in(20000, goToSleep);

  Serial.println(F("valid commands are:"));  
  Serial.println(F("set yyyy-mm-dd hh:mm:ss")); 
  Serial.println(F("cal yyyy-mm-dd hh:mm:ss"));  
  Serial.println(F("reset"));
  Serial.println(F("recorded"));
  Serial.println(F("current"));  
}

void MCP7940Setup() {
  Serial.print(F(__VERSION__));                                               
  Serial.print(F(__DATE__));                                                  
  Serial.print(F(__TIME__));                                                  
  while (!MCP7940.begin()) {                                                  
    Serial.println(F("Unable to find MCP7940M. Checking again in 3s."));      
    delay(400);                                                              
  }
  Serial.println(F("MCP7940 initialized."));                                  
  while (!MCP7940.deviceStatus()) {                                          
    Serial.println(F("Oscillator is off, turning it on."));                   
    bool deviceStatus = MCP7940.deviceStart();                                
    if (!deviceStatus) {                                                      
      Serial.println(F("Oscillator did not start, trying again."));           
      delay(100);                                                            
    }                                
  }                                         
}

void readCommand() {                                                          
  static uint8_t inputBytes = 0;                                              
  while (Serial.available()) {                                                
    inputBuffer[inputBytes] = Serial.read();                                  
    if (inputBuffer[inputBytes]!='\n' && inputBytes<32)      
      inputBytes++;                                                           
    else {                                                                    
      inputBuffer[inputBytes] =  0;                                           
      for (uint8_t i=0;i<inputBytes;i++)                                      
        inputBuffer[i] = toupper(inputBuffer[i]);                             
      Serial.print(F("\nCommand \""));                                        
      Serial.write(inputBuffer);                                              
      Serial.print(F("\" received.\n"));                                   
                                                  
      char workBuffer[10];                                                    
      sscanf(inputBuffer,"%s %*s",workBuffer);                                
      if      (!strcmp(workBuffer,"SET" )) SetDate(0);                  
      else if (!strcmp(workBuffer,"CAL" ))  SetDate(1); 
      else if (!strcmp(workBuffer,"RESET" )) resetMemory();    
      else if (!strcmp(workBuffer,"RECORDED" )) getDates();             
      else if (!strcmp(workBuffer,"CURRENT" )) getTime();
      else {
         Serial.println(F("Unknown command"));                             
      }                          
      inputBytes = 0;                                     
    }                       
  }                      
} 

void SetDate(int cal) {
  uint16_t tokens,year,month,day,hour,minute,second;  
  tokens = sscanf(inputBuffer,"%*s %hu-%hu-%hu %hu:%hu:%hu;", &year,&month,&day,&hour,&minute,&second);           
  if (tokens!=6)                                                      
    Serial.print(F("Unable to parse date/time\n"));                   
  else {           
    if(cal) {
       int8_t trim = MCP7940.calibrate(DateTime(year,month,day, hour,minute,second));    
        Serial.print(F("Trim value set to "));                            
        Serial.print(trim*2); // Each trim tick is 2 cycles      
        Serial.println(F(" clock cycles every minute"));     
    } else {
        MCP7940.adjust(DateTime(year,month,day,hour,minute,second));      
        Serial.print(F("Date has been set."));                            
    }     
  }    
}

void getTime() {
  uint32_t unixtime = MCP7940.now().unixtime();
  prettyPrint(unixtime);
}

void resetMemory() {
   Serial.print("erasing memory: ");                 
 
   uint8_t empty[page_size] = {0,0,0,0};
   writeEEPROMPage(0, empty);
     
   Serial.println("done");
}

void getDates() {
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
    Serial.print("I went to sleep. Bye.");
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
   uint32_t unixtime = MCP7940.now().unixtime();       
  
   short s_pos = getCurrentPosition();
   if(s_pos > 7990) return; //no dates will be recorded since 256k eeprom is full;
   
   s_pos++;   

   //change position field in eepromm, sector 0
   convertbyte(s_pos, data);
   writeEEPROMPage(0, data);
         
   //write date into eeprom, sector 1..n
   convertbyte(unixtime, data);
   short address = page_size*s_pos;
   writeEEPROMPage(address, data);
}

void touched() {
  noInterrupts();

  isTouched = 1;
  
  interrupts();
}

void writeEEPROMPage(short eeAddress, uint8_t *data)
{    
    for(uint8_t offset = 0; offset < page_size; offset++) {        
      short address = eeAddress + offset;
      Wire.beginTransmission(EEPROM_ADR);
      Wire.write((uint8_t)(address >> 8)); // MSB
      Wire.write((uint8_t)(address & 0xFF)); // LSB
      uint8_t part = data[offset];
      Wire.write(part); //Write the data
      delay(1); // times 8
      Wire.endTransmission(); //Send stop condition
    }    
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
