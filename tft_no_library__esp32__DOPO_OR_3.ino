
#include "pins.h"
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define RED2    0x4000
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define GREEN2  0x2FA4
#define CYAN2   0x07FF
//https://electronicseternit.wixsite.com/electronicsforlife/post/using-2-4-tft-lcd-display-without-library
#define LCD_RD   A0 //Serves as read signal/MCU read data at the rising edge. Pg11 Datasheet
#define LCD_WR   A1 //Serves as write signal/command at the rising edge
#define LCD_RS   A2 //D/CX (0=Command/1=Data)       
#define LCD_CS   A3 //Chip Select Pin : Active Low
#define LCD_RST  A4 //Shield Reset

void LCD_write(uint8_t d){
  // Serves as write signal/command at the rising edge
  digitalWrite(LCD_WR,LOW); // WR 0
//  Serial.print(F("d=DATA/COMMAND IN 8 BIT :LCD_WR,LOW:=="));
//  Serial.println(d,BIN);
//    Serial.print(F("0<<3))=="));
//  Serial.println((0<<3),BIN);
//    Serial.print(F("(d&(1<<0))=="));
//  Serial.println((d&(1<<0)),BIN);
  //static uint16_t PA_Set=((((d)&(1<<0))<<9)|(((d)&(1<<2))<<8)|(((d)&(1<<7))<<1));
  uint16_t PA_Set1=((d&(1<<0))<<9);
//  Serial.print(F("PA_Set1=="));
//  Serial.println(PA_Set1,BIN);
  uint16_t PA_Set2=((d &(1<<2))<<8);
//  Serial.print(F("PA_Set2=="));
//  Serial.println(PA_Set2,BIN);
  uint16_t PA_Set3=((d &(1<<7))<<1);
//  Serial.print(F("PA_Set3=="));
//  Serial.println(PA_Set3,BIN);
  uint16_t PA_Set=PA_Set1|PA_Set2|PA_Set3;

   uint16_t PB_Set=(((d)&(1<<3))|(((d)&(1<<4))<<1)|(((d)&(1<<5))>>1)|(((d)&(1<<6))<<4));
   uint16_t PC_Set=(((d)&(1<<1))<<6);
   uint32_t GPIOA_AD=GPIOA->ODR;
   uint32_t GPIOA_MSK=GPIOA_AD|PA_Set;
   uint32_t GPIOB_AD=GPIOB->ODR;
   uint32_t GPIOC_AD=GPIOC->ODR;
//  Serial.print(F("GPIOC_AD::=="));
//  Serial.println(GPIOC_AD,BIN);
//  Serial.print(F("PC_Set=="));
//  Serial.println(PC_Set,BIN);
//  Serial.print(F("GPIOA_MSK in 8 BIT :IN BIN FORMAT:=="));
//  Serial.println(GPIOA_MSK,BIN); 
//  Serial.print(F("GPIOA_AD CIOè QUELLO CHE HO IN QUESTI REGIS::=="));
//  Serial.println(GPIOA_AD,BIN);
//  Serial.print(F("PA_Set=="));
//  Serial.println(PA_Set,BIN);
//  Serial.print(F("GPIOB_AD::=="));
//  Serial.println(GPIOB_AD,BIN);
//  Serial.print(F("PB_Set=="));
//  Serial.println(PB_Set,BIN);
//  Serial.print(F("GPIOC_AD::=="));
//  Serial.println(GPIOC_AD,HEX);
//  Serial.print(F("PC_Set=="));
//  Serial.println(PC_Set,BIN);
   uint16_t PA_Mask=~((1<<10)|(1<<9)|(1<<8));
   uint16_t PB_Mask=~((1<<3)|(1<<4)|(1<<5)|(1<<10));
   uint16_t PC_Mask=~(1<<7);
  uint16_t TEM_REG;
 TEM_REG =GPIOA->ODR ;
 //**Serial.print(F("TEM_REG =GPIOA->ODR:=="));
  //**Serial.println(TEM_REG,BIN);
     //**Serial.print(F("TEM_REG:PRIMA DI TEM_REG &=PA_Mask; :=="));
  //**Serial.println(TEM_REG,BIN);
   TEM_REG &=PA_Mask;
   GPIOA->ODR =TEM_REG;
  GPIOA->ODR |=PA_Set; //((((d)&(1<<0))<<9)|(((d)&(1<<2))<<8)|(((d)&(1<<7))<<1));
  //**Serial.print(F("TEM_REG:A:=="));
  //**Serial.println(TEM_REG,BIN);
  TEM_REG =GPIOB->ODR;
  TEM_REG &=PB_Mask;
  GPIOB->ODR =TEM_REG;
  GPIOB->ODR  |=(((d)&(1<<3))|(((d)&(1<<4))<<1)|(((d)&(1<<5))>>1)|(((d)&(1<<6))<<4));
  //**Serial.print(F("TEM_REG:B:=="));
  //**Serial.println(TEM_REG,BIN);
  TEM_REG =GPIOC->ODR ;
  TEM_REG &=PC_Mask;
  GPIOC->ODR =TEM_REG;
  GPIOC->ODR  |=PC_Set;
  //**Serial.print(F("TEM_REG:C:=="));
  //**Serial.println(TEM_REG,BIN);
digitalWrite(LCD_WR,HIGH); // WR 1

//   uint32_t GPIOA_ADR=GPIOA->ODR;
//   uint32_t GPIOB_ADR=GPIOB->ODR;
//   uint32_t GPIOC_ADR=GPIOC->ODR;
//**Serial.println(F("******DOPO RESETING I BIT DEI PIN *"));
 //** Serial.print(F("GPIOA_ADR::=="));
  //**Serial.println(GPIOA_ADR,BIN);
  
//**Serial.print(F("GPIOB_ADR::=="));
 //** Serial.println(GPIOB_ADR,BIN);

  
  //**Serial.print(F("GPIOC_ADR::=="));
  //**Serial.println(GPIOC_ADR,BIN);
//**Serial.println(F("******LCD_write line 44*"));
}


void LCD_command_write(uint8_t command) {
  // LCD_RS = 0, A2=0, D/CX (0=Command/1=Data) | DataSheet Page 11
  digitalWrite(LCD_RS, LOW);
  
   //**Serial.println(F("******LCD_command_write line 50*"));
  //** Serial.print(F("******LCD_command======::=="));
   //**Serial.println(command,BIN);
   LCD_write(command);  
}

void LCD_data_write(uint8_t data) {
  // LCD_RS = 1, A2=1, D/CX (0=Command/1=Data) | DataSheet Page 11
  digitalWrite(LCD_RS, HIGH);
  //**Serial.println(F("******LCD_data_write line 55*"));
  LCD_write(data);
}

void Lcd_Init(void) {
  //void does not return any value
  //void only execute instruction within it
  //similar to void setup and loop
  //This function will have LCD initialization measures
  //Only the necessary Commands are covered
  //Eventho there are so many more in DataSheet

  //Reset Signal is Active LOW
  digitalWrite(LCD_RST, HIGH);
  delay(5);
  digitalWrite(LCD_RST, LOW); //Actual Reset Done Here
  delay(15);
  digitalWrite(LCD_RST, HIGH);
  delay(15);

  //The below is just preparation for Write Cycle Seq
  digitalWrite(LCD_CS, HIGH); //Chip-Select Active Low Signal
  digitalWrite(LCD_WR, HIGH);
  digitalWrite(LCD_RD, HIGH);
  digitalWrite(LCD_CS, LOW);  //Chip-Select Active Low Signal
//**Serial.println(F(" Innitiate  LINE--79 FOR Breakout board"));
  LCD_command_write(0xC5);    //Test this Out | VCOM Control 1 : Colour Contrast Maybe
  LCD_data_write(0x54);       //VCOM H 1111111 0x7F
  LCD_data_write(0x00);       //VCOM L 0000000
  //LCD_data_write(B1010011);
  
  LCD_command_write(0x36);    //Memory Access Control | DataSheet Page 127
  ///LCD_data_write(0x48);       //Adjust this value to get right color and starting point of x and y
  LCD_data_write(B0000100);     //Example

  LCD_command_write(0x3A);    //COLMOD: Pixel Format Set | DataSheet Page 134
  LCD_data_write(0x55);       //16 Bit RGB and MCU

  LCD_command_write(0x11);    //Sleep Out | DataSheet Page 245
  delay(6);                  //Necessary to wait 5msec before sending next command

  LCD_command_write(0x29);    //Display on.

  LCD_command_write(0x2c);    //Memory Write | DataSheet Page 245
}

void Address_set(int16_t y1, int16_t y2, int16_t x1, int16_t x2) {//**Serial.println(F(" Address_set  LINE--100 FOR Breakout board"));
  LCD_command_write(0x2a);  //Column Address Set | DataSheet Page 110
  LCD_data_write(y1 >> 8);  //8 Bit Shift Right of y1
  LCD_data_write(y1);       //Value of y1
  LCD_data_write(y2 >> 8);  //8 Bit Shift Right of y2
  LCD_data_write(y2);       //Value of y2

  LCD_command_write(0x2b);  //Page Address Set | DataSheet Page 110
  LCD_data_write(x1 >> 8);  //8 Bit Shift Right of x1
  LCD_data_write(x1);       //Value of x1
  LCD_data_write(x2 >> 8);  //8 Bit Shift Right of x2
  LCD_data_write(x2);       //Value of x2

  LCD_command_write(0x2c); // REG 2Ch = Memory Write
}

void drawPixel(int16_t x, int16_t y, uint16_t color) { //**Serial.println(F("******DRAW PIXEL  LINE--120 FOR Breakout board **** #define write8  *********"));
  digitalWrite(LCD_CS, LOW);// Chip Select active
  Address_set(y, y + 1, x, x + 1);
  //
  LCD_command_write(0x2C);
  LCD_data_write(color >>8);
  LCD_data_write(color);
}

void setup() { Serial.begin(9600);
  // Setting Pin 2-7 as Output, DDR is PinMode Command, Pin0,1 Untouched
//  DDRD = DDRD | B11111100;
//  // Setting Pin 8-9 as Output
//  DDRB = DDRB | B00000011;
 
pinMode(PA8,OUTPUT);
pinMode(PA9,OUTPUT);
pinMode(PA10,OUTPUT);
pinMode(PB3,OUTPUT);
pinMode(PB4,OUTPUT);
pinMode(PB5,OUTPUT);
pinMode(PB10,OUTPUT);
pinMode(PC7,OUTPUT);
//  //Setting Analog Pins A4-A0 as Output
pinMode(LCD_RD,OUTPUT);
pinMode(LCD_WR,OUTPUT);
pinMode(LCD_RS,OUTPUT);
pinMode(LCD_CS,OUTPUT);
pinMode(LCD_RST,OUTPUT);
  //Setting Analog Pins A4-A0 as HIGH
  //PORTC = PORTC | B00011111; setWriteDirInline();
 
//
Lcd_Init();
//       for (int i = 50; i < 300; i++) {
//        drawPixel(i, 60, RED);
//        drawPixel(i, 70, RED);
//        drawPixel(i, 80, GREEN);
//        drawPixel(i, 90, GREEN);
//      }
//Serial.println("0x2B,BIN=======");
//Serial.println("\n");
//Serial.println(0x2B,BIN);
//LCD_command_write(0b00101011);
Serial.print("0b11000110,BIN=======***************************:");
Serial.println("\n");
//Serial.println(0b11000110,BIN);
//LCD_command_write(0b11000110);
//Serial.println(0b11111000,BIN);
//LCD_command_write(0b11111000);
}

void loop() {//LCD_command_write(0b00101011);
//0x2b);
 for (int i = 50; i < 300; i++) {
//        drawPixel(i,60,RED);
//        drawPixel(i,70,RED);
//        drawPixel(i,80,BLACK);
        drawPixel(i,90,BLUE);
         drawPixel(i,90,MAGENTA);
          drawPixel(i,90,GREEN);}

}
