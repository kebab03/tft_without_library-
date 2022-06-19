#define write8inline(d) {Serial.println(F("******SONO IN write8inline pin_magic.h   LINE--6  FOR Breakout board **** #define write8  *********"));\                                                                      
   uint32_t pota=(((d)&(1<<0))<<10)|(((d)&(1<<5))<<3)|(((d)&(1<<6))<<3);\
   uint32_t potb=(((d)&(1<<1))<<2)|(((d)&(1<<2))<<3)|(((d)&(1<<3))<<1)|(((d)&(1<<4))<<6);\ 
   uint32_t potc=(((d)&(1<<7))<<0);\
   Serial.println(F("pota:::"));\
   Serial.println(pota,BIN);\
   Serial.println(F("potb=="));\
   Serial.println( potb,BIN);\
   Serial.println(F("potc---"));\
   Serial.println(potc,BIN);\
   GPIOA->BSRR |=(((d)&(1<<0))<<10)|(((d)&(1<<5))<<3)|(((d)&(1<<6))<<3);\ 
   GPIOB->BSRR |=(((d)&(1<<1))<<2)|(((d)&(1<<2))<<3)|(((d)&(1<<3))<<1)|(((d)&(1<<4))<<6);\             
   GPIOC->BSRR |=(((d)&(1<<7))<<0);\          
    }

#define setWriteDirInline(){GPIOA->MODER |= 0b00000000000101010000000000000000;\
                            GPIOB->MODER |= 0b00000000000100000000010101000000;\
                            GPIOC->MODER |= 0b00000000000000000100000000000000;\                                                        
  }
    //GPIOA ->MODER |=  1<<16;//  16 =PA8 =16 =2*8 $$$1<<10;//0x1400;
// basta porre 01  ai 2 bt del  pin , 1 è nel bit di dx del pin 
// 10 =2*pin_number
#define setReadDirInline(){GPIOA->MODER |=  0x00000000;  \
      GPIOB->MODER |=0x00000000;\
      GPIOC->MODER &=~(7<<24);\
  }
#define WRITE8BIT(d){static uint32_t PA_Set=((((d)&(1<<0))<<9)|(((d)&(1<<2))<<8)|(((d)&(1<<7))<<1));\
static uint32_t Mask_PA=((~PA_Set)<<16)|(PA_Set);\
 \
 static uint32_t PB_Set=(((d) &(1<<3))\
               |(((d) &(1<<4))<<1)\
               |(((d) &(1<<5))>>1)\
               |(((d) &(1<<6))<<4));\
 \              
 static uint32_t Mask_PB=((~PB_Set)<<16)|(PB_Set);\
\ 
 static uint32_t PC_Set=(((d)&(1<<1))<<6);\
\ 
 static uint32_t Mask_PC=((~PC_Set)<<16)|(PC_Set);\
 GPIOA->ODR|=Mask_PA;\
 GPIOB->ODR|=Mask_PB;\
 GPIOC->ODR|=Mask_PC;\
  }
