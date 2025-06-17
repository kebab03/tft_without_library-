#include <SPI.h>

// Definizioni dei pin (adatta ai tuoi collegamenti)
#define TFT_MOSI 23
#define TFT_MISO 19
#define TFT_SCLK 18
#define TFT_CS   5
#define TFT_DC   2  // Corrisponde a LCD_RS nel codice originale
#define TFT_RST  4  // Corrisponde a LCD_RST nel codice originale
#define TFT_WR   -1 // Non usato in SPI
#define TFT_RD   -1 // Non usato in SPI

// Definizioni dei colori (come nel tuo codice originale)
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

// Comandi ILI9341
#define ILI9341_SWRESET   0x01
#define ILI9341_SLPOUT    0x11
#define ILI9341_DISPON    0x29
#define ILI9341_CASET     0x2A
#define ILI9341_PASET     0x2B
#define ILI9341_RAMWR     0x2C
#define ILI9341_MADCTL    0x36
#define ILI9341_COLMOD    0x3A
#define ILI9341_VCOMCTL1  0xC5

SPIClass hspi(HSPI);

void sendCommand(uint8_t cmd) {
  digitalWrite(TFT_DC, LOW);  // Modalità comando
  digitalWrite(TFT_CS, LOW);
  hspi.transfer(cmd);
  digitalWrite(TFT_CS, HIGH);
}

void sendData(uint8_t data) {
  digitalWrite(TFT_DC, HIGH); // Modalità dati
  digitalWrite(TFT_CS, LOW);
  hspi.transfer(data);
  digitalWrite(TFT_CS, HIGH);
}

void sendData16(uint16_t data) {
  digitalWrite(TFT_DC, HIGH); // Modalità dati
  digitalWrite(TFT_CS, LOW);
  hspi.transfer(data >> 8);    // Byte alto
  hspi.transfer(data & 0xFF);  // Byte basso
  digitalWrite(TFT_CS, HIGH);
}

void resetDisplay() {
  digitalWrite(TFT_RST, LOW);
  delay(100);
  digitalWrite(TFT_RST, HIGH);
  delay(150);
}

void Lcd_Init(void) {
  // Inizializza i pin
  pinMode(TFT_CS, OUTPUT);
  pinMode(TFT_DC, OUTPUT);
  pinMode(TFT_RST, OUTPUT);
  
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TFT_DC, HIGH);
  
  // Reset del display
  resetDisplay();

  // Inizializza SPI
  hspi.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, TFT_CS);
  hspi.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0)); // 40 MHz

  // Sequenza di inizializzazione (come nel tuo codice originale)
  sendCommand(ILI9341_VCOMCTL1);    // VCOM Control 1
  sendData(0x54);                   // VCOM H
  sendData(0x00);                   // VCOM L

  sendCommand(ILI9341_MADCTL);      // Memory Access Control
  sendData(0x04);                   // Orientamento del display

  sendCommand(ILI9341_COLMOD);      // Pixel Format Set
  sendData(0x55);                   // 16 bit per pixel

  sendCommand(ILI9341_SLPOUT);      // Sleep Out
  delay(150);
  
  sendCommand(ILI9341_DISPON);      // Display On
  delay(150);
   sendCommand(0x2c);    //Memory Write | DataSheet Page 245
}

void Address_set(int16_t y1, int16_t y2, int16_t x1, int16_t x2) {
  sendCommand(ILI9341_CASET);      //Column Address Set | DataSheet Page 110
  sendData(y1 >> 8);//8 Bit Shift Right of y1
  sendData(y1 & 0xFF);//Value of y1
  sendData(y2 >> 8);
  sendData(y2 & 0xFF);

  sendCommand(ILI9341_PASET);       //Page Address Set | DataSheet Page 110
  sendData(x1 >> 8);//8 Bit Shift Right of x1
  sendData(x1 & 0xFF);   //Value of x1
  sendData(x2 >> 8); //8 Bit Shift Right of x2
  sendData(x2 & 0xFF); //Value of x2

  sendCommand(ILI9341_RAMWR);       // Memory Write
}

void drawPixel(int16_t x, int16_t y, uint16_t color) {
  Address_set(y, y + 1, x, x + 1);
  sendData16(color);
}
void fillScreen(uint16_t color) {
  // Imposta l'area di disegno su tutto lo schermo
  sendCommand(ILI9341_CASET); // Column address set
  sendData(0x00); sendData(0);     // X START
  sendData(0x00); sendData(239);   // X END (240-1)

  sendCommand(ILI9341_PASET); // Row address set
  sendData(0x00); sendData(0);     // Y START
  sendData(0x01); sendData(0x3F);  // Y END (320-1)

  sendCommand(ILI9341_RAMWR); // Memory write

  digitalWrite(TFT_DC, HIGH);
  digitalWrite(TFT_CS, LOW);

  uint8_t hi = color >> 8; // i 8 bit MSB/ left side  per quest high bit
  uint8_t lo = color & 0xFF;

  // Invia i dati per tutti i pixel in una volta sola
  for(uint32_t i = 0; i < 240UL * 320UL; i++) {
    hspi.transfer(hi);
    hspi.transfer(lo);
  }

  digitalWrite(TFT_CS, HIGH);
}
void setup() {
  Serial.begin(115200);
  Lcd_Init();
  
  // Esempio di disegno (come nel tuo loop originale)
  for (int i = 50; i < 100; i++) {
    drawPixel(i, 60, RED);
    drawPixel(i, 70, RED);
    drawPixel(i, 80, GREEN);
    drawPixel(i, 90, GREEN);
  }
  Serial.println("Inizializzazione completata");
delay(2000);
   // Esempi di utilizzo:
  fillScreen(RED);     // Riempie lo schermo di rosso
  delay(1000);
  fillScreen(GREEN);   // Riempie lo schermo di verde
  delay(1000);
  fillScreen(BLUE);    // Riempie lo schermo di blu

}

void loop() {
  // Esempio di animazione (come nel tuo loop originale)
  for (int i = 100; i < 200; i++) {
    drawPixel(i, 90, BLUE);
    drawPixel(i, 100, MAGENTA);
    drawPixel(i, 120, GREEN);
    delay(10);
  }
}