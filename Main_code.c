/*
 * FroynBast
 * MKT3811 Microprocessor and programming project
 * Temperature and pressure sensor
 * Interfacing PIC16F877A microcontroller with BMP280 temperature and pressure sensor.
 * Temperature and pressure values are displayed on 16x2 LCD.
 */

// define device I2C address: 0xEC or 0xEE (0xEE is library default address)
#define BMP280_I2C_ADDRESS  0xEC

// LCD module connections
#define LCD_RS_PIN      PIN_D0
#define LCD_RW_PIN      PIN_D1
#define LCD_ENABLE_PIN  PIN_D2
#define LCD_DATA4       PIN_D3
#define LCD_DATA5       PIN_D4
#define LCD_DATA6       PIN_D5
#define LCD_DATA7       PIN_D6
// end LCD module connections

#include <16F877.h>
#fuses HS, NOWDT, NOPROTECT, NOLVP                       
#use delay(clock = 8MHz)
#use I2C(MASTER, I2C1, FAST = 400000, STREAM = BMP280_STREAM)
#include <BMP280_Lib.c>  // include BMP280 sensor driver source file
#include <lcd.c>         // include LCD driver source file

signed int32 temperature;
unsigned int32 pressure;

void main()
{
  delay_ms(1000);  // wait 1 second

  lcd_init();       // initialize LCD module
  lcd_putc('\f');   // clear the LCD

  // initialize the BMP280 sensor
  if(BMP280_begin(MODE_NORMAL) == 0)
  {  // connection error or device address wrong!
    lcd_gotoxy(1, 1);    // go to column 1 row 1
    lcd_putc("Connection");
    lcd_gotoxy(1, 2);    // go to column 1 row 2
    lcd_putc("error!");
    while(TRUE);  // stay here
  }

  lcd_gotoxy(1, 1);    // go to column 1 row 1
  lcd_putc("Temp:");
  lcd_gotoxy(1, 2);    // go to column 1 row 2
  lcd_putc("Pres:");
  
  while(TRUE)
  {
    // Read temperature (in hundredths C) and pressure (in Pa)
    // values from BMP280 sensor
    BMP280_readTemperature(&temperature);  // read temperature
    BMP280_readPressure(&pressure);        // read pressure

    // print data on the LCD screen
    // 1: print temperature
    lcd_gotoxy(6, 1);    // go to column 6 row 1
    if(temperature < 0)
    {
      lcd_putc('-');
      temperature = abs(temperature);
    }
    else
      lcd_putc(' ');

    printf(lcd_putc, "%02Lu.%02Lu%cC", temperature / 100, temperature % 100, 223);

    // 2: print pressure
    lcd_gotoxy(7, 2);    // go to column 7 row 1
    printf(lcd_putc, "%04Lu.%02LuhPa", pressure/100, pressure % 100);

    delay_ms(2000);  // wait 2 seconds

  }

}
// end of code.
