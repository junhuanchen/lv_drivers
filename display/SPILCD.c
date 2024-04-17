
#include "SPILCD.h"
#include "DEV_Config.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include LV_DRV_DISP_INCLUDE
#include LV_DRV_DELAY_INCLUDE

/*********************
 *      DEFINES
 *********************/
#define SPILCD_BAUD      		2000000    /*< 2,5 MHz (400 ns)*/

#define SPILCD_CMD_MODE     0
#define SPILCD_DATA_MODE    1

/* SPILCD Commands that we know of.  Limited documentation */
#define SPILCD_INVOFF	0x20
#define SPILCD_INVON	0x21
#define SPILCD_DISPON	0x29
#define SPILCD_CASET	0x2A
#define SPILCD_RASET	0x2B
#define SPILCD_RAMWR	0x2C
#define SPILCD_COLMOD	0x3A
#define SPILCD_MADCTL	0x36
#define SPILCD_MADCTL_MY  0x80
#define SPILCD_MADCTL_MX  0x40
#define SPILCD_MADCTL_MV  0x20
#define SPILCD_MADCTL_RGB 0x00
#define SPILCD_DISFNCTRL	0xB6

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

static void LCD_2IN4_Reset(void)
{
	puts("LCD_2IN4_Reset");
	DEV_Digital_Write(LCD_CS, 1);
	DEV_Delay_ms(100);
	DEV_Digital_Write(LCD_RST, 0);
	DEV_Delay_ms(200);
	DEV_Digital_Write(LCD_RST, 1);
	DEV_Delay_ms(200);
	DEV_Digital_Write(LCD_RST, 0);
	DEV_Delay_ms(200);
	DEV_Digital_Write(LCD_RST, 1);
	DEV_Delay_ms(200);
}
static void LCD_2IN4_Write_Command(UBYTE data)	 
{	
	DEV_Digital_Write(LCD_CS, 0);
	DEV_Digital_Write(LCD_DC, 0);
	DEV_SPI_WriteByte(data);
}

static void LCD_2IN4_WriteData_Byte(UBYTE data) 
{	
	DEV_Digital_Write(LCD_CS, 0);
	DEV_Digital_Write(LCD_DC, 1);
	DEV_SPI_WriteByte(data);  
	DEV_Digital_Write(LCD_CS,1);
}  

static void SPILCD_set_addr_win(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  uint16_t x_start = x0 + 0, x_end = x1 + 0;
  uint16_t y_start = y0 + 0, y_end = y1 + 0;

  LCD_2IN4_Write_Command(SPILCD_CASET); // Column addr set
  LCD_2IN4_WriteData_Byte(x_start >> 8);
  LCD_2IN4_WriteData_Byte(x_start & 0xFF);     // XSTART 
  LCD_2IN4_WriteData_Byte(x_end >> 8);
  LCD_2IN4_WriteData_Byte(x_end & 0xFF);     // XEND

  LCD_2IN4_Write_Command(SPILCD_RASET); // Row addr set
  LCD_2IN4_WriteData_Byte(y_start >> 8);
  LCD_2IN4_WriteData_Byte(y_start & 0xFF);     // YSTART
  LCD_2IN4_WriteData_Byte(y_end >> 8);
  LCD_2IN4_WriteData_Byte(y_end & 0xFF);     // YEND

  LCD_2IN4_Write_Command(SPILCD_RAMWR);
}

#include <signal.h>
void  Handler_2IN4_LCD(int signo)
{
    //System Exit
    printf("\r\nHandler:Program stop\r\n");     
    DEV_ModuleExit();
    exit(0);
}

void SPILCD_setRotation(uint8_t m) {

  LCD_2IN4_Write_Command(SPILCD_MADCTL);
  m %= 4; // can't be higher than 3
  switch (m) {
   case 0:
     LCD_2IN4_WriteData_Byte(SPILCD_MADCTL_MX | SPILCD_MADCTL_MY | SPILCD_MADCTL_RGB);

    //  _xstart = _colstart;
    //  _ystart = _rowstart;
     break;
   case 1:
     LCD_2IN4_WriteData_Byte(SPILCD_MADCTL_MY | SPILCD_MADCTL_MV | SPILCD_MADCTL_RGB);

    //  _ystart = _colstart;
    //  _xstart = _rowstart;
     break;
  case 2:
     LCD_2IN4_WriteData_Byte(SPILCD_MADCTL_RGB);
 
    //  _xstart = _colstart;
    //  _ystart = _rowstart;
     break;

   case 3:
     LCD_2IN4_WriteData_Byte(SPILCD_MADCTL_MX | SPILCD_MADCTL_MV | SPILCD_MADCTL_RGB);

    //  _ystart = _colstart;
    //  _xstart = _rowstart;
     break;
  }
}

void SPILCD_invertDisplay(bool i)
{
  LCD_2IN4_Write_Command(i ? SPILCD_INVON : SPILCD_INVOFF);
}

void SPILCD_init()
{
	signal(SIGINT, Handler_2IN4_LCD);
	signal(SIGTERM, Handler_2IN4_LCD);
	signal(SIGKILL, Handler_2IN4_LCD);
	signal(SIGQUIT, Handler_2IN4_LCD);
	
	DEV_ModuleInit();
	
	LCD_2IN4_Reset();

	LCD_2IN4_Write_Command(0x11); //Sleep out
	
	LCD_2IN4_Write_Command(0xCF);
	LCD_2IN4_WriteData_Byte(0x00);
	LCD_2IN4_WriteData_Byte(0xC1);
	LCD_2IN4_WriteData_Byte(0X30);
	LCD_2IN4_Write_Command(0xED);
	LCD_2IN4_WriteData_Byte(0x64);
	LCD_2IN4_WriteData_Byte(0x03);
	LCD_2IN4_WriteData_Byte(0X12);
	LCD_2IN4_WriteData_Byte(0X81);
	LCD_2IN4_Write_Command(0xE8);
	LCD_2IN4_WriteData_Byte(0x85);
	LCD_2IN4_WriteData_Byte(0x00);
	LCD_2IN4_WriteData_Byte(0x79);
	LCD_2IN4_Write_Command(0xCB);
	LCD_2IN4_WriteData_Byte(0x39);
	LCD_2IN4_WriteData_Byte(0x2C);
	LCD_2IN4_WriteData_Byte(0x00);
	LCD_2IN4_WriteData_Byte(0x34);
	LCD_2IN4_WriteData_Byte(0x02);
	LCD_2IN4_Write_Command(0xF7);
	LCD_2IN4_WriteData_Byte(0x20);
	LCD_2IN4_Write_Command(0xEA);
	LCD_2IN4_WriteData_Byte(0x00);
	LCD_2IN4_WriteData_Byte(0x00);
	LCD_2IN4_Write_Command(0xC0); //Power control
	LCD_2IN4_WriteData_Byte(0x1D); //VRH[5:0]
	LCD_2IN4_Write_Command(0xC1); //Power control
	LCD_2IN4_WriteData_Byte(0x12); //SAP[2:0];BT[3:0]
	LCD_2IN4_Write_Command(0xC5); //VCM control
	LCD_2IN4_WriteData_Byte(0x33);
	LCD_2IN4_WriteData_Byte(0x3F);
	LCD_2IN4_Write_Command(0xC7); //VCM control
	LCD_2IN4_WriteData_Byte(0x92);
	LCD_2IN4_Write_Command(0x3A); // Memory Access Control
	LCD_2IN4_WriteData_Byte(0x55);
	LCD_2IN4_Write_Command(0x36); // Memory Access Control
	LCD_2IN4_WriteData_Byte(0b1011000); // 0x58
	// SPILCD_setRotation(0);
	// SPILCD_invertDisplay(0);
	LCD_2IN4_Write_Command(0xB1);
	LCD_2IN4_WriteData_Byte(0x00);
	LCD_2IN4_WriteData_Byte(0x12);
	LCD_2IN4_Write_Command(0xB6); // Display Function Control
	LCD_2IN4_WriteData_Byte(0x0A);
	LCD_2IN4_WriteData_Byte(0xA2);

	LCD_2IN4_Write_Command(0x44);
	LCD_2IN4_WriteData_Byte(0x02);

	LCD_2IN4_Write_Command(0xF2); // 3Gamma Function Disable
	LCD_2IN4_WriteData_Byte(0x00);
	LCD_2IN4_Write_Command(0x26); //Gamma curve selected
	LCD_2IN4_WriteData_Byte(0x01);
	LCD_2IN4_Write_Command(0xE0); //Set Gamma
	LCD_2IN4_WriteData_Byte(0x0F);
	LCD_2IN4_WriteData_Byte(0x22);
	LCD_2IN4_WriteData_Byte(0x1C);
	LCD_2IN4_WriteData_Byte(0x1B);
	LCD_2IN4_WriteData_Byte(0x08);
	LCD_2IN4_WriteData_Byte(0x0F);
	LCD_2IN4_WriteData_Byte(0x48);
	LCD_2IN4_WriteData_Byte(0xB8);
	LCD_2IN4_WriteData_Byte(0x34);
	LCD_2IN4_WriteData_Byte(0x05);
	LCD_2IN4_WriteData_Byte(0x0C);
	LCD_2IN4_WriteData_Byte(0x09);
	LCD_2IN4_WriteData_Byte(0x0F);
	LCD_2IN4_WriteData_Byte(0x07);
	LCD_2IN4_WriteData_Byte(0x00);
	LCD_2IN4_Write_Command(0XE1); //Set Gamma
	LCD_2IN4_WriteData_Byte(0x00);
	LCD_2IN4_WriteData_Byte(0x23);
	LCD_2IN4_WriteData_Byte(0x24);
	LCD_2IN4_WriteData_Byte(0x07);
	LCD_2IN4_WriteData_Byte(0x10);
	LCD_2IN4_WriteData_Byte(0x07);
	LCD_2IN4_WriteData_Byte(0x38);
	LCD_2IN4_WriteData_Byte(0x47);
	LCD_2IN4_WriteData_Byte(0x4B);
	LCD_2IN4_WriteData_Byte(0x0A);
	LCD_2IN4_WriteData_Byte(0x13);
	LCD_2IN4_WriteData_Byte(0x06);
	LCD_2IN4_WriteData_Byte(0x30);
	LCD_2IN4_WriteData_Byte(0x38);
	LCD_2IN4_WriteData_Byte(0x0F);
	LCD_2IN4_Write_Command(0x29); //Display on

}

void LCD_2IN4_SetWindow(UWORD Xstart, UWORD Ystart, UWORD Xend, UWORD  Yend)
{ 
	LCD_2IN4_Write_Command(0x2a);
	LCD_2IN4_WriteData_Byte(Xstart >>8);
	LCD_2IN4_WriteData_Byte(Xstart & 0xff);
	LCD_2IN4_WriteData_Byte((Xend - 1) >> 8);
	LCD_2IN4_WriteData_Byte((Xend - 1) & 0xff);

	LCD_2IN4_Write_Command(0x2b);
	LCD_2IN4_WriteData_Byte(Ystart >>8);
	LCD_2IN4_WriteData_Byte(Ystart & 0xff);
	LCD_2IN4_WriteData_Byte((Yend - 1) >> 8);
	LCD_2IN4_WriteData_Byte((Yend - 1) & 0xff);

	LCD_2IN4_Write_Command(0x2C);
}

void LCD_2IN4_Clear(UWORD Color)
{
	UWORD i;
	UWORD image[SPILCD_HOR_RES];
	for(i=0;i<SPILCD_HOR_RES;i++){
		image[i] = Color>>8 | (Color&0xff)<<8;
	}
	UBYTE *p = (UBYTE *)(image);
	LCD_2IN4_SetWindow(0, 0, SPILCD_HOR_RES, SPILCD_VER_RES);
	DEV_Digital_Write(LCD_DC, 1);
	for(i = 0; i < SPILCD_VER_RES; i++){
		DEV_SPI_Write_nByte(p,SPILCD_HOR_RES*2);
	}
}

void SPILCD_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_p)
{
  SPILCD_set_addr_win(area->x1, area->y1, area->x2, area->y2);
  int32_t size = (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1) * 2;

	// printf("_lvgl_ui__flush area %d\r\n", lv_area_get_size(area));
	
	// size = (uint32_t)(area_p->x2 - area_p->x1 + 1) * (area_p->y2 - area_p->y1 + 1);
	// printf("area x1:%d,y1:%d,x2:%d,y2:%d size:%d\r\n", area->x1, area->y1, area->x2, area->y2, size);
	
	DEV_Digital_Write(LCD_CS, 0);
	DEV_Digital_Write(LCD_DC, 1);
	#define perdata 1024
	int pos = 0, sum = size / perdata;
	uint8_t *buf = (uint8_t*)color_p;
	while (sum--) {
		DEV_SPI_Write_nByte(buf + pos, perdata);
		pos += perdata;
	}
	DEV_SPI_Write_nByte(buf + pos, size % perdata);
	// DEV_SPI_Write_nByte((uint8_t*)color_p, len);
	DEV_Digital_Write(LCD_CS, 1);

  // LV_DRV_DISP_SPI_CS(1);
  lv_disp_flush_ready(drv);         /* Indicate you are ready with the flushing*/
}
