/* 
 * This is a port of the RA8875 TFT LCD Controller library
 * by Adafruit (see below) for use with MPLAB X IDE / XC8
 * compiler and the PIC18F (tested using PIC18F46J11)
 * 
 * File:   Stubb_RA8875.h
 * Author: Kyle Stubbins (kyle@kylestubbins.com)
 *
 * Created on January 30, 2016, 12:31 PM
 */

/**************************************************************************/
/*!
    @file     Adafruit_RA8875.cpp
    @author   Limor Friend/Ladyada, K.Townsend/KTOWN for Adafruit Industries
    @license  BSD license, all text above and below must be included in
              any redistribution

 This is the library for the Adafruit RA8875 Driver board for TFT displays
 ---------------> http://www.adafruit.com/products/1590
 The RA8875 is a TFT driver for up to 800x480 dotclock'd displays
 It is tested to work with displays in the Adafruit shop. Other displays
 may need timing adjustments and are not guanteed to work.
 
 Adafruit invests time and resources providing this open
 source code, please support Adafruit and open-source hardware
 by purchasing products from Adafruit!
 
 Written by Limor Fried/Ladyada for Adafruit Industries.
 BSD license, check license.txt for more information.
 All text above must be included in any redistribution.

    @section  HISTORY
    
    v1.0 - First release
*/

#ifndef STUBB_RA8875_H
#define	STUBB_RA8875_H

/** I N C L U D E S **********************************************************/

#include "stubb_globals.h"
#include <plib/spi.h>
#include "stubb_spi.h"

/* E N U M E R A T I O N S ***************************************************/



/* T Y P E D E F S ***********************************************************/

// Touch screen calibration structs
typedef struct Point 
{
    int32_t x;
    int32_t y;
} tsPoint_t;

typedef struct //Matrix
{
    int32_t An,
            Bn,
            Cn,
            Dn,
            En,
            Fn,
            Divider ;
} tsMatrix_t;


/* D E F I N E S *************************************************************/

/* uncomment / comment the following settings as required */

// uncomment to use with Adafruit modules
#define RA8875_ADAFRUIT

/* uncomment to use the hardware reset pin (Adafruit) 
 * (this is optional as the RA8875_ADAFRUIT define above
 *  will also set this)
 * leave commented to use with EastRising modules 
 */
//#define RA8875_USE_RESET_PIN

/* uncomment to use the hardware wait pin, instead of polling */
//#define RA8875_USE_WAIT_PIN

/* uncomment to use the hardware interrupt pin */
//#define RA8875_USE_INTERRUPT_PIN

// uncomment the proper size for the display
//#define RA8875_P480x272     // 4.3" displays
#define RA8875_P800x480         // 5" & 7" displays          

/* uncomment to use the integrated touch screen */
#define RA8875_USE_TOUCH

/* uncomment to use layer functionality */
//#define RA8875_USE_LAYERS

/* define macros for the hardware CS pin */
#define RA8875_CS_ENABLE()              RA8875_CS = 0;
#define RA8875_CS_DISABLE()             RA8875_CS = 1;    

// Colors (RGB565)
#define	RA8875_BLACK                    0x0000
#define	RA8875_BLUE                     0x001F
#define	RA8875_RED                      0xF800
#define	RA8875_GREEN                    0x07E0
#define RA8875_CYAN                     0x07FF
#define RA8875_MAGENTA                  0xF81F
#define RA8875_YELLOW                   0xFFE0  
#define RA8875_WHITE                    0xFFFF
#define RA8875_GREY                     0x8410
#define RA8875_LTGREY                   0xBDF7
#define RA8875_LTBLUE                   0x021F
#define RA8875_LTRED                    0xF965

// Command/Data pins for SPI
#define RA8875_DATAWRITE                0x00
#define RA8875_DATAREAD                 0x40
#define RA8875_CMDWRITE                 0x80
#define RA8875_CMDREAD                  0xC0

// Registers & bits
#define RA8875_PWRR                     0x01
#define RA8875_PWRR_DISPON              0x80
#define RA8875_PWRR_DISPOFF             0x00
#define RA8875_PWRR_SLEEP               0x02
#define RA8875_PWRR_NORMAL              0x00
#define RA8875_PWRR_SOFTRESET           0x01

#define RA8875_MRWC                     0x02

#define RA8875_GPIOX                    0xC7

#define RA8875_PLLC1                    0x88
#define RA8875_PLLC1_PLLDIV2            0x80
#define RA8875_PLLC1_PLLDIV1            0x00

#define RA8875_PLLC2                    0x89
#define RA8875_PLLC2_DIV1               0x00
#define RA8875_PLLC2_DIV2               0x01
#define RA8875_PLLC2_DIV4               0x02
#define RA8875_PLLC2_DIV8               0x03
#define RA8875_PLLC2_DIV16              0x04
#define RA8875_PLLC2_DIV32              0x05
#define RA8875_PLLC2_DIV64              0x06
#define RA8875_PLLC2_DIV128             0x07

#define RA8875_SYSR                     0x10
#define RA8875_SYSR_8BPP                0x00
#define RA8875_SYSR_16BPP               0x0C
#define RA8875_SYSR_MCU8                0x00
#define RA8875_SYSR_MCU16               0x03

#define RA8875_PCSR                     0x04
#define RA8875_PCSR_PDATR               0x00
#define RA8875_PCSR_PDATL               0x80
#define RA8875_PCSR_CLK                 0x00
#define RA8875_PCSR_2CLK                0x01
#define RA8875_PCSR_4CLK                0x02
#define RA8875_PCSR_8CLK                0x03
    
#define RA8875_HDWR                     0x14

#define RA8875_HNDFTR                   0x15
#define RA8875_HNDFTR_DE_HIGH           0x00
#define RA8875_HNDFTR_DE_LOW            0x80

#define RA8875_HNDR                     0x16
#define RA8875_HSTR                     0x17
#define RA8875_HPWR                     0x18
#define RA8875_HPWR_LOW                 0x00
#define RA8875_HPWR_HIGH                0x80

#define RA8875_VDHR0                    0x19
#define RA8875_VDHR1                    0x1A
#define RA8875_VNDR0                    0x1B
#define RA8875_VNDR1                    0x1C
#define RA8875_VSTR0                    0x1D
#define RA8875_VSTR1                    0x1E
#define RA8875_VPWR                     0x1F
#define RA8875_VPWR_LOW                 0x00
#define RA8875_VPWR_HIGH                0x80

#define RA8875_HSAW0                    0x30
#define RA8875_HSAW1                    0x31
#define RA8875_VSAW0                    0x32
#define RA8875_VSAW1                    0x33

#define RA8875_HEAW0                    0x34
#define RA8875_HEAW1                    0x35
#define RA8875_VEAW0                    0x36
#define RA8875_VEAW1                    0x37

#define RA8875_MCLR                     0x8E
#define RA8875_MCLR_START               0x80
#define RA8875_MCLR_STOP                0x00
#define RA8875_MCLR_READSTATUS          0x80
#define RA8875_MCLR_FULL                0x00
#define RA8875_MCLR_ACTIVE              0x40

#define RA8875_DCR                      0x90
#define RA8875_DCR_LINESQUTRI_START     0x80
#define RA8875_DCR_LINESQUTRI_STOP      0x00
#define RA8875_DCR_LINESQUTRI_STATUS    0x80
#define RA8875_DCR_CIRCLE_START         0x40
#define RA8875_DCR_CIRCLE_STATUS        0x40
#define RA8875_DCR_CIRCLE_STOP          0x00
#define RA8875_DCR_FILL                 0x20
#define RA8875_DCR_NOFILL               0x00
#define RA8875_DCR_DRAWLINE             0x00
#define RA8875_DCR_DRAWTRIANGLE         0x01
#define RA8875_DCR_DRAWSQUARE           0x10

#define RA8875_CIRCLESQUARE             0xA0
#define RA8875_CIRCLESQUARE_STATUS      0x80

#define RA8875_ELLIPSE                  0xA0
#define RA8875_ELLIPSE_STATUS           0x80

#define RA8875_MWCR0                    0x40
#define RA8875_MWCR0_GFXMODE            0x00
#define RA8875_MWCR0_TXTMODE            0x80

#define RA8875_CURH0                    0x46
#define RA8875_CURH1                    0x47
#define RA8875_CURV0                    0x48
#define RA8875_CURV1                    0x49

#define RA8875_P1CR                     0x8A
#define RA8875_P1CR_ENABLE              0x80
#define RA8875_P1CR_DISABLE             0x00
#define RA8875_P1CR_CLKOUT              0x10
#define RA8875_P1CR_PWMOUT              0x00

#define RA8875_P1DCR                    0x8B

#define RA8875_P2CR                     0x8C
#define RA8875_P2CR_ENABLE              0x80
#define RA8875_P2CR_DISABLE             0x00
#define RA8875_P2CR_CLKOUT              0x10
#define RA8875_P2CR_PWMOUT              0x00

#define RA8875_P2DCR                    0x8D

#define RA8875_PWM_CLK_DIV1             0x00
#define RA8875_PWM_CLK_DIV2             0x01
#define RA8875_PWM_CLK_DIV4             0x02
#define RA8875_PWM_CLK_DIV8             0x03
#define RA8875_PWM_CLK_DIV16            0x04
#define RA8875_PWM_CLK_DIV32            0x05
#define RA8875_PWM_CLK_DIV64            0x06
#define RA8875_PWM_CLK_DIV128           0x07
#define RA8875_PWM_CLK_DIV256           0x08
#define RA8875_PWM_CLK_DIV512           0x09
#define RA8875_PWM_CLK_DIV1024          0x0A
#define RA8875_PWM_CLK_DIV2048          0x0B
#define RA8875_PWM_CLK_DIV4096          0x0C
#define RA8875_PWM_CLK_DIV8192          0x0D
#define RA8875_PWM_CLK_DIV16384         0x0E
#define RA8875_PWM_CLK_DIV32768         0x0F

#define RA8875_TPCR0                    0x70
#define RA8875_TPCR0_ENABLE             0x80
#define RA8875_TPCR0_DISABLE            0x00
#define RA8875_TPCR0_WAIT_512CLK        0x00
#define RA8875_TPCR0_WAIT_1024CLK       0x10
#define RA8875_TPCR0_WAIT_2048CLK       0x20
#define RA8875_TPCR0_WAIT_4096CLK       0x30
#define RA8875_TPCR0_WAIT_8192CLK       0x40
#define RA8875_TPCR0_WAIT_16384CLK      0x50
#define RA8875_TPCR0_WAIT_32768CLK      0x60
#define RA8875_TPCR0_WAIT_65536CLK      0x70
#define RA8875_TPCR0_WAKEENABLE         0x08
#define RA8875_TPCR0_WAKEDISABLE        0x00
#define RA8875_TPCR0_ADCCLK_DIV1        0x00
#define RA8875_TPCR0_ADCCLK_DIV2        0x01
#define RA8875_TPCR0_ADCCLK_DIV4        0x02
#define RA8875_TPCR0_ADCCLK_DIV8        0x03
#define RA8875_TPCR0_ADCCLK_DIV16       0x04
#define RA8875_TPCR0_ADCCLK_DIV32       0x05
#define RA8875_TPCR0_ADCCLK_DIV64       0x06
#define RA8875_TPCR0_ADCCLK_DIV128      0x07

#define RA8875_TPCR1                    0x71
#define RA8875_TPCR1_AUTO               0x00
#define RA8875_TPCR1_MANUAL             0x40
#define RA8875_TPCR1_VREFINT            0x00
#define RA8875_TPCR1_VREFEXT            0x20
#define RA8875_TPCR1_DEBOUNCE           0x04
#define RA8875_TPCR1_NODEBOUNCE         0x00
#define RA8875_TPCR1_IDLE               0x00
#define RA8875_TPCR1_WAIT               0x01
#define RA8875_TPCR1_LATCHX             0x02
#define RA8875_TPCR1_LATCHY             0x03

#define RA8875_TPXH                     0x72
#define RA8875_TPYH                     0x73
#define RA8875_TPXYL                    0x74

#define RA8875_INTC1                    0xF0
#define RA8875_INTC1_KEY                0x10
#define RA8875_INTC1_DMA                0x08
#define RA8875_INTC1_TP                 0x04
#define RA8875_INTC1_BTE                0x02

#define RA8875_INTC2                    0xF1
#define RA8875_INTC2_KEY                0x10
#define RA8875_INTC2_DMA                0x08
#define RA8875_INTC2_TP                 0x04
#define RA8875_INTC2_BTE                0x02

/* Stubb Mods */
#define RA8875_TEXT_SMALL               0x00
#define RA8875_TEXT_MEDIUM              0x01
#define RA8875_TEXT_LARGE               0x02
#define RA8875_TEXT_XLARGE              0x03

/* Scroll Window Registers */
#define RA8875_HSSW0                    0x38
#define RA8875_HSSW1                    0x39
#define RA8875_VSSW0                    0x3A
#define RA8875_VSSW1                    0x3B
#define RA8875_HESW0                    0x3C
#define RA8875_HESW1                    0x3D
#define RA8875_VESW0                    0x3E
#define RA8875_VESW1                    0x3F

/* Scroll Window Offset Regsiters */
#define RA8875_HOFS0                    0x24
#define RA8875_HOFS1                    0x25
#define RA8875_VOFS0                    0x26
#define RA8875_VOFS1                    0x27

/* Layers Transparency Mode */
#define RA8875_LTPR0                    0x52
#define RA8875_LTPR0_LAYER1             0x00
#define RA8875_LTPR0_LAYER2             0x01
#define RA8875_LTPR0_LIGHTEN            0x02
#define RA8875_LTPR0_TRANSPARENT        0x03
#define RA8875_LTPR0_OR                 0x04
#define RA8875_LTPR0_AND                0x05
#define RA8875_LTPR0_FLOAT              0x06
#define RA8875_LTPR0_FLOAT_TRANS        0x20
/* Layer Scroll */
#define RA8875_BOTH                     0x00
#define RA8875_LAYER1                   0x01
#define RA8875_LAYER2                   0x02
#define RA8875_BUFFER                   0x03
/* Layers Transparency Setting */
#define RA8875_LTPR1                    0x53
/* Layer Transparency - Layer 1 */
#define RA8875_LTPR1_1_TOTAL            0x00
#define RA8875_LTPR1_1_78               0x01
#define RA8875_LTPR1_1_34               0x02
#define RA8875_LTPR1_1_58               0x03
#define RA8875_LTPR1_1_12               0x04
#define RA8875_LTPR1_1_38               0x05
#define RA8875_LTPR1_1_14               0x06
#define RA8875_LTPR1_1_18               0x07
#define RA8875_LTPR1_1_DISABLE          0x08
/* Layer Transparency - Layer 2 */
#define RA8875_LTPR1_2_TOTAL            0x00
#define RA8875_LTPR1_2_78               0x10
#define RA8875_LTPR1_2_34               0x20
#define RA8875_LTPR1_2_58               0x30
#define RA8875_LTPR1_2_12               0x40
#define RA8875_LTPR1_2_38               0x50
#define RA8875_LTPR1_2_14               0x60
#define RA8875_LTPR1_2_18               0x70
#define RA8875_LTPR1_2_DISABLE          0x80
/* Background Color for Transparent */
#define RA8875_BGTR0                    0x67
#define RA8875_BGTR1                    0x68
#define RA8875_BGTR2                    0x69

/* Display Configuration Register */
#define RA8875_DPCR                     0x20
#define RA8875_DPCR_ONE_LAYER           0x00
#define RA8875_DPCR_TWO_LAYERS          0x80

/* Font Control Register 0 */
#define RA8875_FNCR0                    0x21

/* Font Control Register 1 */
#define RA8875_FNCR1                    0x22


/* Memory Write Control Register 1 */
#define RA8875_MWCR1                    0x41
#define RA8875_MWCR1_LAYER1             0x00
#define RA8875_MWCR1_LAYER2             0x01

#define RA8875_MWCR1_DEST_LAYERS        0x00
#define RA8875_MWCR1_DEST_CGRAM         0x01
#define RA8875_MWCR1_DEST_GCURSOR       0x10
#define RA8875_MWCR1_DEST_PATTERN       0x11


/* Orientation - Angles */
#define RA8875_ORIENTATION_UP           0
#define RA8875_ORIENTATION_RIGHT        90
#define RA8875_ORIENTATION_DOWN         180
#define RA8875_ORIENTATION_LEFT         270



/* P R O T O T Y P E S *******************************************************/

uint8_t RA8875_begin(void);
uint16_t RA8875_width(void);
uint16_t RA8875_height(void);
void RA8875_writeReg(uint8_t reg, uint8_t val);
uint8_t RA8875_readReg(uint8_t reg);
void RA8875_displayOn(uint8_t on);
void RA8875_sleep(uint8_t sleep) ;
void RA8875_setActiveWindow(uint16_t XL, uint16_t XR, uint16_t YT, uint16_t YB);
void RA8875_clearActiveWindow(uint8_t full);
void RA8875_setWriteDestination(uint8_t dest);
void RA8875_setActiveLayer(uint8_t layer);
uint8_t RA8875_getActiveLayer(void);
void RA8875_setScrollLayer(uint8_t mode);
void RA8875_setScrollWindow(uint16_t XL, uint16_t XR, uint16_t YT, uint16_t YB);
void RA8875_scrollWindow(int16_t offsetX, int16_t offsetY);
void RA8875_setDisplayConfig(uint8_t config);
void RA8875_transparencyColor(uint16_t color);
void RA8875_transparencyMode(uint8_t mode, uint8_t layer1, uint8_t layer2);
void RA8875_setOrientation(int16_t angle);
uint8_t RA8875_isPortraitMode(void);

/* Backlight */
void RA8875_gpiox (uint8_t on);
void RA8875_PWM1out(uint8_t p);
void RA8875_PWM2out(uint8_t p);
void RA8875_PWM1config(uint8_t on, uint8_t clock);
void RA8875_PWM2config(uint8_t on, uint8_t clock);

/* Touch screen */
void RA8875_touchEnable(uint8_t on);
uint8_t RA8875_touched(void);
uint8_t RA8875_touchRead(uint16_t *x, uint16_t *y);

/* Text */
void RA8875_textMode(void);
void RA8875_textSetCursor(uint16_t x, uint16_t y) ;
uint16_t RA8875_textGetCursor_X(void);
uint16_t RA8875_textGetCursor_Y(void);
void RA8875_textColor(uint16_t foreColor, uint16_t bgColor);
void RA8875_textTransparent(uint16_t foreColor);
void RA8875_textEnlarge(uint8_t scale);
void RA8875_textWrite(const char *buffer, uint16_t len);

/* Graphics */
void RA8875_graphicsMode(void);
uint8_t RA8875_waitPoll(uint8_t regname, uint8_t waitflag) ;
void RA8875_setXY(uint16_t x, uint16_t y);
void RA8875_drawPixel(int16_t x, int16_t y, uint16_t color);
void RA8875_pushPixels(uint32_t num, uint16_t p);
void RA8875_drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void RA8875_drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
void RA8875_drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
void RA8875_fillScreen(uint16_t color);
void RA8875_drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
void RA8875_fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
void RA8875_drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void RA8875_fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void RA8875_drawRoundedRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r1, int16_t r2, uint16_t color);
void RA8875_fillRoundedRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r1, int16_t r2, uint16_t color);
void RA8875_drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void RA8875_fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void RA8875_drawEllipse(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint16_t color);
void RA8875_fillEllipse(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint16_t color);
void RA8875_drawCurve(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint8_t curvePart, uint16_t color);
void RA8875_fillCurve(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint8_t curvePart, uint16_t color);

/* GFX Helper Functions */
void RA8875_circleHelper(int16_t x0, int16_t y0, int16_t r, uint16_t color, uint8_t filled);
void RA8875_rectHelper  (int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color, uint8_t filled);
void RA8875_roundedRectHelper(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r1, int16_t r2, uint16_t color, uint8_t filled);
void RA8875_triangleHelper(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color, uint8_t filled);
void RA8875_ellipseHelper(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint16_t color, uint8_t filled);
void RA8875_curveHelper(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint8_t curvePart, uint16_t color, uint8_t filled);


/* Private */
void RA8875_PLLInit(void);
void RA8875_initialize(uint8_t color, uint8_t mcu);
void RA8875_hardReset(void);
void RA8875_softReset(void);

void RA8875_writeData(uint8_t data);
uint8_t RA8875_readData(void);
void RA8875_writeCommand(uint8_t cmd);
uint8_t RA8875_readStatus(void);

#endif	/* STUBB_RA8875_H */

