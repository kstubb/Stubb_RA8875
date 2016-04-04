/* 
 * This is a port of the RA8875 TFT LCD Controller library
 * by Adafruit (see below) for use with MPLAB X IDE / XC8
 * compiler and the PIC18F (tested using PIC18F46J11)
 * 
 * File:   Stubb_RA8875.c
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

/* I N C L U D E S ********************************************************/

#include "Stubb_RA8875.h"

/* V A R I A B L E S ******************************************************/

/* Private */
uint16_t _width = 0;
uint16_t _height = 0;
uint8_t _textScale = 0;
uint8_t _portraitMode = FALSE;


/* F U N C T I O N S ******************************************************/

/**************************************************************************/
/*!
    Main entry point to setup the module
*/
/**************************************************************************/
uint8_t RA8875_begin(void)
{
    uint8_t temp = 0;
    
#if defined (RA8875_P480x272)
    _width = 480;
    _height = 272;
    
#elif defined (RA8875_P800x480)
    _width = 800;
    _height = 480;
    
#else 
#error RA8875 display size not defined!
#endif
    
    RA8875_CS_TRIS = 0;
    RA8875_CS = 1;

#if defined (RA8875_USE_RESET_PIN)  || defined (RA8875_ADAFRUIT)    
    RA8875_RST_TRIS = 0;
    RA8875_RST = 0;
#endif
    
#if defined (RA8875_USE_WAIT_PIN)
    RA8875_WAIT_TRIS = 1;   // set the WAIT pin as input
    RA8875_WAIT = 1;        // set the input high
#endif 
    
#if defined (RA8875_USE_INTERRUPT_PIN)
    RA8875_INTERRUPT_TRIS = 1;
    RA8875_INTERRUPT = 1;
#endif
    
    delay(100);
    
#if defined (RA8875_USE_RESET_PIN) || defined (RA8875_ADAFRUIT)
    // perform a hardware reset of the LCD
    RA8875_hardReset();
#else
    // perform a software reset of the LCD
    RA8875_softReset();
#endif
    
    if (RA8875_readReg(0) != 0x75) {
        return FALSE;
    }
    
    // initialize the LCD
    RA8875_initialize(RA8875_SYSR_16BPP, RA8875_SYSR_MCU8);
    
    return TRUE;
}


/**************************************************************************/
/*!
      Returns the display width in pixels
      
      @returns  The 1-based display width in pixels
*/
/**************************************************************************/
uint16_t RA8875_width(void) 
{ 
    return _width; 
}


/**************************************************************************/
/*!
      Returns the display height in pixels
      @returns  The 1-based display height in pixels
*/
/**************************************************************************/
uint16_t RA8875_height(void) 
{ 
    return _height; 
}


/**************************************************************************/
/*!
    Writes a data value to a specified register 
 
    @args reg       The register to write to
    @args val       The value to write to the register
*/
/**************************************************************************/
void RA8875_writeReg(uint8_t reg, uint8_t val) 
{
    RA8875_writeCommand(reg);
    RA8875_writeData(val);
}


/**************************************************************************/
/*!
 *  Reads a data value from a specified register 
 *  
 *  @args reg       The register to read from
 *  @returns        data byte read
*/
/**************************************************************************/
uint8_t RA8875_readReg(uint8_t reg) 
{
    RA8875_writeCommand(reg);
    return RA8875_readData();
}


/**************************************************************************/
/*! 
    Turns the display on / off
*/
/**************************************************************************/
void RA8875_displayOn(uint8_t on) 
{
    if (on) 
        RA8875_writeReg(RA8875_PWRR, RA8875_PWRR_NORMAL | RA8875_PWRR_DISPON);
    else
        RA8875_writeReg(RA8875_PWRR, RA8875_PWRR_NORMAL | RA8875_PWRR_DISPOFF);
    return;
}


/**************************************************************************/
/*!
    Puts the display into sleep mode, or disables sleep mode if enabled 
*/
/**************************************************************************/
void RA8875_sleep(uint8_t sleep) 
{
    if (sleep) 
        RA8875_writeReg(RA8875_PWRR, RA8875_PWRR_DISPOFF | RA8875_PWRR_SLEEP);
    else
        RA8875_writeReg(RA8875_PWRR, RA8875_PWRR_DISPOFF);
    return;
}


/**************************************************************************/
/*!
    Sets the size of the active window 
 
    @args XL       X direction, Left side
    @args XR       X direction, Right side
    @args YT       Y direction, Top
    @args YB       Y direction, Bottom
*/
/**************************************************************************/
void RA8875_setActiveWindow(uint16_t XL, uint16_t XR, uint16_t YT, uint16_t YB)
{
    /* Set active window X */
    RA8875_writeReg(RA8875_HSAW0, (uint8_t)(XL & 0xFF));        // horizontal start point
    RA8875_writeReg(RA8875_HSAW1, (uint8_t)(XL >> 8));
    RA8875_writeReg(RA8875_HEAW0, (uint8_t)(XR & 0xFF));        // horizontal end point
    RA8875_writeReg(RA8875_HEAW1, (uint8_t)(XR >> 8));
  
    /* Set active window Y */
    RA8875_writeReg(RA8875_VSAW0, (uint8_t)(YT & 0xFF));        // vertical start point
    RA8875_writeReg(RA8875_VSAW1, (uint8_t)(YT >> 8));  
    RA8875_writeReg(RA8875_VEAW0, (uint8_t)(YB & 0xFF));        // horizontal end point
    RA8875_writeReg(RA8875_VEAW1, (uint8_t)(YB >> 8));
}


void RA8875_clearActiveWindow(uint8_t full)
{
    uint8_t temp = RA8875_readReg(RA8875_MCLR);
    full == true ? temp &= RA8875_MCLR_FULL : temp |= RA8875_MCLR_ACTIVE;
    RA8875_writeData(temp | RA8875_MCLR_START);
}


#if defined (RA8875_USE_LAYERS)
/**************************************************************************/
/*!
    Sets the destination for read / write operations
    00 = Layer 1 ~ 2
    01 = CGRAM
    10 = Graphic Cursor
    11 = Pattern
  
    @args dest       The target destination
*/
/**************************************************************************/
void RA8875_setWriteDestination(uint8_t dest)
{
    /* Read the current value of the MWCR1 register */
    uint8_t current = RA8875_readReg(RA8875_MWCR1);
    /* Set the active layer */
    RA8875_writeReg(RA8875_MWCR1, (dest & 0x0C) | (current & 0xF3));
}


/**************************************************************************/
/*!
    Sets the specified layer as active for read / write operations
 
    @args layer       The layer to set active for read / write
*/
/**************************************************************************/
void RA8875_setActiveLayer(uint8_t layer)
{
    /* Read the current value of the MWCR1 register */
    uint8_t current = RA8875_readReg(RA8875_MWCR1);
    /* Set the active layer */
    RA8875_writeReg(RA8875_MWCR1, (layer & 0x01) | (current & 0xFE));
}


/**************************************************************************/
/*!
    Returns the active drawing layer
*/
/**************************************************************************/
uint8_t RA8875_getActiveLayer(void) 
{
    return (RA8875_readReg(RA8875_MWCR1) & 0x01);
}


/**************************************************************************/
/*!
 *  Sets the background color for use with layer transparency
 * 
 *  @args color[in] The RGB565 color to use
*/
/**************************************************************************/
void RA8875_transparencyColor(uint16_t color)
{
    RA8875_writeReg(RA8875_BGTR0, (color & 0xf800) >> 11);
    RA8875_writeReg(RA8875_BGTR1, (color & 0x07e0) >> 5);
    RA8875_writeReg(RA8875_BGTR2, color & 0x001f);
}


/**************************************************************************/
/*!
 *  Sets the transparency of each layer for mixed display
 * 
 *  @args mode[in]      The mode for layer visibility
 *  @args layer1[in]    The transparency setting for layer 1
 *  @args layer2[in]    The transparency setting for layer 2
*/
/**************************************************************************/
void RA8875_transparencyMode(uint8_t mode, uint8_t layer1, uint8_t layer2)
{
    /* read the current values of the LTPR0 register */
    uint8_t current = RA8875_readReg(RA8875_LTPR0);
    
    /* Set the Transparency Mode */
    RA8875_writeReg(RA8875_LTPR0, (mode & 0x03) | (current & 0xF8));
    
    /* Set the Transparency Settings */
    RA8875_writeReg(RA8875_LTPR1, (layer2 << 4) | (layer1 & 0x07));

}
#endif 


/**************************************************************************/
/*!
    Sets the size of the scroll window 
 
    @args XL       X direction, Left side
    @args XR       X direction, Right side
    @args YT       Y direction, Top
    @args YB       Y direction, Bottom
*/
/**************************************************************************/
void RA8875_setScrollWindow(uint16_t XL, uint16_t XR, uint16_t YT, uint16_t YB)
{
    /* Set scroll window X */
    RA8875_writeReg(RA8875_HSSW0, (uint8_t)(XL & 0xFF));        // horizontal start point
    RA8875_writeReg(RA8875_HSSW1, (uint8_t)(XL >> 8));
    RA8875_writeReg(RA8875_HESW0, (uint8_t)(XR & 0xFF));        // horizontal end point
    RA8875_writeReg(RA8875_HESW1, (uint8_t)(XR >> 8));
  
    /* Set scroll window Y */
    RA8875_writeReg(RA8875_VSSW0, (uint8_t)(YT & 0xFF));        // vertical start point
    RA8875_writeReg(RA8875_VSSW1, (uint8_t)(YT >> 8));  
    RA8875_writeReg(RA8875_VESW0, (uint8_t)(YB & 0xFF));        // horizontal end point
    RA8875_writeReg(RA8875_VESW1, (uint8_t)(YB >> 8));
}


/**************************************************************************/
/*!
    Uses the hardware scroll function to scroll the screen 

    @args offsetX       Number of pixels to offset in the X direction
    @args offsetY       Number of pixels to offset in the Y direction
*/
/**************************************************************************/
void RA8875_scrollWindow(int16_t offsetX, int16_t offsetY)
{
    /* Read the current horizontal offset */
    int16_t hofs = (uint16_t)(RA8875_readReg(RA8875_HOFS1) << 8) | RA8875_readReg(RA8875_HOFS0);
    /* Read the current vertical offset */
    int16_t vofs = (uint16_t)(RA8875_readReg(RA8875_VOFS1) << 8) | RA8875_readReg(RA8875_VOFS0);

    /* Calculate the offsets */
    hofs -= offsetX;
    vofs -= offsetY;
    
    /* 
     * TODO: Check to make sure the offsets are valid
     * HOFS < (HESW - HSSW)
     * VOFS < (VESW - VSSW)
    */
    
    /* Write the horizontal offset */
    RA8875_writeReg(RA8875_HOFS0, (uint16_t)(hofs & 0xFF));
    RA8875_writeReg(RA8875_HOFS1, (uint16_t)(hofs >> 8));
    /* Write the vertical offset */
    RA8875_writeReg(RA8875_VOFS0, (uint16_t)(vofs & 0xFF));
    RA8875_writeReg(RA8875_VOFS1, (uint16_t)(vofs >> 8));
}


/**************************************************************************/
/*!
 *  Sets the scroll mode for use with layers
 * 
 *  @args mode[in]  The mode for layer scroll
*/
/**************************************************************************/
void RA8875_setScrollLayer(uint8_t mode)
{
    /* Read the current value of the LTPR0 register */
    uint8_t current = RA8875_readReg(RA8875_LTPR0);
    /* Set the Scroll Mode for Layers */
    RA8875_writeCommand(RA8875_LTPR0);  
    RA8875_writeData((mode & 0xC0) | (current & 0x3F));
}


/**************************************************************************/
/*!
 *  Sets the display configuration register
 * 
 *  @args config[in]    0x00 = 1 layer, 0x80 = 2 layers
*/
/**************************************************************************/
void RA8875_setDisplayConfig(uint8_t config)
{
    /* Read the current value of DPCR register */
    uint8_t current = RA8875_readReg(RA8875_DPCR);
    RA8875_writeReg(RA8875_DPCR, (config & 0x80) | (current & 0x7F));
}



void RA8875_setOrientation(int16_t angle)
{
    uint8_t fncr1Val = RA8875_readReg(RA8875_FNCR1);
    uint8_t dpcrVal = RA8875_readReg(RA8875_DPCR);
    
    switch (angle) 
    {
        case RA8875_ORIENTATION_UP:
            //fncr1Val = 0x00 | (fncr1Val & 0xEF);
            dpcrVal = 0x00 | (dpcrVal & 0x80);
            _portraitMode = FALSE;
            break;
            
        case RA8875_ORIENTATION_RIGHT:
            //fncr1Val = 0x10 | (fncr1Val & 0xEF);
            /* flip the horizontal scan direction */
            dpcrVal = 0x08 | (dpcrVal & 0x80);
            _portraitMode = TRUE;
            break;
            
        case RA8875_ORIENTATION_DOWN:
            //fncr1Val = 0x00 | (fncr1Val & 0xEF);
            /* flip the horizontal and vertical scan direction */
            dpcrVal = 0x0C | (dpcrVal & 0x80);
            _portraitMode = FALSE;
            break;
            
        case RA8875_ORIENTATION_LEFT:
            //fncr1Val = 0x10 | (fncr1Val & 0xEF);
            /* flip the vertical scan direction */
            dpcrVal = 0x04 | (dpcrVal & 0x80);
            _portraitMode = TRUE;
            break;
            
        default:
            // no action
            break;
    }
    
    if (_portraitMode) {
#if defined (RA8875_P480x272)
        _width = 272;
        _height = 480;
    
#elif defined (RA8875_P800x480)
        _width = 480;
        _height = 800;
    
#else 
#error RA8875 display size not defined!
#endif        
        fncr1Val |= (1 << 4);
    } else {
#if defined (RA8875_P480x272)
        _width = 480;
        _height = 272;
    
#elif defined (RA8875_P800x480)
        _width = 800;
        _height = 480;
    
#else 
#error RA8875 display size not defined!
#endif        
        fncr1Val &= ~(1 << 4);
    }

    /* Set the Font Rotation */
    RA8875_writeReg(RA8875_FNCR1, fncr1Val);
        
    /* Set the Scan Direction */
    RA8875_writeReg(RA8875_DPCR, dpcrVal);
    
    /* Set the Active Window */
//    if (_portraitMode) {
//        RA8875_setActiveWindow(0, _height - 1, 0, _width - 1);        
//    } else {
//        RA8875_setActiveWindow(0, _width - 1, 0, _height - 1);        
//    }
    
    /* Set the Touch Screen Calibration */
    

    return;
}


uint8_t RA8875_isPortraitMode(void)
{
    return _portraitMode;
}


/**************************************************************************/
/*!
 *  Enable / Disable the TFT - display enable tied to GPIOX
*/
/**************************************************************************/
void RA8875_gpiox (uint8_t on) 
{
    if (on) {
        RA8875_writeReg(RA8875_GPIOX, 1);
    } else {
        RA8875_writeReg(RA8875_GPIOX, 0);
    }
    return;
}


/**************************************************************************/
/*!
 *  Sets the display backlight brightness via PWM
 * 
 *  @args p       0 (OFF) - 255 (brightest)
*/
/**************************************************************************/
void RA8875_PWM1out(uint8_t p) 
{
    RA8875_writeReg(RA8875_P1DCR, p);
}

/**************************************************************************/
/*!
*/
/**************************************************************************/
void RA8875_PWM2out(uint8_t p) 
{
    RA8875_writeReg(RA8875_P2DCR, p);
}

/**************************************************************************/
/*!
 *  Configures the PWM for the display backlight
*/
/**************************************************************************/
void RA8875_PWM1config(uint8_t on, uint8_t clock) 
{
    if (on) {
        RA8875_writeReg(RA8875_P1CR, RA8875_P1CR_ENABLE | (clock & 0xF));
    } else {
        RA8875_writeReg(RA8875_P1CR, RA8875_P1CR_DISABLE | (clock & 0xF));
    }
}

/**************************************************************************/
/*!
*/
/**************************************************************************/
void RA8875_PWM2config(uint8_t on, uint8_t clock) 
{
    if (on) {
        RA8875_writeReg(RA8875_P2CR, RA8875_P2CR_ENABLE | (clock & 0xF));
    } else {
        RA8875_writeReg(RA8875_P2CR, RA8875_P2CR_DISABLE | (clock & 0xF));
    }
}


/**************************************************************************/
/*!
      Enables or disables the on-chip touch screen controller
*/
/**************************************************************************/
void RA8875_touchEnable(uint8_t on) 
{
    if (on) {
        /* Enable Touch Panel (Reg 0x70) */
        RA8875_writeReg(RA8875_TPCR0, RA8875_TPCR0_ENABLE        | 
                    RA8875_TPCR0_WAIT_4096CLK  |
                    RA8875_TPCR0_WAKEDISABLE   | 
                    RA8875_TPCR0_ADCCLK_DIV4); // 10mhz max!
        /* Set Auto Mode      (Reg 0x71) */
        RA8875_writeReg(RA8875_TPCR1, RA8875_TPCR1_AUTO    | 
                 // RA8875_TPCR1_VREFEXT | 
                    RA8875_TPCR1_DEBOUNCE);
        /* Enable TP INT */
        RA8875_writeReg(RA8875_INTC1, RA8875_readReg(RA8875_INTC1) | RA8875_INTC1_TP);
    } else {
        /* Disable TP INT */
        RA8875_writeReg(RA8875_INTC1, RA8875_readReg(RA8875_INTC1) & ~RA8875_INTC1_TP);
        /* Disable Touch Panel (Reg 0x70) */
        RA8875_writeReg(RA8875_TPCR0, RA8875_TPCR0_DISABLE);
    }
}


/**************************************************************************/
/*!
      Checks if a touch event has occurred
      
      @returns  True is a touch event has occurred (reading it via
                touchRead() will clear the interrupt in memory)
*/
/**************************************************************************/
uint8_t RA8875_touched(void) 
{
    if (RA8875_readReg(RA8875_INTC2) & RA8875_INTC2_TP)
        return TRUE;
    return FALSE;
}

/**************************************************************************/
/*!
      Reads the last touch event
      
      @args x[out]  Pointer to the uint16_t field to assign the raw X value
      @args y[out]  Pointer to the uint16_t field to assign the raw Y value
      
      @note Calling this function will clear the touch panel interrupt on
            the RA8875, resetting the flag used by the 'touched' function
*/
/**************************************************************************/
uint8_t RA8875_touchRead(uint16_t *x, uint16_t *y) 
{
    uint16_t tx, ty;
    uint8_t temp;
  
    tx = RA8875_readReg(RA8875_TPXH);
    ty = RA8875_readReg(RA8875_TPYH);
    temp = RA8875_readReg(RA8875_TPXYL);
    tx <<= 2;
    ty <<= 2;
    tx |= temp & 0x03;        // get the bottom x bits
    ty |= (temp >> 2) & 0x03; // get the bottom y bits

    *x = tx;
    *y = ty;

    /* Clear TP INT Status */
    RA8875_writeReg(RA8875_INTC2, RA8875_INTC2_TP);

    return TRUE;
}


/**************************** Text Mode ***********************************/


/**************************************************************************/
/*!
      Sets the display in text mode (as opposed to graphics mode)
*/
/**************************************************************************/
void RA8875_textMode(void) 
{
    /* Set text mode */
    RA8875_writeCommand(RA8875_MWCR0);
    uint8_t temp = RA8875_readData();
    temp |= RA8875_MWCR0_TXTMODE;     // Set bit 7
    RA8875_writeData(temp);
  
    /* Select the internal (ROM) font */
    RA8875_writeCommand(0x21);
    temp = RA8875_readData();
    temp &= ~((1<<7) | (1<<5));       // Clear bits 7 and 5
    RA8875_writeData(temp);
}


/**************************************************************************/
/*!
      Sets the display in text mode (as opposed to graphics mode)
      
      @args x[in] The x position of the cursor (in pixels, 0..1023)
      @args y[in] The y position of the cursor (in pixels, 0..511)
*/
/**************************************************************************/
void RA8875_textSetCursor(uint16_t x, uint16_t y) 
{
    /* Set cursor location */
    RA8875_writeCommand(0x2A);
    RA8875_writeData(x & 0xFF);
    RA8875_writeCommand(0x2B);
    RA8875_writeData(x >> 8);
    RA8875_writeCommand(0x2C);
    RA8875_writeData(y & 0xFF);
    RA8875_writeCommand(0x2D);
    RA8875_writeData(y >> 8);
}


/**************************************************************************/
/*!
    Returns the current X position of the cursor
*/
/**************************************************************************/
uint16_t RA8875_textGetCursor_X(void)
{
    return RA8875_readReg(0x2A) | (RA8875_readReg(0x2B) << 8);
}


/**************************************************************************/
/*!
    Returns the current Y position of the cursor
*/
/**************************************************************************/
uint16_t RA8875_textGetCursor_Y(void)
{
    uint16_t tmp = (RA8875_readReg(0x2C)) | (RA8875_readReg(0x2D) << 8);
    return tmp;
}


/**************************************************************************/
/*!
      Sets the fore and background color when rendering text
      
      @args foreColor[in] The RGB565 color to use when rendering the text
      @args bgColor[in]   The RGB565 color to use for the background
*/
/**************************************************************************/
void RA8875_textColor(uint16_t foreColor, uint16_t bgColor)
{
    /* Set Fore Color */
    RA8875_writeCommand(0x63);
    RA8875_writeData((foreColor & 0xf800) >> 11);
    RA8875_writeCommand(0x64);
    RA8875_writeData((foreColor & 0x07e0) >> 5);
    RA8875_writeCommand(0x65);
    RA8875_writeData((foreColor & 0x001f));
  
    /* Set Background Color */
    RA8875_writeCommand(0x60);
    RA8875_writeData((bgColor & 0xf800) >> 11);
    RA8875_writeCommand(0x61);
    RA8875_writeData((bgColor & 0x07e0) >> 5);
    RA8875_writeCommand(0x62);
    RA8875_writeData((bgColor & 0x001f));
  
    /* Clear transparency flag */
    RA8875_writeCommand(0x22);
    uint8_t temp = RA8875_readData();
    temp &= ~(1<<6); // Clear bit 6
    RA8875_writeData(temp);
}

/**************************************************************************/
/*!
      Sets the fore color when rendering text with a transparent bg
      
      @args foreColor[in] The RGB565 color to use when rendering the text
*/
/**************************************************************************/
void RA8875_textTransparent(uint16_t foreColor)
{
    /* Set Fore Color */
    RA8875_writeCommand(0x63);
    RA8875_writeData((foreColor & 0xf800) >> 11);
    RA8875_writeCommand(0x64);
    RA8875_writeData((foreColor & 0x07e0) >> 5);
    RA8875_writeCommand(0x65);
    RA8875_writeData((foreColor & 0x001f));

    /* Set transparency flag */
    RA8875_writeCommand(0x22);
    uint8_t temp = RA8875_readData();
    temp |= (1<<6); // Set bit 6
    RA8875_writeData(temp);  
}

/**************************************************************************/
/*!
      Sets the text enlarge settings, using one of the following values:
      
      0 = 1x zoom
      1 = 2x zoom
      2 = 3x zoom
      3 = 4x zoom
      
      @args scale[in]   The zoom factor (0..3 for 1-4x zoom)
*/
/**************************************************************************/
void RA8875_textEnlarge(uint8_t scale)
{
    if (scale > 3) 
        scale = 3;

    /* Set font size flags */
    RA8875_writeCommand(0x22);
    uint8_t temp = RA8875_readData();
    temp &= ~(0xF); // Clears bits 0..3
    temp |= scale << 2;
    temp |= scale;
    RA8875_writeData(temp);  

    _textScale = scale;
}


/**************************************************************************/
/*!
    Renders some text on the screen when in text mode
      
    @args buffer[in]    The buffer containing the characters to render
    @args len[in]       The size of the buffer in bytes
*/
/**************************************************************************/
void RA8875_textWrite(const char *buffer, uint16_t len) 
{
    if (len == 0) len = strlen(buffer);
    RA8875_writeCommand(RA8875_MRWC);
    for (uint16_t i = 0; i < len; i++) {
        RA8875_writeData(buffer[i]);
        // if we are writing large / extra large text
        // we need to delay 
#if defined (RA8875_USE_WAIT_PIN)
        while(! RA8875_WAIT);
#else
        if (_textScale > RA8875_TEXT_MEDIUM) 
            delay(1);
#endif
    }
}

/***************************** Graphics ***********************************/


/**************************************************************************/
/*!
      Sets the display in graphics mode (as opposed to text mode)
*/
/**************************************************************************/
void RA8875_graphicsMode(void) 
{
    RA8875_writeCommand(RA8875_MWCR0);
    uint8_t temp = RA8875_readData();
    temp &= ~RA8875_MWCR0_TXTMODE;  // bit #7
    RA8875_writeData(temp);
}


/**************************************************************************/
/*!
      Waits for screen to finish by polling the status!
*/
/**************************************************************************/
uint8_t RA8875_waitPoll(uint8_t regname, uint8_t waitflag) 
{
  /* Wait for the command to finish */
#if defined (RA8875_USE_WAIT_PIN)
    /* we're using the hardware WAIT pin */
    while(! RA8875_WAIT);
    return TRUE;
#else
    /* we're polling the status register */
    while (1) {
      uint8_t temp = RA8875_readReg(regname);
      if (!(temp & waitflag))
        return TRUE;
    }  
#endif
    return FALSE; // MEMEFIX: yeah i know, unreached! - add timeout?
}


/**************************************************************************/
/*!
      Sets the current X/Y position on the display before drawing
      
      @args x[in] The 0-based x location
      @args y[in] The 0-base y location
*/
/**************************************************************************/
void RA8875_setXY(uint16_t x, uint16_t y) 
{
    RA8875_writeReg(RA8875_CURH0, x);
    RA8875_writeReg(RA8875_CURH1, x >> 8);
    RA8875_writeReg(RA8875_CURV0, y);
    RA8875_writeReg(RA8875_CURV1, y >> 8);  
}


/**************************************************************************/
/*!
      Draws a single pixel at the specified location
      @args x[in]     The 0-based x location
      @args y[in]     The 0-base y location
      @args color[in] The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void RA8875_drawPixel(int16_t x, int16_t y, uint16_t color)
{
    RA8875_writeReg(RA8875_CURH0, x);
    RA8875_writeReg(RA8875_CURH1, x >> 8);
    RA8875_writeReg(RA8875_CURV0, y);
    RA8875_writeReg(RA8875_CURV1, y >> 8);  
    RA8875_writeCommand(RA8875_MRWC);
    RA8875_CS_ENABLE();
    SPI1_Exchange8bit(RA8875_DATAWRITE);
    SPI1_Exchange8bit(color >> 8);
    SPI1_Exchange8bit(color);
    RA8875_CS_DISABLE();
}


/**************************************************************************/
/*!
      HW accelerated function to push a chunk of raw pixel data
      
      @args num[in] The number of pixels to push
      @args p[in]   The pixel color to use
*/
/**************************************************************************/
void RA8875_pushPixels(uint32_t num, uint16_t p) 
{
    RA8875_CS_ENABLE();
    SPI1_Exchange8bit(RA8875_DATAWRITE);
    while (num--) {
        SPI1_Exchange8bit(p >> 8);
        SPI1_Exchange8bit(p);
    }
    RA8875_CS_DISABLE();
}


/**************************************************************************/
/*!
      Draws a HW accelerated line on the display
    
      @args x0[in]    The 0-based starting x location
      @args y0[in]    The 0-base starting y location
      @args x1[in]    The 0-based ending x location
      @args y1[in]    The 0-base ending y location
      @args color[in] The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void RA8875_drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
    /* Set X */
    RA8875_writeCommand(0x91);
    RA8875_writeData(x0);
    RA8875_writeCommand(0x92);
    RA8875_writeData(x0 >> 8);
  
    /* Set Y */
    RA8875_writeCommand(0x93);
    RA8875_writeData(y0); 
    RA8875_writeCommand(0x94);
    RA8875_writeData(y0 >> 8);
  
    /* Set X1 */
    RA8875_writeCommand(0x95);
    RA8875_writeData(x1);
    RA8875_writeCommand(0x96);
    RA8875_writeData((x1) >> 8);
  
    /* Set Y1 */
    RA8875_writeCommand(0x97);
    RA8875_writeData(y1); 
    RA8875_writeCommand(0x98);
    RA8875_writeData((y1) >> 8);
  
    /* Set Color */
    RA8875_writeCommand(0x63);
    RA8875_writeData((color & 0xf800) >> 11);
    RA8875_writeCommand(0x64);
    RA8875_writeData((color & 0x07e0) >> 5);
    RA8875_writeCommand(0x65);
    RA8875_writeData((color & 0x001f));

    /* Draw! */
    RA8875_writeCommand(RA8875_DCR);
    RA8875_writeData(0x80);
  
    /* Wait for the command to finish */
    RA8875_waitPoll(RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
}


/**************************************************************************/
/*!
*/
/**************************************************************************/
void RA8875_drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
    RA8875_drawLine(x, y, x, y+h, color);
}


/**************************************************************************/
/*!
*/
/**************************************************************************/
void RA8875_drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{
    RA8875_drawLine(x, y, x+w, y, color);
}


/**************************************************************************/
/*!
      Fills the screen with the specified RGB565 color
      @args color[in] The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void RA8875_fillScreen(uint16_t color)
{
    if(_portraitMode) {
        RA8875_rectHelper(0, 0, _height-1, _width-1, color, TRUE);
    } else {
        RA8875_rectHelper(0, 0, _width-1, _height-1, color, TRUE);
    }
}


/**************************************************************************/
/*!
      Draws a HW accelerated circle on the display
      @args x[in]     The 0-based x location of the center of the circle
      @args y[in]     The 0-based y location of the center of the circle
      @args r[in]     The circle's radius
      @args color[in] The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void RA8875_drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
    RA8875_circleHelper(x0, y0, r, color, FALSE);
}


/**************************************************************************/
/*!
      Draws a HW accelerated filled circle on the display
      @args x[in]     The 0-based x location of the center of the circle
      @args y[in]     The 0-based y location of the center of the circle
      @args r[in]     The circle's radius
      @args color[in] The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void RA8875_fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
    RA8875_circleHelper(x0, y0, r, color, TRUE);
}


/**************************************************************************/
/*!
      Draws a HW accelerated rectangle on the display
      @args x[in]     The 0-based x location of the top-left corner
      @args y[in]     The 0-based y location of the top-left corner
      @args w[in]     The rectangle width
      @args h[in]     The rectangle height
      @args color[in] The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void RA8875_drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    RA8875_rectHelper(x, y, x+w, y+h, color, FALSE);
}


/**************************************************************************/
/*!
      Draws a HW accelerated filled rectangle on the display
      @args x[in]     The 0-based x location of the top-left corner
      @args y[in]     The 0-based y location of the top-left corner
      @args w[in]     The rectangle width
      @args h[in]     The rectangle height
      @args color[in] The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void RA8875_fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    RA8875_rectHelper(x, y, x+w, y+h, color, TRUE);
}


/**************************************************************************/
/*!
      Draws a HW accelerated rounded rectangle on the display
      @args x[in]     The 0-based x location of the top-left corner
      @args y[in]     The 0-based y location of the top-left corner
      @args w[in]     The rectangle width
      @args h[in]     The rectangle height
      @args r1[in]    The corner radius (long axis)
      @args r2[in]    The corner radius (short axis)
      @args color[in] The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void RA8875_drawRoundedRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r1, int16_t r2, uint16_t color)
{
    RA8875_roundedRectHelper(x, y, x+w, y+h, r1, r2, color, FALSE);
}


/**************************************************************************/
/*!
      Draws a HW accelerated filled rounded rectangle on the display
      @args x[in]     The 0-based x location of the top-left corner
      @args y[in]     The 0-based y location of the top-left corner
      @args w[in]     The rectangle width
      @args h[in]     The rectangle height
      @args r1[in]    The corner radius (long axis)
      @args r2[in]    The corner radius (short axis)
      @args color[in] The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void RA8875_fillRoundedRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r1, int16_t r2, uint16_t color)
{
    RA8875_roundedRectHelper(x, y, x+w, y+h, r1, r2, color, TRUE);
}


/**************************************************************************/
/*!
      Draws a HW accelerated triangle on the display
      @args x0[in]    The 0-based x location of point 0 on the triangle
      @args y0[in]    The 0-based y location of point 0 on the triangle
      @args x1[in]    The 0-based x location of point 1 on the triangle
      @args y1[in]    The 0-based y location of point 1 on the triangle
      @args x2[in]    The 0-based x location of point 2 on the triangle
      @args y2[in]    The 0-based y location of point 2 on the triangle
      @args color[in] The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void RA8875_drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
    RA8875_triangleHelper(x0, y0, x1, y1, x2, y2, color, FALSE);
}


/**************************************************************************/
/*!
      Draws a HW accelerated filled triangle on the display
      @args x0[in]    The 0-based x location of point 0 on the triangle
      @args y0[in]    The 0-based y location of point 0 on the triangle
      @args x1[in]    The 0-based x location of point 1 on the triangle
      @args y1[in]    The 0-based y location of point 1 on the triangle
      @args x2[in]    The 0-based x location of point 2 on the triangle
      @args y2[in]    The 0-based y location of point 2 on the triangle
      @args color[in] The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void RA8875_fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
    RA8875_triangleHelper(x0, y0, x1, y1, x2, y2, color, TRUE);
}


/**************************************************************************/
/*!
      Draws a HW accelerated ellipse on the display
      @args xCenter[in]   The 0-based x location of the ellipse's center
      @args yCenter[in]   The 0-based y location of the ellipse's center
      @args longAxis[in]  The size in pixels of the ellipse's long axis
      @args shortAxis[in] The size in pixels of the ellipse's short axis
      @args color[in]     The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void RA8875_drawEllipse(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint16_t color)
{
    RA8875_ellipseHelper(xCenter, yCenter, longAxis, shortAxis, color, FALSE);
}


/**************************************************************************/
/*!
      Draws a HW accelerated filled ellipse on the display
      @args xCenter[in]   The 0-based x location of the ellipse's center
      @args yCenter[in]   The 0-based y location of the ellipse's center
      @args longAxis[in]  The size in pixels of the ellipse's long axis
      @args shortAxis[in] The size in pixels of the ellipse's short axis
      @args color[in]     The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void RA8875_fillEllipse(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint16_t color)
{
    RA8875_ellipseHelper(xCenter, yCenter, longAxis, shortAxis, color, TRUE);
}


/**************************************************************************/
/*!
      Draws a HW accelerated curve on the display
      @args xCenter[in]   The 0-based x location of the ellipse's center
      @args yCenter[in]   The 0-based y location of the ellipse's center
      @args longAxis[in]  The size in pixels of the ellipse's long axis
      @args shortAxis[in] The size in pixels of the ellipse's short axis
      @args curvePart[in] The corner to draw, where in clock-wise motion:
                            0 = 180-270°
                            1 = 270-0°
                            2 = 0-90°
                            3 = 90-180°
      @args color[in]     The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void RA8875_drawCurve(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint8_t curvePart, uint16_t color)
{
    RA8875_curveHelper(xCenter, yCenter, longAxis, shortAxis, curvePart, color, FALSE);
}


/**************************************************************************/
/*!
      Draws a HW accelerated filled curve on the display
      @args xCenter[in]   The 0-based x location of the ellipse's center
      @args yCenter[in]   The 0-based y location of the ellipse's center
      @args longAxis[in]  The size in pixels of the ellipse's long axis
      @args shortAxis[in] The size in pixels of the ellipse's short axis
      @args curvePart[in] The corner to draw, where in clock-wise motion:
                            0 = 180-270°
                            1 = 270-0°
                            2 = 0-90°
                            3 = 90-180°
      @args color[in]     The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void RA8875_fillCurve(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint8_t curvePart, uint16_t color)
{
    RA8875_curveHelper(xCenter, yCenter, longAxis, shortAxis, curvePart, color, TRUE);
}


/* P R I V A T E **********************************************************/

/**************************************************************************/
/*!
    Helper function for higher level circle drawing code
*/
/**************************************************************************/
void RA8875_circleHelper(int16_t x0, int16_t y0, int16_t r, uint16_t color, uint8_t filled)
{
    /* Set X */
    RA8875_writeCommand(0x99);
    RA8875_writeData(x0);
    RA8875_writeCommand(0x9a);
    RA8875_writeData(x0 >> 8);
  
    /* Set Y */
    RA8875_writeCommand(0x9b);
    RA8875_writeData(y0); 
    RA8875_writeCommand(0x9c);	   
    RA8875_writeData(y0 >> 8);
  
    /* Set Radius */
    RA8875_writeCommand(0x9d);
    RA8875_writeData(r);  
  
    /* Set Color */
    RA8875_writeCommand(0x63);
    RA8875_writeData((color & 0xf800) >> 11);
    RA8875_writeCommand(0x64);
    RA8875_writeData((color & 0x07e0) >> 5);
    RA8875_writeCommand(0x65);
    RA8875_writeData((color & 0x001f));
  
    /* Draw! */
    RA8875_writeCommand(RA8875_DCR);
    if (filled) {
        RA8875_writeData(RA8875_DCR_CIRCLE_START | RA8875_DCR_FILL);
    } else {
        RA8875_writeData(RA8875_DCR_CIRCLE_START | RA8875_DCR_NOFILL);
    }
  
    /* Wait for the command to finish */
    RA8875_waitPoll(RA8875_DCR, RA8875_DCR_CIRCLE_STATUS);
}


/**************************************************************************/
/*!
      Helper function for higher level rectangle drawing code
*/
/**************************************************************************/
void RA8875_rectHelper(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color, uint8_t filled)
{
    /* Set X */
    RA8875_writeCommand(0x91);
    RA8875_writeData(x);
    RA8875_writeCommand(0x92);
    RA8875_writeData(x >> 8);
  
    /* Set Y */
    RA8875_writeCommand(0x93);
    RA8875_writeData(y); 
    RA8875_writeCommand(0x94);	   
    RA8875_writeData(y >> 8);
  
    /* Set X1 */
    RA8875_writeCommand(0x95);
    RA8875_writeData(w);
    RA8875_writeCommand(0x96);
    RA8875_writeData((w) >> 8);
  
    /* Set Y1 */
    RA8875_writeCommand(0x97);
    RA8875_writeData(h); 
    RA8875_writeCommand(0x98);
    RA8875_writeData((h) >> 8);

    /* Set Color */
    RA8875_writeCommand(0x63);
    RA8875_writeData((color & 0xf800) >> 11);
    RA8875_writeCommand(0x64);
    RA8875_writeData((color & 0x07e0) >> 5);
    RA8875_writeCommand(0x65);
    RA8875_writeData((color & 0x001f));

    /* Draw! */
    RA8875_writeCommand(RA8875_DCR);
    if (filled) {
        RA8875_writeData(0xB0);
    } else {
        RA8875_writeData(0x90);
    }
  
    /* Wait for the command to finish */
    RA8875_waitPoll(RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
}


/**************************************************************************/
/*!
      Helper function for higher level rectangle drawing code
*/
/**************************************************************************/
void RA8875_roundedRectHelper(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r1, int16_t r2, uint16_t color, uint8_t filled)
{
    /* Set X */
    RA8875_writeCommand(0x91);
    RA8875_writeData(x);
    RA8875_writeCommand(0x92);
    RA8875_writeData(x >> 8);
  
    /* Set Y */
    RA8875_writeCommand(0x93);
    RA8875_writeData(y); 
    RA8875_writeCommand(0x94);	   
    RA8875_writeData(y >> 8);
  
    /* Set X1 */
    RA8875_writeCommand(0x95);
    RA8875_writeData(w);
    RA8875_writeCommand(0x96);
    RA8875_writeData((w) >> 8);
  
    /* Set Y1 */
    RA8875_writeCommand(0x97);
    RA8875_writeData(h); 
    RA8875_writeCommand(0x98);
    RA8875_writeData((h) >> 8);

    /* Set Rounded Corner */
    RA8875_writeCommand(0xA1);
    RA8875_writeData(r1);
    RA8875_writeCommand(0xA2);
    RA8875_writeData((r1) >> 8);
    
    RA8875_writeCommand(0xA3);
    RA8875_writeData(r2);
    RA8875_writeCommand(0xA4);
    RA8875_writeData((r2) >> 8);

    /* Set Color */
    RA8875_writeCommand(0x63);
    RA8875_writeData((color & 0xf800) >> 11);
    RA8875_writeCommand(0x64);
    RA8875_writeData((color & 0x07e0) >> 5);
    RA8875_writeCommand(0x65);
    RA8875_writeData((color & 0x001f));

    /* Draw! */
    RA8875_writeCommand(RA8875_CIRCLESQUARE);
    if (filled) {
        RA8875_writeData(0xE0);
    } else {
        RA8875_writeData(0xA0);
    }
  
    /* Wait for the command to finish */
    RA8875_waitPoll(RA8875_CIRCLESQUARE, RA8875_CIRCLESQUARE_STATUS);
}


/**************************************************************************/
/*!
      Helper function for higher level triangle drawing code
*/
/**************************************************************************/
void RA8875_triangleHelper(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color, uint8_t filled)
{
    /* Set Point 0 */
    RA8875_writeCommand(0x91);
    RA8875_writeData(x0);
    RA8875_writeCommand(0x92);
    RA8875_writeData(x0 >> 8);
    RA8875_writeCommand(0x93);
    RA8875_writeData(y0); 
    RA8875_writeCommand(0x94);
    RA8875_writeData(y0 >> 8);

    /* Set Point 1 */
    RA8875_writeCommand(0x95);
    RA8875_writeData(x1);
    RA8875_writeCommand(0x96);
    RA8875_writeData(x1 >> 8);
    RA8875_writeCommand(0x97);
    RA8875_writeData(y1); 
    RA8875_writeCommand(0x98);
    RA8875_writeData(y1 >> 8);

    /* Set Point 2 */
    RA8875_writeCommand(0xA9);
    RA8875_writeData(x2);
    RA8875_writeCommand(0xAA);
    RA8875_writeData(x2 >> 8);
    RA8875_writeCommand(0xAB);
    RA8875_writeData(y2); 
    RA8875_writeCommand(0xAC);
    RA8875_writeData(y2 >> 8);
  
    /* Set Color */
    RA8875_writeCommand(0x63);
    RA8875_writeData((color & 0xf800) >> 11);
    RA8875_writeCommand(0x64);
    RA8875_writeData((color & 0x07e0) >> 5);
    RA8875_writeCommand(0x65);
    RA8875_writeData((color & 0x001f));
  
    /* Draw! */
    RA8875_writeCommand(RA8875_DCR);
    if (filled) {
        RA8875_writeData(0xA1);
    } else {
        RA8875_writeData(0x81);
    }
  
    /* Wait for the command to finish */
    RA8875_waitPoll(RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
}

/**************************************************************************/
/*!
      Helper function for higher level ellipse drawing code
*/
/**************************************************************************/
void RA8875_ellipseHelper(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint16_t color, uint8_t filled)
{
    /* Set Center Point */
    RA8875_writeCommand(0xA5);
    RA8875_writeData(xCenter);
    RA8875_writeCommand(0xA6);
    RA8875_writeData(xCenter >> 8);
    RA8875_writeCommand(0xA7);
    RA8875_writeData(yCenter); 
    RA8875_writeCommand(0xA8);
    RA8875_writeData(yCenter >> 8);

    /* Set Long and Short Axis */
    RA8875_writeCommand(0xA1);
    RA8875_writeData(longAxis);
    RA8875_writeCommand(0xA2);
    RA8875_writeData(longAxis >> 8);
    RA8875_writeCommand(0xA3);
    RA8875_writeData(shortAxis); 
    RA8875_writeCommand(0xA4);
    RA8875_writeData(shortAxis >> 8);
  
    /* Set Color */
    RA8875_writeCommand(0x63);
    RA8875_writeData((color & 0xf800) >> 11);
    RA8875_writeCommand(0x64);
    RA8875_writeData((color & 0x07e0) >> 5);
    RA8875_writeCommand(0x65);
    RA8875_writeData((color & 0x001f));
  
    /* Draw! */
    RA8875_writeCommand(0xA0);
    if (filled) {
        RA8875_writeData(0xC0);
    } else {
        RA8875_writeData(0x80);
    }
  
    /* Wait for the command to finish */
    RA8875_waitPoll(RA8875_ELLIPSE, RA8875_ELLIPSE_STATUS);
}

/**************************************************************************/
/*!
      Helper function for higher level curve drawing code
*/
/**************************************************************************/
void RA8875_curveHelper(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint8_t curvePart, uint16_t color, uint8_t filled)
{
    /* Set Center Point */
    RA8875_writeCommand(0xA5);
    RA8875_writeData(xCenter);
    RA8875_writeCommand(0xA6);
    RA8875_writeData(xCenter >> 8);
    RA8875_writeCommand(0xA7);
    RA8875_writeData(yCenter); 
    RA8875_writeCommand(0xA8);
    RA8875_writeData(yCenter >> 8);

    /* Set Long and Short Axis */
    RA8875_writeCommand(0xA1);
    RA8875_writeData(longAxis);
    RA8875_writeCommand(0xA2);
    RA8875_writeData(longAxis >> 8);
    RA8875_writeCommand(0xA3);
    RA8875_writeData(shortAxis); 
    RA8875_writeCommand(0xA4);
    RA8875_writeData(shortAxis >> 8);
  
    /* Set Color */
    RA8875_writeCommand(0x63);
    RA8875_writeData((color & 0xf800) >> 11);
    RA8875_writeCommand(0x64);
    RA8875_writeData((color & 0x07e0) >> 5);
    RA8875_writeCommand(0x65);
    RA8875_writeData((color & 0x001f));

    /* Draw! */
    RA8875_writeCommand(0xA0);
    if (filled) {
        RA8875_writeData(0xD0 | (curvePart & 0x03));
    } else {
        RA8875_writeData(0x90 | (curvePart & 0x03));
    }
  
    /* Wait for the command to finish */
    RA8875_waitPoll(RA8875_ELLIPSE, RA8875_ELLIPSE_STATUS);
}


/**************************************************************************/
/*!
 *  Initialize the display PLL
*/
/**************************************************************************/
void RA8875_PLL_init(void)
{
#ifdef RA8875_P480x272
    RA8875_writeReg(RA8875_PLLC1, RA8875_PLLC1_PLLDIV1 + 10);
    delay(1);
    RA8875_writeReg(RA8875_PLLC2, RA8875_PLLC2_DIV4);
    delay(1);
#endif
    
#ifdef RA8875_P800x480
    RA8875_writeReg(RA8875_PLLC1, RA8875_PLLC1_PLLDIV1 + 10);
    delay(1);
    RA8875_writeReg(RA8875_PLLC2, RA8875_PLLC2_DIV4);
    delay(1);
#endif
    return;
}


/**************************************************************************/
/*!
 *  Initialize the display
*/
/**************************************************************************/
void RA8875_initialize(uint8_t color, uint8_t mcu)
{
    RA8875_PLL_init();
    /* Set the default color depth to 16-bpp & the MCU interface to 8-bit */
    RA8875_writeReg(RA8875_SYSR, color | mcu); 
    
    /* Timing values */
    uint8_t pixclk;
    uint8_t hsync_start;
    uint8_t hsync_pw;
    uint8_t hsync_finetune;
    uint8_t hsync_nondisp;
    uint8_t vsync_pw; 
    uint16_t vsync_nondisp;
    uint16_t vsync_start;
  
#if defined (RA8875_P480x272)
    pixclk          = RA8875_PCSR_PDATL | RA8875_PCSR_4CLK;
    hsync_nondisp   = 10;
    hsync_start     = 8;
    hsync_pw        = 48;
    hsync_finetune  = 0;
    vsync_nondisp   = 3;
    vsync_start     = 8;
    vsync_pw        = 10;
    
#elif defined (RA8875_P800x480)
    pixclk          = RA8875_PCSR_PDATL | RA8875_PCSR_2CLK;
    hsync_nondisp   = 26;
    hsync_start     = 32;
    hsync_pw        = 96;
    hsync_finetune  = 0;
    vsync_nondisp   = 32;
    vsync_start     = 23;
    vsync_pw        = 2;
#endif
    
    RA8875_writeReg(RA8875_PCSR, pixclk);
    delay(1);
  
    /* Horizontal settings registers */
    RA8875_writeReg(RA8875_HDWR, (_width / 8) - 1);                          // H width: (HDWR + 1) * 8 = 480
    RA8875_writeReg(RA8875_HNDFTR, RA8875_HNDFTR_DE_HIGH + hsync_finetune);
    RA8875_writeReg(RA8875_HNDR, (hsync_nondisp - hsync_finetune - 2)/8);    // H non-display: HNDR * 8 + HNDFTR + 2 = 10
    RA8875_writeReg(RA8875_HSTR, hsync_start/8 - 1);                         // Hsync start: (HSTR + 1)*8 
    RA8875_writeReg(RA8875_HPWR, RA8875_HPWR_LOW + (hsync_pw/8 - 1));        // HSync pulse width = (HPWR+1) * 8
  
    /* Vertical settings registers */
    RA8875_writeReg(RA8875_VDHR0, (uint16_t)(_height - 1) & 0xFF);
    RA8875_writeReg(RA8875_VDHR1, (uint16_t)(_height - 1) >> 8);
    RA8875_writeReg(RA8875_VNDR0, vsync_nondisp-1);                          // V non-display period = VNDR + 1
    RA8875_writeReg(RA8875_VNDR1, vsync_nondisp >> 8);
    RA8875_writeReg(RA8875_VSTR0, vsync_start-1);                            // Vsync start position = VSTR + 1
    RA8875_writeReg(RA8875_VSTR1, vsync_start >> 8);
    RA8875_writeReg(RA8875_VPWR, RA8875_VPWR_LOW + vsync_pw - 1);            // Vsync pulse width = VPWR + 1
  
    /* Set active window X */
    RA8875_writeReg(RA8875_HSAW0, 0);                                        // horizontal start point
    RA8875_writeReg(RA8875_HSAW1, 0);
    RA8875_writeReg(RA8875_HEAW0, (uint16_t)(_width - 1) & 0xFF);            // horizontal end point
    RA8875_writeReg(RA8875_HEAW1, (uint16_t)(_width - 1) >> 8);
  
    /* Set active window Y */
    RA8875_writeReg(RA8875_VSAW0, 0);                                        // vertical start point
    RA8875_writeReg(RA8875_VSAW1, 0);  
    RA8875_writeReg(RA8875_VEAW0, (uint16_t)(_height - 1) & 0xFF);           // horizontal end point
    RA8875_writeReg(RA8875_VEAW1, (uint16_t)(_height - 1) >> 8);
  
    /* ToDo: Setup touch panel? */
  
    /* Clear the entire window */
    RA8875_clearActiveWindow(TRUE);
    delay(500);
    return;
}

// performs a hardware reset of the RA8875
void RA8875_hardReset(void)
{
	RA8875_RST = 0;   // RA8875 RESET pin
	delay(100);
	RA8875_RST = 1;
	delay(100);
}

// performs a software reset of the RA8875
void RA8875_softReset(void)
{
	RA8875_writeCommand(RA8875_PWRR);
    RA8875_writeData(0x01);
    RA8875_writeData(0x00);
    delay(1);
}


/* L O W   L E V E L *********************************************************/

void RA8875_writeData(uint8_t data)
{
    RA8875_CS_ENABLE();
    SPI1_Exchange8bit(RA8875_DATAWRITE);
    SPI1_Exchange8bit(data);
    RA8875_CS_DISABLE();
    return;
}

uint8_t RA8875_readData(void)
{
    RA8875_CS_ENABLE();
    SPI1_Exchange8bit(RA8875_DATAREAD);
    uint8_t x = SPI1_Exchange8bit(0xFF);
    RA8875_CS_DISABLE();    
    return x;
}

void RA8875_writeCommand(uint8_t cmd)
{
    RA8875_CS_ENABLE();
    SPI1_Exchange8bit(RA8875_CMDWRITE);
    SPI1_Exchange8bit(cmd);
    RA8875_CS_DISABLE();  
    return;
}

/**************************************************************************/
/*!
 *  Returns the value of the status register 
 *  @returns        display status byte
*/
/**************************************************************************/
uint8_t RA8875_readStatus(void)
{
    RA8875_CS_ENABLE();
    SPI1_Exchange8bit(RA8875_CMDREAD);
    uint8_t x = SPI1_Exchange8bit(0xFF);
    RA8875_CS_DISABLE();
    return x;
}
