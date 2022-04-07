/*********************************************************************
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
*                           www.segger.com                           *
**********************************************************************
*                                                                    *
* C-file generated by                                                *
*                                                                    *
*        Bitmap Converter (ST) for emWin V5.44.                      *
*        Compiled Nov 10 2017, 08:52:20                              *
*                                                                    *
*        (c) 1998 - 2017 Segger Microcontroller GmbH & Co. KG        *
*                                                                    *
**********************************************************************
*                                                                    *
* Source file: wall                                                  *
* Dimensions:  25 * 25                                               *
* NumColors:   65536 colors + 8 bit alpha channel                    *
*                                                                    *
**********************************************************************
*/

#include <stdlib.h>

#include "GUI.h"

#ifndef GUI_CONST_STORAGE
  #define GUI_CONST_STORAGE const
#endif

extern GUI_CONST_STORAGE GUI_BITMAP bmwall;

static GUI_CONST_STORAGE unsigned char _acwall[] = {
  0x00,0xB5,0xB6, 0x00,0xD6,0x9A, 0x00,0xCE,0x79, 0x00,0xCE,0x79, 0x00,0xCE,0x79, 0x00,0xCE,0x79, 0x00,0xCE,0x79, 0x00,0xCE,0x79, 0x00,0xCE,0x79, 0x00,0xCE,0x79, 0x00,0xCE,0x79, 0x00,0xCE,0x79, 0x00,0xCE,0x79, 0x00,0xCE,0x79, 0x00,0xCE,0x79, 
        0x00,0xCE,0x79, 0x00,0xCE,0x79, 0x00,0xCE,0x79, 0x00,0xCE,0x79, 0x00,0xCE,0x79, 0x00,0xCE,0x79, 0x00,0xCE,0x79, 0x00,0xCE,0x79, 0x00,0xC6,0x38, 0x00,0xBD,0xF7,
  0x00,0xCE,0x59, 0x00,0xFF,0xFF, 0x00,0xFF,0xFF, 0x00,0xFF,0xFF, 0x00,0xFF,0xFF, 0x00,0xFF,0xFF, 0x00,0xFF,0xFF, 0x00,0xFF,0xFF, 0x00,0xFF,0xFF, 0x00,0xFF,0xFF, 0x00,0xFF,0xFF, 0x00,0xFF,0xFF, 0x00,0xFF,0xFF, 0x00,0xFF,0xFF, 0x00,0xFF,0xFF, 
        0x00,0xFF,0xFF, 0x00,0xFF,0xFF, 0x00,0xFF,0xFF, 0x00,0xFF,0xFF, 0x00,0xFF,0xFF, 0x00,0xFF,0xFF, 0x00,0xFF,0xFF, 0x00,0xF7,0xBE, 0x00,0xAD,0x75, 0x00,0xA5,0x34,
  0x00,0xCE,0x59, 0x00,0xFF,0xFF, 0x00,0xFF,0xDF, 0x00,0xEF,0x9D, 0x00,0xEF,0x7D, 0x00,0xEF,0x7D, 0x00,0xEF,0x7D, 0x00,0xEF,0x7D, 0x00,0xEF,0x7D, 0x00,0xEF,0x7D, 0x00,0xEF,0x7D, 0x00,0xEF,0x7D, 0x00,0xEF,0x7D, 0x00,0xEF,0x7D, 0x00,0xEF,0x7D, 
        0x00,0xEF,0x7D, 0x00,0xEF,0x7D, 0x00,0xEF,0x7D, 0x00,0xEF,0x7D, 0x00,0xEF,0x7D, 0x00,0xEF,0x7D, 0x00,0xEF,0x5D, 0x00,0xAD,0x75, 0x00,0x63,0x2C, 0x00,0xA5,0x14,
  0x00,0xCE,0x59, 0x00,0xFF,0xFF, 0x00,0xF7,0xBE, 0x00,0xC6,0x38, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 
        0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xB5,0xB6, 0x00,0x6B,0x6D, 0x00,0x63,0x0C, 0x00,0xA5,0x14,
  0x00,0xCE,0x59, 0x00,0xFF,0xFF, 0x00,0xF7,0x9E, 0x00,0xC6,0x18, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 
        0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xB5,0x96, 0x00,0x6B,0x6D, 0x00,0x63,0x0C, 0x00,0xA5,0x14,
  0x00,0xCE,0x59, 0x00,0xFF,0xFF, 0x00,0xF7,0x9E, 0x00,0xC6,0x18, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 
        0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xB5,0xB6, 0x00,0x6B,0x6D, 0x00,0x63,0x0C, 0x00,0xA5,0x14,
  0x00,0xCE,0x59, 0x00,0xFF,0xFF, 0x00,0xF7,0x9E, 0x00,0xC6,0x18, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 
        0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xB5,0xB6, 0x00,0x6B,0x6D, 0x00,0x63,0x0C, 0x00,0xA5,0x14,
  0x00,0xCE,0x59, 0x00,0xFF,0xFF, 0x00,0xF7,0x9E, 0x00,0xC6,0x18, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 
        0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xB5,0xB6, 0x00,0x6B,0x6D, 0x00,0x63,0x0C, 0x00,0xA5,0x14,
  0x00,0xCE,0x59, 0x00,0xFF,0xFF, 0x00,0xF7,0x9E, 0x00,0xC6,0x18, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 
        0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xB5,0xB6, 0x00,0x6B,0x6D, 0x00,0x63,0x0C, 0x00,0xA5,0x14,
  0x00,0xCE,0x59, 0x00,0xFF,0xFF, 0x00,0xF7,0x9E, 0x00,0xC6,0x18, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 
        0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xB5,0xB6, 0x00,0x6B,0x6D, 0x00,0x63,0x0C, 0x00,0xA5,0x14,
  0x00,0xCE,0x59, 0x00,0xFF,0xFF, 0x00,0xF7,0x9E, 0x00,0xC6,0x18, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 
        0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xB5,0xB6, 0x00,0x6B,0x6D, 0x00,0x63,0x0C, 0x00,0xA5,0x14,
  0x00,0xCE,0x59, 0x00,0xFF,0xFF, 0x00,0xF7,0x9E, 0x00,0xC6,0x18, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 
        0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xB5,0xB6, 0x00,0x6B,0x6D, 0x00,0x63,0x0C, 0x00,0xA5,0x14,
  0x00,0xCE,0x59, 0x00,0xFF,0xFF, 0x00,0xF7,0x9E, 0x00,0xC6,0x18, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 
        0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xB5,0xB6, 0x00,0x6B,0x6D, 0x00,0x63,0x0C, 0x00,0xA5,0x14,
  0x00,0xCE,0x59, 0x00,0xFF,0xFF, 0x00,0xF7,0x9E, 0x00,0xC6,0x18, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 
        0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xB5,0xB6, 0x00,0x6B,0x6D, 0x00,0x63,0x0C, 0x00,0xA5,0x14,
  0x00,0xCE,0x59, 0x00,0xFF,0xFF, 0x00,0xF7,0x9E, 0x00,0xC6,0x18, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 
        0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xB5,0xB6, 0x00,0x6B,0x6D, 0x00,0x63,0x0C, 0x00,0xA5,0x14,
  0x00,0xCE,0x59, 0x00,0xFF,0xFF, 0x00,0xF7,0x9E, 0x00,0xC6,0x18, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 
        0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xB5,0xB6, 0x00,0x6B,0x6D, 0x00,0x63,0x0C, 0x00,0xA5,0x14,
  0x00,0xCE,0x59, 0x00,0xFF,0xFF, 0x00,0xF7,0x9E, 0x00,0xC6,0x18, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 
        0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xB5,0xB6, 0x00,0x6B,0x6D, 0x00,0x63,0x0C, 0x00,0xA5,0x14,
  0x00,0xCE,0x59, 0x00,0xFF,0xFF, 0x00,0xF7,0x9E, 0x00,0xC6,0x18, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 
        0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xB5,0xB6, 0x00,0x6B,0x6D, 0x00,0x63,0x0C, 0x00,0xA5,0x14,
  0x00,0xCE,0x59, 0x00,0xFF,0xFF, 0x00,0xF7,0x9E, 0x00,0xC6,0x18, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 
        0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xB5,0xB6, 0x00,0x6B,0x6D, 0x00,0x63,0x0C, 0x00,0xA5,0x14,
  0x00,0xCE,0x59, 0x00,0xFF,0xFF, 0x00,0xF7,0x9E, 0x00,0xC6,0x18, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 
        0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xB5,0x96, 0x00,0x73,0x6D, 0x00,0x63,0x0C, 0x00,0xA5,0x14,
  0x00,0xCE,0x59, 0x00,0xFF,0xFF, 0x00,0xF7,0x9E, 0x00,0xC6,0x18, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 
        0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xBD,0xF7, 0x00,0xB5,0x96, 0x00,0x73,0x6D, 0x00,0x63,0x0C, 0x00,0xA5,0x14,
  0x00,0xCE,0x59, 0x00,0xFF,0xFF, 0x00,0xEF,0x9D, 0x00,0xB5,0xB6, 0x00,0xB5,0x96, 0x00,0xB5,0x96, 0x00,0xB5,0x96, 0x00,0xB5,0x96, 0x00,0xB5,0x96, 0x00,0xB5,0x96, 0x00,0xB5,0x96, 0x00,0xB5,0x96, 0x00,0xB5,0x96, 0x00,0xB5,0x96, 0x00,0xB5,0x96, 
        0x00,0xB5,0x96, 0x00,0xB5,0x96, 0x00,0xB5,0x96, 0x00,0xB5,0x96, 0x00,0xB5,0x76, 0x00,0xB5,0x76, 0x00,0xAD,0x55, 0x00,0x6B,0x4D, 0x00,0x63,0x0C, 0x00,0xA5,0x14,
  0x00,0xCE,0x59, 0x00,0xF7,0xBE, 0x00,0xAD,0x75, 0x00,0x6B,0x6D, 0x00,0x6B,0x6D, 0x00,0x6B,0x6D, 0x00,0x6B,0x6D, 0x00,0x6B,0x6D, 0x00,0x6B,0x6D, 0x00,0x6B,0x6D, 0x00,0x6B,0x6D, 0x00,0x6B,0x6D, 0x00,0x6B,0x6D, 0x00,0x6B,0x6D, 0x00,0x6B,0x6D, 
        0x00,0x6B,0x6D, 0x00,0x6B,0x6D, 0x00,0x6B,0x6D, 0x00,0x6B,0x6D, 0x00,0x6B,0x4D, 0x00,0x6B,0x4D, 0x00,0x6B,0x4D, 0x00,0x63,0x0C, 0x00,0x63,0x0C, 0x00,0x9D,0x13,
  0x00,0xC6,0x18, 0x00,0xAD,0x75, 0x00,0x63,0x2C, 0x00,0x63,0x0C, 0x00,0x63,0x0C, 0x00,0x63,0x0C, 0x00,0x63,0x0C, 0x00,0x63,0x0C, 0x00,0x63,0x0C, 0x00,0x63,0x0C, 0x00,0x63,0x0C, 0x00,0x63,0x0C, 0x00,0x63,0x0C, 0x00,0x63,0x0C, 0x00,0x63,0x0C, 
        0x00,0x63,0x0C, 0x00,0x63,0x0C, 0x00,0x63,0x0C, 0x00,0x63,0x0C, 0x00,0x63,0x0C, 0x00,0x63,0x0C, 0x00,0x63,0x0C, 0x00,0x63,0x0C, 0x00,0x63,0x0C, 0x00,0x9C,0xF3,
  0x00,0x9C,0xF3, 0x00,0x7B,0xCF, 0x00,0x7B,0xAF, 0x00,0x7B,0xAF, 0x00,0x7B,0xAF, 0x00,0x7B,0xAF, 0x00,0x7B,0xAF, 0x00,0x7B,0xCF, 0x00,0x7B,0xAF, 0x00,0x7B,0xAF, 0x00,0x7B,0xAF, 0x00,0x7B,0xAF, 0x00,0x7B,0xAF, 0x00,0x7B,0xAF, 0x00,0x7B,0xAF, 
        0x00,0x7B,0xAF, 0x00,0x7B,0xAF, 0x00,0x7B,0xAF, 0x00,0x7B,0xCF, 0x00,0x73,0xAF, 0x00,0x73,0xCF, 0x00,0x7B,0xCE, 0x00,0x7B,0xAF, 0x00,0x7B,0xAF, 0x00,0x7B,0xCF
};

GUI_CONST_STORAGE GUI_BITMAP bmwall = {
  25, // xSize
  25, // ySize
  75, // BytesPerLine
  24, // BitsPerPixel
  (unsigned char *)_acwall,  // Pointer to picture data
  NULL,  // Pointer to palette
  GUI_DRAW_BMPA565
};

/*************************** End of file ****************************/