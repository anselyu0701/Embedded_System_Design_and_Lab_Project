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
* Source file: 08                                                    *
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

extern GUI_CONST_STORAGE GUI_BITMAP bm08;

static GUI_CONST_STORAGE unsigned char _ac08[] = {
  0x00,0x9C,0xF3, 0x00,0xAD,0x55, 0x00,0xAD,0x75, 0x00,0xB5,0x75, 0x00,0xB5,0x75, 0x00,0xB5,0x75, 0x00,0xB5,0x75, 0x00,0xB5,0x76, 0x00,0xB5,0x76, 0x00,0xB5,0x76, 0x00,0xB5,0x76, 0x00,0xB5,0x76, 0x00,0xB5,0x76, 0x00,0xB5,0x76, 0x00,0xB5,0x76, 
        0x00,0xB5,0x76, 0x00,0xB5,0x76, 0x00,0xB5,0x76, 0x00,0xB5,0x75, 0x00,0xB5,0x75, 0x00,0xB5,0x75, 0x00,0xB5,0x75, 0x00,0xB5,0x75, 0x00,0x9C,0xB2, 0x00,0x7B,0xCF,
  0x00,0xAD,0x55, 0x00,0xBD,0xD7, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 
        0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xA5,0x14, 0x00,0x84,0x10,
  0x00,0xAD,0x55, 0x00,0xBD,0xF7, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 
        0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xA5,0x14, 0x00,0x84,0x30,
  0x00,0xAD,0x55, 0x00,0xBD,0xF7, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 
        0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xA5,0x14, 0x00,0x84,0x30,
  0x00,0xAD,0x55, 0x00,0xBD,0xF7, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 
        0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xA5,0x14, 0x00,0x84,0x30,
  0x00,0xAD,0x55, 0x00,0xBD,0xF7, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 
        0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xA5,0x14, 0x00,0x84,0x30,
  0x00,0xAD,0x55, 0x00,0xBD,0xF7, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 
        0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xA5,0x14, 0x00,0x84,0x30,
  0x00,0xAD,0x55, 0x00,0xBD,0xF7, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 
        0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xA5,0x14, 0x00,0x84,0x30,
  0x00,0xAD,0x55, 0x00,0xBD,0xF7, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 
        0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xA5,0x14, 0x00,0x84,0x30,
  0x00,0xAD,0x55, 0x00,0xBD,0xF7, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 
        0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xA5,0x14, 0x00,0x84,0x30,
  0x00,0xAD,0x55, 0x00,0xBD,0xF7, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0x54,0x58, 0x00,0x00,0x00, 0x00,0x00,0x00, 0x00,0x00,0x00, 0x00,0x8A,0x60, 
        0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xA5,0x14, 0x00,0x84,0x30,
  0x00,0xAD,0x55, 0x00,0xBD,0xF7, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0x2B,0x75, 0x00,0x28,0x00, 0x00,0xC5,0x4D, 0x00,0xC6,0x18, 0x00,0xAE,0x18, 0x00,0x01,0x6D, 
        0x00,0x8A,0x60, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xA5,0x14, 0x00,0x84,0x30,
  0x00,0xAD,0x55, 0x00,0xBD,0xF7, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0x02,0x71, 0x00,0xAB,0x65, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0x2B,0x75, 
        0x00,0x69,0x60, 0x00,0xC6,0x15, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xA5,0x14, 0x00,0x84,0x30,
  0x00,0xAD,0x55, 0x00,0xBD,0xF7, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0x2B,0x75, 0x00,0x69,0x60, 0x00,0xC6,0x15, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0x02,0x71, 
        0x00,0xAB,0x65, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xA5,0x14, 0x00,0x84,0x30,
  0x00,0xAD,0x55, 0x00,0xBD,0xF7, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xAE,0x18, 0x00,0x01,0x6D, 0x00,0x50,0x00, 0x00,0x8E,0x11, 0x00,0x00,0x09, 0x00,0x69,0x60, 
        0x00,0xC6,0x15, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xA5,0x14, 0x00,0x84,0x30,
  0x00,0xAD,0x55, 0x00,0xBD,0xF7, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0x6D,0x58, 0x00,0x00,0x05, 0x00,0x00,0x00, 0x00,0x28,0x00, 0x00,0xC5,0x4D, 
        0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xA5,0x14, 0x00,0x84,0x30,
  0x00,0xAD,0x55, 0x00,0xBD,0xF7, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0x2B,0x75, 0x00,0x50,0x00, 0x00,0xC6,0x11, 0x00,0xC6,0x18, 0x00,0x8E,0x18, 0x00,0x00,0x09, 
        0x00,0x8A,0x60, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xA5,0x14, 0x00,0x84,0x30,
  0x00,0xAD,0x55, 0x00,0xBD,0xF7, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0x8E,0x18, 0x00,0x28,0x09, 0x00,0xC5,0x4D, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0x6D,0x58, 
        0x00,0x28,0x05, 0x00,0xC5,0x4D, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xA5,0x14, 0x00,0x84,0x30,
  0x00,0xAD,0x55, 0x00,0xBD,0xF7, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0x6D,0x58, 0x00,0x28,0x05, 0x00,0xC5,0x4D, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0x6D,0x58, 
        0x00,0x28,0x05, 0x00,0xC5,0x4D, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xA5,0x14, 0x00,0x84,0x30,
  0x00,0xAD,0x55, 0x00,0xBD,0xF7, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xAE,0x18, 0x00,0x01,0x6D, 0x00,0x69,0x60, 0x00,0xC6,0x15, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0x02,0x71, 
        0x00,0x69,0x60, 0x00,0xC6,0x15, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xA5,0x14, 0x00,0x84,0x30,
  0x00,0xAD,0x55, 0x00,0xBD,0xF7, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xAE,0x18, 0x00,0x01,0x6D, 0x00,0x00,0x00, 0x00,0x00,0x00, 0x00,0x00,0x00, 0x00,0x8A,0x60, 
        0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xA5,0x14, 0x00,0x84,0x30,
  0x00,0xAD,0x55, 0x00,0xBD,0xD7, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 
        0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xC6,0x18, 0x00,0xA5,0x34, 0x00,0x84,0x10,
  0x00,0xA5,0x14, 0x00,0xAD,0x75, 0x00,0xB5,0x96, 0x00,0xB5,0x96, 0x00,0xB5,0x96, 0x00,0xB5,0x96, 0x00,0xB5,0x96, 0x00,0xB5,0x96, 0x00,0xB5,0x96, 0x00,0xB5,0x96, 0x00,0xB5,0x96, 0x00,0xB5,0x96, 0x00,0xB5,0x96, 0x00,0xB5,0x96, 0x00,0xB5,0x96, 
        0x00,0xB5,0x96, 0x00,0xB5,0x96, 0x00,0xB5,0x96, 0x00,0xB5,0x96, 0x00,0xB5,0x96, 0x00,0xB5,0x96, 0x00,0xB5,0x96, 0x00,0xB5,0x96, 0x00,0x9C,0xD3, 0x00,0x7B,0xEF,
  0x00,0x94,0x92, 0x00,0x8C,0x51, 0x00,0x94,0x92, 0x00,0x94,0x92, 0x00,0x94,0x92, 0x00,0x94,0x92, 0x00,0x94,0x92, 0x00,0x94,0x92, 0x00,0x94,0x92, 0x00,0x94,0x92, 0x00,0x94,0x92, 0x00,0x94,0x92, 0x00,0x94,0x92, 0x00,0x94,0x92, 0x00,0x94,0x92, 
        0x00,0x94,0x92, 0x00,0x94,0x92, 0x00,0x94,0x92, 0x00,0x94,0x92, 0x00,0x94,0x92, 0x00,0x94,0x92, 0x00,0x94,0x92, 0x00,0x94,0x72, 0x00,0x84,0x10, 0x00,0x73,0xAE,
  0x00,0x9C,0xF3, 0x00,0xA5,0x34, 0x00,0xAD,0x55, 0x00,0xB5,0x75, 0x00,0xB5,0x75, 0x00,0xB5,0x75, 0x00,0xB5,0x75, 0x00,0xB5,0x75, 0x00,0xB5,0x75, 0x00,0xB5,0x75, 0x00,0xB5,0x75, 0x00,0xB5,0x75, 0x00,0xB5,0x75, 0x00,0xB5,0x75, 0x00,0xB5,0x75, 
        0x00,0xB5,0x75, 0x00,0xB5,0x75, 0x00,0xB5,0x75, 0x00,0xB5,0x75, 0x00,0xB5,0x75, 0x00,0xB5,0x75, 0x00,0xB5,0x75, 0x00,0xAD,0x55, 0x00,0x94,0xB2, 0x00,0x7B,0xCF
};

GUI_CONST_STORAGE GUI_BITMAP bm08 = {
  25, // xSize
  25, // ySize
  75, // BytesPerLine
  24, // BitsPerPixel
  (unsigned char *)_ac08,  // Pointer to picture data
  NULL,  // Pointer to palette
  GUI_DRAW_BMPA565
};

/*************************** End of file ****************************/
