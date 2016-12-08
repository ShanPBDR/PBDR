#ifndef    __PRINT_H_
#define    __PRINT_H_
#include "stm32f4xx.h"
void print(char* fmt, ...);//use for debug information
void data_Output(char* fmt, ...);  //use for userdata output 
void printch(char ch);
void printdec(int dec);
void printflt(double flt);
void printbin(int bin);
void printhex(int hex);
void printstr(char* str);
extern void BSP_Uart2_WrByte(u8 c);
#define console_print(ch) BSP_Uart2_WrByte(ch)
//#define console_print(ch)    putchar(ch)

#endif    /*#ifndef __PRINT_H_*/
