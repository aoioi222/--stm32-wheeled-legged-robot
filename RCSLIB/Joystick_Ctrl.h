/*@filename: Joystick_Ctrl.h
 *@author èƒ¡å…´å›?
 *@brief:  æ‰‹æŸ„æŽ§åˆ¶
 *@date:2023-7-27
*/
#ifndef _JOYSTICK_CTRL_H_
#define _JOYSTICK_CTRL_H_

#include "rcs.h"

#define BLUETOOTH_BAUD 9600
   
#define INIT_KEY                128             //æ‰‹æŸ„åˆå§‹è¿”å›ž128
#define KEY_ON                  1               //æŒ‰é”®æŒ‰ä¸‹
#define KEY_OFF                 0               //æŒ‰é”®æ¾å¼€
#define AMPLIFY                 10              //æ”¾å¤§ä¿¡å·
//æ— é™æ‰‹æŸ„æŒ‰é”®å¯¹ç…§è¡?
#define UP_KEY									0x0001					//æ–¹å‘é”?
#define DOWN_KEY								0x0002
#define LEFT_KEY								0x0004
#define RIGHT_KEY								0x0008
#define KEY1									0x0010					//æ™®é€šæŒ‰é”?
#define KEY2									0x0020
#define KEY3									0x0040
#define KEY4									0x0080
#define KEY5									0x0100
#define KEY6									0x0200
#define LSWITCH_KEY							    0x0400					//å·¦å³è‡ªé”å¼€å…?
#define RSWITCH_KEY							    0x0800	
#define LSTICK_KEY							    0x1000					//å·¦å³é¥æ„ŸæŒ‰é”®
#define RSTICK_KEY							    0x2000



void BlueTooth_Init(RCS_PIN_USART USARTx_MAP);
void Joystick_Ctrl(void);    //ç®€å•é¥æŽ§æŽ§åˆ¶byå°èƒ¡
float Get_Current2(void);
void Joystick_Absolute_Ctrl(void);
void Joystick_Helm_Ctrl(void); //èˆµè½®é¥æŽ§
void Joystick_Direction_Ctrl(void);
void Joystick_Rocker_Ctrl(void);
void Joystick_Direction_Ctrl_Slow(void);

extern int key_flag[10];       //Ê®¸ö¹¦ÄÜ°´¼ü£¨ÓÐÏß£©£¬Áù¸ö¹¦ÄÜ°´¼ü£¨ÎÞÏß£©
extern int stop_key;             //¼±Í£¼ü £¨ÓÐÏß´óºìÉ«µÚÈý¸ö£©£¬ÓÒ¿ª¹Ø£¨ÎÞÏß£©
extern int switch_key;            //×Ô¶¯/ÊÖ¶¯µ²×ª»»¼ü£¨ÓÐÏß´óºìÉ«µÚ¶þ¸ö£©£¬×ó¿ª¹Ø£¨ÎÞÏß£©
extern int stick_key[2];							//Ò£¸Ð°´¼ü£¨0ÓÒ£¬1×óÎÞÏß£©
extern int direction_key[4];    //·½Ïò°´¼ü,0ÉÏ£¬1ÏÂ£¬2×ó£¬3ÓÒ
extern int rocker_lx,rocker_ly;  //×óÒ¡¸Ë¿ØÖÆÔË¶¯
extern int rocker_rx,rocker_ry;  //ÓÒÒ£¸Ð¿ØÖÆÐý×ª

#endif
