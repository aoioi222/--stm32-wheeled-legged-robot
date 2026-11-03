/*@filename: Joystick_Ctrl.c
 *@author     胡兴国       
 *@brief:     手柄按键检测与控制
 *@date: 2023-7-27
*/
 
#include "rcs.h"

RCS_PIN_USART BLE_USART_MAP;
/*--------------全局变量-------------*/
int key_flag[10] = {0};       //十个功能按键（有线），六个功能按键（无线）
int stop_key = 0;             //急停键 （有线大红色第三个），右开关（无线）
int switch_key =1;            //自动/手动挡转换键（有线大红色第二个），左开关（无线）
int stick_key[2];							//遥感按键（0右，1左无线）
int direction_key[4]= {0};    //方向按键,0上，1下，2左，3右
static uint16_t key_data;			//按键数据
int rocker_lx,rocker_ly;  //左摇杆控制运动
int rocker_rx,rocker_ry;  //右遥感控制旋转
static int flag[10] = {0};		//参数判断
static float output1,output2,output3,output4;

int knob_l,knob_r;
	

/*--------------静态函数-------------*/
static void BlueTooth_Interrupt(void);
static __inline void RC_Remote_Judge(uint32_t raw_key);   //按键判定,手柄传出数据为所有按键值之和
/*----------------------------------*/

/**
	@name: BlueTooth_Init
	@brief: 无限手柄接受蓝牙初始化
**/
void BlueTooth_Init(RCS_PIN_USART USARTx_MAP)
{
	BLE_USART_MAP=USARTx_MAP;
	RCS_USART_Config(USARTx_MAP.USARTx, USARTx_MAP.GPIOx,USARTx_MAP.GPIO_Pin_Tx,USARTx_MAP.GPIO_Pin_Rx,BlueTooth_Interrupt ,BLUETOOTH_BAUD, BLUETOOTH_PRI);
	delay_ms(50);				
}



/**
	@name: BlueTooth_Interrupt
	@brief: 蓝牙接收中断初始化
**/
static void BlueTooth_Interrupt(void)
{
	static uint8_t receive_char[12];
	static uint8_t i=0;	 
	int16_t    receive_data[5];

	receive_char[i++]=(u8)USART_ReceiveData(BLE_USART_MAP.USARTx);//获取一个字节
	if(receive_char[0]!=0xCE)//不是包头
    {
		i=0;
		return;
	}
	if((i==12)&&(receive_char[11]==0xEC))//遇到包尾
	{
		i=0;
		receive_data[0] = ( ( (int16_t)(receive_char[1]))<<8) | (int16_t)receive_char[2];//高低八位组合形成按键信息
		receive_data[1] = ( ( (int16_t)(receive_char[3]))<<8) | (int16_t)receive_char[4];//高低八位组合形成摇杆信息1
		receive_data[2] = ( ( (int16_t)(receive_char[5]))<<8) | (int16_t)receive_char[6];//高低八位组合形成摇杆信息2
		receive_data[3] = ( ( (int16_t)(receive_char[7]))<<8) | (int16_t)receive_char[8];//高低八位组合形成左旋钮信息
		receive_data[4] = ( ( (int16_t)(receive_char[9]))<<8) | (int16_t)receive_char[10];//高低八位组合形成右旋钮信息		
		//---------------------左侧方向键------------------------------
    if ((receive_data[0]&UP_KEY)==UP_KEY)      direction_key[0]=1; 
		else                                       direction_key[0]=0; 
		if ((receive_data[0]&DOWN_KEY)==DOWN_KEY)  direction_key[1]=1; 
		else                                       direction_key[1]=0;
		if ((receive_data[0]&LEFT_KEY)==LEFT_KEY)  direction_key[2]=1; 
		else                                       direction_key[2]=0;
		if ((receive_data[0]&RIGHT_KEY)==RIGHT_KEY) direction_key[3]=1; 
		else                                        direction_key[3]=0;
		//--------------------右侧功能键-------------------------------	
		if ((receive_data[0]&KEY1)==KEY1)        key_flag[0]=1; 
		else                                     key_flag[0]=0;
		if ((receive_data[0]&KEY2)==KEY2)        key_flag[1]=1; 
		else                                     key_flag[1]=0;
		if ((receive_data[0]&KEY3)==KEY3)        key_flag[2]=1; 
		else                                     key_flag[2]=0;
		if ((receive_data[0]&KEY4)==KEY4)        key_flag[3]=1; 
		else                                     key_flag[3]=0;
		if ((receive_data[0]&KEY5)==KEY5)        key_flag[4]=1; 
		else                                     key_flag[4]=0;
		if ((receive_data[0]&KEY6)==KEY6)        key_flag[5]=1; 
		else                                     key_flag[5]=0;
		//-------------------急停与模式切换----------------------------
		if ((receive_data[0]&LSWITCH_KEY)==LSWITCH_KEY)     switch_key=1; 
		else                                                switch_key=0;
		if ((receive_data[0]&RSWITCH_KEY)==RSWITCH_KEY)     stop_key=1; 
		else                                                stop_key=0;
		//-------------------没啥用的摇杆按键--------------------------
		if ((receive_data[0]&LSTICK_KEY)==LSTICK_KEY)       key_flag[6]=1; 
		else                                                key_flag[6]=0;
		if ((receive_data[0]&RSTICK_KEY)==RSTICK_KEY)       key_flag[7]=1; 
		else                                                key_flag[7]=0;
		//--------------------ADC数据----------------------------------
		rocker_lx=(receive_data[1]&0xff00)>>8;        //Lx
		rocker_ly=receive_data[1]&0x00ff;             //Ly
		rocker_rx=(receive_data[2]&0xff00)>>8;        //Rx
		rocker_ry=receive_data[2]&0x00ff;             //Ry
		knob_l = receive_data[3];
		knob_r = receive_data[4];
	}
	if(i>12)i=0;
	USART_ClearITPendingBit(BLE_USART_MAP.USARTx, USART_IT_RXNE);
}


/**
	@name: RC_Remote_Judge
	@brief: RC无限手柄键位判定
	@param: 键位值
**/
static __inline void RC_Remote_Judge(uint32_t raw_key)   //按键判定,手柄传出数据为所有按键值之和
{
    //---------------------左侧方向键------------------------------
    if ((raw_key&UP_KEY)==UP_KEY)    direction_key[0]=1; 
	else                             direction_key[0]=0; 
	if ((raw_key&UP_KEY)==DOWN_KEY)  direction_key[1]=1; 
	else                             direction_key[1]=0;
	if ((raw_key&UP_KEY)==LEFT_KEY)  direction_key[2]=1; 
	else                             direction_key[2]=0;
	if ((raw_key&UP_KEY)==RIGHT_KEY) direction_key[3]=1; 
	else                             direction_key[3]=0;
	//--------------------右侧功能键-------------------------------	
	if ((raw_key&KEY1)==KEY1)        key_flag[0]=1; 
	else                             key_flag[0]=0;
	if ((raw_key&KEY2)==KEY2)        key_flag[1]=1; 
	else                             key_flag[1]=0;
	if ((raw_key&KEY3)==KEY3)        key_flag[2]=1; 
	else                             key_flag[2]=0;
	if ((raw_key&KEY4)==KEY4)        key_flag[3]=1; 
	else                             key_flag[3]=0;
	if ((raw_key&KEY5)==KEY5)        key_flag[4]=1; 
	else                             key_flag[4]=0;
	if ((raw_key&KEY6)==KEY1)        key_flag[5]=1; 
	else                             key_flag[5]=0;
	//-------------------急停与模式切换----------------------------
	if ((raw_key&LSWITCH_KEY)==LSWITCH_KEY)  switch_key=1; 
	else                                     switch_key=0;
	if ((raw_key&RSWITCH_KEY)==RSWITCH_KEY)  stop_key=1; 
	else                                     stop_key=0;
	//-------------------没啥用的摇杆按键--------------------------
	if ((raw_key&LSTICK_KEY)==LSTICK_KEY)    key_flag[6]=1; 
	else                                     key_flag[6]=0;
	if ((raw_key&RSTICK_KEY)==RSTICK_KEY)    key_flag[7]=1; 
	else                                     key_flag[7]=0;
}