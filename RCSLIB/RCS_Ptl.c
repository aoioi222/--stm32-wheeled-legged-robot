/**
 * @filename:RCS_Ptl.c
 * @brief:实现多种常用软件通信协议
 * @layers:ISO数据链路层
 * @contribute:CYK-Dot @2024-4-6 实现简单定长头尾协议
 * ----------------------------------------------
 * @changelog:CYK-Dot @2024-5-14 实现两种变长头尾协议(带确认/不带确认),并统一链路层的函数格式
*/
#include "RCS_Ptl.h"

/**
 * @name:RCS_Rcv_EasyProtocol
 * @brief:简单定长头尾协议接收
 * @param:input_byte  一次输入一个字节
 * @param:temp_arr    存放中间结果的数组
 * @param:output_flag 存放协议状态机状态的变量指针
 * @param:output_arr  存放最终解包的数组
 * @param:protocol    使用的协议指针
*/
void Protocol_Rcv_Easy(uint8_t input_byte,  uint8_t* temp_arr,
                        uint8_t* output_flag,uint8_t* output_arr,
                        RCS_Easy_Ptl* protocol)
{
    //--------局部变量声明------------------------------
    static volatile int rcv_number=0;
    volatile int i;
    //--------协议状态机--------------------------------
	switch(*output_flag)
	{
		//________等待包头_______________________________
		case 0:
			if (input_byte==protocol->Start_Byte)
			{
				memset(temp_arr,0,sizeof(temp_arr));
				rcv_number=0;
				*output_flag=1;
			}
		break;
		//________获取数据_______________________________
		case 1:
			temp_arr[rcv_number]=input_byte;
			rcv_number=rcv_number+1;

			if ((rcv_number==(protocol->Len+1))&&(input_byte==protocol->End_Byte))
			{
				//输出
				for(i=0;i<protocol->Len;i++)
				{
					output_arr[i]=temp_arr[i];
				}
				//callback
				protocol->Finish_Callback(output_arr);
				
				//复位状态机
				*output_flag=0;
				rcv_number=0;
			}

			if (rcv_number>(protocol->Len+1))
			{
				//复位状态机
				*output_flag=0;
				rcv_number=0;
			}
		break;
	}
}
void Receive_visiondata (uint8_t input_byte,  uint8_t* temp_arr,
                        uint8_t* output_flag,uint8_t* output_arr,
                        RCS_Easy_Ptl2* protocol)
{
    //--------局部变量声明------------------------------
    static volatile int rcv_number=0;
    volatile int i;
    //--------协议状态机--------------------------------
	switch(*output_flag)
	{
		//________等待包头_______________________________
		case 0:
			if (input_byte == protocol->Start_Byte1) 
				{
                temp_arr[rcv_number++] = input_byte;
                *output_flag = 1;
        }
		break;
		case 1:
			if (input_byte == protocol->Start_Byte2) 
				{  
                temp_arr[rcv_number++] = input_byte;
                *output_flag = 2;
        } 
			else 
				{
                // 如果接收到的不是第二个包头，重置状态机
                *output_flag = 0;
                rcv_number = 0;
        }
		break;
		//________获取数据_______________________________
		case 2:
            temp_arr[rcv_number++] = input_byte;
   
            if (rcv_number == protocol->Len+3 && input_byte == protocol->End_Byte) 
						{ 
                for (i = 0; i < protocol->Len; i++) 
							{
                    output_arr[i] = temp_arr[i + 2];  // 跳过包头
              }
                protocol->Finish_Callback(output_arr);

                // 复位状态机
                *output_flag =0;
                rcv_number = 0;
            } 
						else if (rcv_number > protocol->Len + 2) 
						
						{
                // 超出数据长度，重置状态机
                *output_flag = 0;
                rcv_number = 0;
            }
            break;
	}
}

/**
 * @name:RCS_Get_EasyProtocol
 * @brief:将数据打包成简单定长头尾协议
*/
void Protocol_Get_Easy(uint8_t* input_arr,uint8_t* output_arr,RCS_Easy_Ptl* protocol)
{
    int i;
    output_arr[0]=protocol->Start_Byte;
    output_arr[protocol->Len+1]=protocol->End_Byte;
    for(i=0;i<protocol->Len;i++) output_arr[i+1]=input_arr[i];
}

/**
 * @name:Protocol_Init_LenVerify
 * @brief:初始化一个长度校验协议
**/
void Protocol_Init_LenVerify(RCS_LenVerify_Ptl* ptl,uint8_t start_byte,uint8_t len,uint8_t* buffer_arr)
{
    ptl->Start_Byte=start_byte;
    ptl->Len=len;
    ptl->RcvBufr_Arr=buffer_arr;
    ptl->RcvBufr_Pnt=0;
    ptl->RcvStat_Fsm=0;
}

/**
 * @name:Protocol_Rcv_LenVerify_IT
 * @brief:将字节流逐个传入该函数,实现长度校验协议的解析
 * @param:ptl_handler_void  协议句柄的指针,对于长度校验协议,数据类型为RCS_LenVerify_Ptl,将该指针转换为void*送入该函数即可
 * @param:input_byte        接收到的字节
 * @param:output_arr        解析后的报文将会更新到该数组中。解析失败不会将该数组中的内容清0,而是保持不变
 * @reval:uint8_t           返回当前的接收情况,比如接收是否完成。详见RCS_Ptl.h中RCS_Common_Ptl_Log的定义
 * @tips:在传入ptl_handler_void之前,需要先对其做初始化
**/
uint8_t Protocol_Rcv_LenVerify_IT(void* ptl_handler_void,uint8_t input_byte,uint8_t* output_arr)
{
    RCS_LenVerify_Ptl* ptl=(RCS_LenVerify_Ptl*)ptl_handler_void;

    switch(ptl->RcvStat_Fsm)
    {
        //---------------------等待包头-------------------------------------
        //①包头字节出现，开始接收数据
        //②包头字节未出现，则不做任何理会
        case 0:
            if (input_byte==ptl->Start_Byte)
            {
                ptl->RcvStat_Fsm=1;            //转到下一个状态
                ptl->RcvBufr_Pnt=0;            //从0开始存放数据
                ptl->RcvBufr_Arr[1+ptl->Len]=ptl->Start_Byte;//第Len+1个数据用于存放校验和
                return RCV_IDLE;
            }
            break;

        //----------------接收数据+实时累加校验-----------------------------
        //①当接收的字节数达到预期，而且校验正确，则拷贝数据，并重新开始。
        //②当接收的字节数达到预期，但是校验不正确，则重新开始。
        //③接收的字节数未达预期，则继续接收。
        case 1:
            ptl->RcvBufr_Arr[ptl->RcvBufr_Pnt]=input_byte;
            ptl->RcvBufr_Arr[1+ptl->Len]+=input_byte;
            ptl->RcvBufr_Pnt++;

            if (ptl->RcvBufr_Pnt == 1+ptl->Len)
            {
                //情况①
                if (ptl->RcvBufr_Arr[1+ptl->Len] == (uint8_t)(ptl->RcvBufr_Arr[ptl->Len]+ptl->RcvBufr_Arr[ptl->Len]))
                {
                    memcpy(output_arr,ptl->RcvBufr_Arr,ptl->Len);
                    ptl->RcvStat_Fsm=0;
                    return RCV_CPLT;

                }
                //情况②
                else
                {
                    ptl->RcvStat_Fsm=0;
                    return RCV_FAIL;
                }
            }
            else
            {
                //情况③
                return RCV_PROC;
            }
            break;
    }
}

/**
 * @name:Protocol_Get_LenVerify
 * @brief:将待发送的数组按照长度校验协议打包
 * @param:ptl_handler_void  协议句柄的指针,对于长度校验协议,数据类型为RCS_LenVerify_Ptl,将该指针转换为void*送入该函数即可
 * @param:output_arr        打包好的数据将会更新到该数组中
 * @param:input_arr         需要打包的数据
 * @param:len               需要打包的数据的长度，对于长度校验协议，这项参数是无效的，可以不填
 * @tips:在传入ptl_handler_void之前,需要先对其做初始化
 * @reval:包装好的字节总数
*/
uint8_t Protocol_Get_LenVerify(void* ptl_handler_void,uint8_t* output_arr,uint8_t* input_arr,uint8_t len)
{
    RCS_LenVerify_Ptl* ptl=(RCS_LenVerify_Ptl*)ptl_handler_void;
    uint16_t i;

    output_arr[0]=ptl->Start_Byte;
    output_arr[ptl->Len+1]=ptl->Start_Byte;
    for(i=0;i<ptl->Len;i++)
    {
        output_arr[1+i]=input_arr[i];
        output_arr[ptl->Len+1]+=input_arr[i];
    }
    return ptl->Len+2;
}



/**
 * @name:Protocol_Rcv_RCSBleNet_IT
 * @brief:将字节流逐个传入该函数,实现变长头尾协议的解析
 * @param:ptl_handler_void  协议句柄的指针,对于变长头尾协议,数据类型为RCS_Comm_Ptl_Handler,将该指针转换为void*送入该函数即可
 * @param:input_byte        接收到的字节
 * @param:output_arr        解析后的报文将会更新到该数组中。解析失败不会将该数组中的内容清0,而是保持不变
 * @reval:uint8_t           返回当前的接收情况,比如接收是否完成。详见RCS_Ptl.h中RCS_Common_Ptl_Log的定义
 * @tips:在传入ptl_handler_void之前,需要先对其做初始化
*/
uint8_t Protocol_Rcv_RCSBleNet_IT(void* ptl_handler_void,uint8_t input_byte,uint8_t* output_arr)
{
	//局部变量
	uint32_t total_crc;
	uint8_t i;
	//变量具象化
	RCS_Comm_Ptl_Handler*  ptl_handler_p=(RCS_Comm_Ptl_Handler*)ptl_handler_void;
	RCS_Common_Ptl*        ptl_param=&(ptl_handler_p->PTL_Param);
	RCS_Common_Ptl_Buffer* ptl_buf=&(ptl_handler_p->PTL_Rcv_Buffer);
	RCS_Common_Ptl_Log*    ptl_log=&(ptl_handler_p->PTL_Log);

	//状态机
	switch(ptl_buf->status_flag)
	{
		//-------------------等待包头--------------------------------
		case 0:
			if (input_byte==ptl_param->Start_Byte)
			{
				//数据复位
				ptl_buf->rcv_pointer=0;
				ptl_buf->crc_h=0;
				ptl_buf->crc_l=0;
				memset(ptl_buf->rcv_buffer,0,sizeof(ptl_buf->rcv_buffer));
				//接收到的字节和包头一致，则跳转
				ptl_buf->status_flag=2;
			}
			//未完成接收
			*ptl_log=RCV_PROC;
			return RCV_PROC;
            break;

		//-----------------接收帧类型---------------------------------
		case 2:
			//数据接收(buffer[0]=帧类型)
			ptl_buf->rcv_buffer[ptl_buf->rcv_pointer]=input_byte;
			ptl_buf->rcv_pkg_type=input_byte;
			ptl_buf->rcv_pointer++;
			//直接跳转
			ptl_buf->status_flag=3;
			//未完成接收
			*ptl_log=RCV_PROC;
			return RCV_PROC;
            break;

		//------------------接收报文长度--------------------------------
		case 3:
			//数据接收(buffer[1]=报文长度)
			ptl_buf->rcv_buffer[ptl_buf->rcv_pointer]=input_byte;
			ptl_buf->rcv_pkg_len=input_byte;
			ptl_buf->rcv_pointer++;
			//直接进入下一个case
			ptl_buf->status_flag=4;
			//未完成接收
			*ptl_log=RCV_PROC;
			return RCV_PROC;
		break;

		//-------------------接收数据-----------------------------------
		case 4:
			//数据接收(buffer[2~2+N]=数据)
			ptl_buf->rcv_buffer[ptl_buf->rcv_pointer]=input_byte;
			ptl_buf->rcv_pointer++;

			//接收到的字节数到达指定数量。根据校验方式的不同决定跳转的case，准备接收校验数据
			if (ptl_buf->rcv_pointer == (2 + ptl_buf->rcv_pkg_len) )
			{
				if (ptl_buf->rcv_pkg_type==Pkg_Type_Verify)
					ptl_buf->status_flag=10;
				else if (ptl_buf->rcv_pkg_type==Pkg_Type_Direct)
					ptl_buf->status_flag=20;
				else if (ptl_buf->rcv_pkg_type==Pkg_Type_FullVerify)
					ptl_buf->status_flag=30;//todo
			}

			//少收到一个字节,本次接收失败,强行重新开始接收
			if ((ptl_buf->rcv_pkg_type==Pkg_Type_Direct)||(ptl_buf->rcv_pkg_type==Pkg_Type_FullVerify))
			{
				//连续收到结束字节+开始字节,基本上能排除报文误识别为包尾的情况
				if ((ptl_buf->rcv_buffer[ptl_buf->rcv_pointer-1]==ptl_param->Start_Byte)&&(ptl_buf->rcv_buffer[ptl_buf->rcv_pointer-2]==ptl_param->End_Byte))
				{
					//数据复位
					ptl_buf->rcv_pointer=0;
					ptl_buf->crc_h=0;
					ptl_buf->crc_l=0;
					memset(ptl_buf->rcv_buffer,0,sizeof(ptl_buf->rcv_buffer));
					//重新开始接收
					ptl_buf->status_flag=2;
					//接收失败
					*ptl_log=RCV_FAIL;
					return RCV_FAIL;
				}
			}

			//未完成接收
			*ptl_log=RCV_PROC;
			return RCV_PROC;
		break;

		//-------------------基本校验-----------------------------------------------
		case 10:
			//数据接收(buffer[3+N]=高八位校验值)
			ptl_buf->rcv_buffer[ptl_buf->rcv_pointer]=input_byte;
			ptl_buf->rcv_pointer++;
			//进入下一个case
			ptl_buf->status_flag=11;
			//未完成接收
			*ptl_log=RCV_PROC;
			return RCV_PROC;
		break;
		case 11:
			//数据接收(buffer[4+N]=低八位校验值)
			ptl_buf->rcv_buffer[ptl_buf->rcv_pointer]=input_byte;
			ptl_buf->rcv_pointer++;
			//计算校验和
			total_crc=ptl_param->Start_Byte;
			for(i=1;i<=1+ptl_buf->rcv_pkg_len;i++) total_crc+=ptl_buf->rcv_buffer[i];
			ptl_buf->crc_h=(total_crc&0xff00)>>8;
			ptl_buf->crc_l= total_crc&0xff;
			//进入下一个case
			if ((ptl_buf->crc_h ==ptl_buf->rcv_buffer[2+ptl_buf->rcv_pkg_len])
			  &&(ptl_buf->crc_l ==ptl_buf->rcv_buffer[3+ptl_buf->rcv_pkg_len]))
			{
				for(i=0;i<ptl_buf->rcv_pkg_len;i++) output_arr[i]=ptl_buf->rcv_buffer[i+2];
				ptl_buf->status_flag=0;
				*ptl_log=RCV_CPLT;
				return RCV_CPLT;
			}
			else
			{
				ptl_buf->status_flag=0;
				*ptl_log=RCV_FAIL;
				return RCV_FAIL;
			}
		break;

		//------------------直通校验--------------------------------------
		case 20:
			//数据接收(buffer[3+N]=包尾)
			ptl_buf->rcv_buffer[ptl_buf->rcv_pointer]=input_byte;
			ptl_buf->rcv_pointer++;
			//校验
			if (input_byte==ptl_param->End_Byte)
			{
				for(i=0;i<ptl_buf->rcv_pkg_len;i++) output_arr[i]=ptl_buf->rcv_buffer[i+2];
				ptl_buf->status_flag=0;
				*ptl_log=RCV_CPLT;
				return RCV_CPLT;
			}
			else
			{
				ptl_buf->status_flag=0;
				*ptl_log=RCV_FAIL;
				return RCV_FAIL;
			}
		break;
		//------------------完整校验--------------------------------------
		case 30:
			//数据接收(buffer[3+N]=高八位校验值)
			ptl_buf->rcv_buffer[ptl_buf->rcv_pointer]=input_byte;
			ptl_buf->rcv_pointer++;
			//进入下一个case
			ptl_buf->status_flag=31;
			//未完成接收
			*ptl_log=RCV_PROC;
			return RCV_PROC;
		break;
		case 31:
			//数据接收(buffer[4+N]=低八位校验值)
			ptl_buf->rcv_buffer[ptl_buf->rcv_pointer]=input_byte;
			ptl_buf->rcv_pointer++;
			//计算校验和
			total_crc=ptl_param->Start_Byte;
			for(i=0;i<=1+ptl_buf->rcv_pkg_len;i++) total_crc+=ptl_buf->rcv_buffer[i];
			ptl_buf->crc_h=(total_crc&0xff00)>>8;
			ptl_buf->crc_l= total_crc&0xff;
			//进入下一个case
			if ((ptl_buf->crc_h ==ptl_buf->rcv_buffer[2+ptl_buf->rcv_pkg_len])
			  &&(ptl_buf->crc_l ==ptl_buf->rcv_buffer[3+ptl_buf->rcv_pkg_len]))
			{
				for(i=0;i<ptl_buf->rcv_pkg_len;i++) output_arr[i]=ptl_buf->rcv_buffer[i+2];
				ptl_buf->status_flag=32;
				*ptl_log=RCV_PROC;
				return RCV_PROC;
			}
			else
			{
				ptl_buf->status_flag=0;
				*ptl_log=RCV_FAIL;
				return RCV_FAIL;
			}
		break;
		case 32:
			//数据接收(buffer[3+N]=包尾)
			ptl_buf->rcv_buffer[ptl_buf->rcv_pointer]=input_byte;
			ptl_buf->rcv_pointer++;
			//校验
			if (input_byte==ptl_param->End_Byte)
			{
				for(i=0;i<ptl_buf->rcv_pkg_len;i++) output_arr[i]=ptl_buf->rcv_buffer[i+2];
				ptl_buf->status_flag=0;
				*ptl_log=RCV_CPLT;
				return RCV_CPLT;
			}
			else
			{
				ptl_buf->status_flag=0;
				*ptl_log=RCV_FAIL;
				return RCV_FAIL;
			}
		break;

		//------------------错误状态--------------------------------------
		default:
			*ptl_log=RCV_ERR;
			return RCV_ERR;
		break;
	}
}
/**
 * @name:Protocol_Get_RCSBleNet
 * @brief:将待发送的数组按照变长头尾协议打包
 * @param:ptl_handler_void  协议句柄的指针,对于变长头尾协议,数据类型为RCS_Comm_Ptl_Handler,将该指针转换为void*送入该函数即可
 * @param:output_arr        打包好的数据将会更新到该数组中
 * @param:input_arr         需要打包的数据
 * @param:len               需要打包的数据的长度
 * @usage:变长头尾协议有三种帧格式,但这三种格式对有用的数据来说是透明的,选哪种取决于实际需求,比如需要准确还是需要快速
 * @tips:在传入ptl_handler_void之前,需要先对其做初始化
 * @reval:包装好的字节总数
*/
uint8_t Protocol_Get_RCSBleNet(void* ptl_handler_void,uint8_t* output_arr,uint8_t* input_arr,uint8_t len)
{
	//局部变量
	int i;
	uint32_t crc_total=0;
	//一般指针具象化
	RCS_Common_Ptl* ptl_param=(RCS_Common_Ptl*)ptl_handler_void;
	//报文生成
	switch(ptl_param->Pkg_Type)
	{
		case Pkg_Type_Direct:
			output_arr[0]=ptl_param->Start_Byte;
			output_arr[1]=Pkg_Type_Direct;
			output_arr[2]=len;
			for(i=0;i<len;i++) output_arr[3+i]=input_arr[i];
			output_arr[3+len]=ptl_param->End_Byte;
			return 4+len;
		break;

		case Pkg_Type_Verify:
			output_arr[0]=ptl_param->Start_Byte; crc_total+=output_arr[0];
			output_arr[1]=Pkg_Type_Verify; crc_total+=output_arr[1];
			output_arr[2]=len;             crc_total+=output_arr[2];
			for(i=0;i<len;i++)
			{
				output_arr[3+i]=input_arr[i];
				crc_total+=input_arr[i];
			}
			output_arr[3+len]=(crc_total & 0xff00) >>8;
			output_arr[4+len]= crc_total & 0xff;
			return 5+len;
		break;

		case Pkg_Type_FullVerify:
			output_arr[0]=ptl_param->Start_Byte;crc_total+=output_arr[0];
			output_arr[1]=Pkg_Type_FullVerify;  crc_total+=output_arr[1];
			output_arr[2]=len;                  crc_total+=output_arr[2];
			for(i=0;i<len;i++)
			{
				output_arr[3+i]=input_arr[i];
				crc_total+=input_arr[i];
			}
			output_arr[3+len]=(crc_total & 0xff00) >>8;
			output_arr[4+len]= crc_total & 0xff;
			output_arr[5+len]= ptl_param->End_Byte;
			return 6+len;
		break;
	}
}

/**
 * @name:Protocol_Init_RCSBleNet
 * @brief:初始化一个变长头尾协议(蓝牙串口环网协议)
 * @param:ptl         协议句柄的地址
 * @param:start_byte  帧的起始字节,需要根据数据的分布情况,选择一个最不可能干扰数据的值
 * @param:end_byte    帧的结束字节,需要根据数据的分布情况,选择一个最不可能干扰数据的值
 * @param:pkg_type    帧的包装方式,详见@ref:RCS_Common_Ptl_PkgType
 * @param:buffer_arr  为了协议的接收,需要占用一部分内存空间,这部分空间需要手动指定,该数组的大小不应小于有效数据的最小长度+6
 * @reval:是否成功完成初始化
*/
uint8_t Protocol_Init_RCSBleNet(RCS_Comm_Ptl_Handler* ptl_Handler_p,
                                uint8_t start_byte,uint8_t end_byte,RCS_Common_Ptl_PkgType pkg_type,
								                uint8_t* buffer_arr)
{
	ptl_Handler_p->PTL_Param.Start_Byte=start_byte;
	ptl_Handler_p->PTL_Param.End_Byte=end_byte;
	ptl_Handler_p->PTL_Param.Pkg_Type=pkg_type;

	ptl_Handler_p->PTL_Rcv_Buffer.rcv_buffer=buffer_arr;
	ptl_Handler_p->PTL_Rcv_Buffer.crc_h=0;
	ptl_Handler_p->PTL_Rcv_Buffer.crc_l=0;
	ptl_Handler_p->PTL_Rcv_Buffer.rcv_pkg_len=0;
	ptl_Handler_p->PTL_Rcv_Buffer.rcv_pkg_type=0;
	ptl_Handler_p->PTL_Rcv_Buffer.rcv_pointer=0;
	ptl_Handler_p->PTL_Rcv_Buffer.status_flag=0;

	ptl_Handler_p->PTL_Log=RCV_IDLE;

	return 1;
}