/**
 * @filename:RCS_DataStructure.c
 * @brief:实现基本的数据结构
 * @changlog: 2024-2-8 CYK-Dot  初次创建
*/

/* =========头文件====================================*/
#include "RCS_DataStructure.h"

/* =========私有宏====================================*/
#define QUENE_EMPTY_SIGN 0
#define QUENE_FILLED_SIGN 1

/* =======函数实现====================================*/

/**
 * @name:Quene_Init
 * @brief:初始化环形队列
 * @param:RCS_QUENE* quene  指向队列的指针
 * @param:void* Quene_Data_Arr 存放队列内容的数组或者内存空间
 * @param:uint16_t Quene_Len 队列长度
 * @param:uint16_t Member_Size 成员长度
 * @tips:假设这是一个存放float的队列,那么Member_Size可以填Sizeof(float)
*/
void Quene_Init(RCS_QUENE* quene,void* Quene_Data_Arr,uint16_t Quene_Len,uint16_t Member_Size)
{
	quene->bottom=0;
	quene->top=0;
	quene->is_empty=QUENE_EMPTY_SIGN;
	quene->len=Quene_Len;
	quene->data=Quene_Data_Arr;
	quene->data_size=Member_Size;
}

/**
 * @name:Quene_Add_Member
 * @brief:为环形队列添加成员
 * @param:RCS_QUENE* quene  指向队列的指针
 * @param:void* member      指向成员的指针
 * @tips:记得先把指针类型强制转换为void*再送入这个函数
*/
void Quene_Add_Member(RCS_QUENE* quene,void* member)
{
	//环形入队
	memcpy(quene->data+(quene->top)*(quene->data_size),member,quene->data_size);
	//*(quene->data+(quene->top)*(quene->data_size))=*(member); 不能用
	
	//更新标志位
	quene->top++;
	quene->is_empty=QUENE_FILLED_SIGN;
	if (quene->top >= quene->len) quene->top=0;	
}

/**
 * @name:Quene_Quit_Member
 * @brief:从环形队列中取出一个成员
 * @param:RCS_QUENE* quene  指向队列的指针
 * @reval:void*             指向出队成员的指针
 * @tips:把返回的void*指针强制转换为需要的类型即可获取数据
*/
void* Quene_Quit_Member(RCS_QUENE* quene)
{
	static void* reval;

	if (quene->is_empty==QUENE_EMPTY_SIGN)//队头=队尾不意味着队空
	{
		reval=QUENE_MEMBER_EMPTY;//空指针
	}
	else
	{
		//出队
		reval=quene->data+(quene->bottom)*(quene->data_size);
		quene->bottom++;
		
		//更新标志位
		if (quene->bottom >= quene->len) quene->bottom=0;	
		if (quene->bottom == quene->top) quene->is_empty=QUENE_EMPTY_SIGN;//出队之后队头=队尾才是队空
	}
	
	return reval;
}

/**
 * @name:Quene_Clear
 * @brief:清空环形队列
 * @param:RCS_QUENE* quene  指向队列的指针
*/
void Quene_Clear(RCS_QUENE* quene)
{
	quene->bottom=0;
	quene->top=0;
	//quene->is_empty=QUENE_EMPTY_SIGN;
}



