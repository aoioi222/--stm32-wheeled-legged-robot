/**
* @file         
* @brief		为RCSLIB提供C++支持,包括常用的STD容器、内存分配等
* @author		CYK-Dot
* @date		    2024-12-13
* @version	    v1.0
* @par 使用方式
*               在需要使用C++时,请勿引用std命名空间,而应使用RCS_CXX命名空间,以避免裸机操作单片机堆区 
*/

#ifndef RCS_CXX_h
#define RCS_CXX_h

#include "stdint.h"
#include "FreeRTOS.h"
#include <unordered_map>
#include <vector>
#include <deque>
#include <stack>
#include <array>
#include <queue>

/**
*@namespace RCS_CXX
*@brief     RCSLIB使用的C++命名空间。在该命名空间下,使用FreeRTOS的内存管理接口替代了C标准库的内存管理接口,并部署了一些常用的C++容器
*@defgroup  RCS_CXX_Group RCS_CXX 命名空间
*/
namespace RCS_CXX
{
	/**
	* @funtion       
	* @brief         将某个二进制整数的某一位置1
	* @details       这是一个模板函数，需要用 <_T> 指定数据类型
	* @tparam        _T 指定的数据类型，可以是 uint8_t 等整型
	* @param[in]     value 原先的二进制整数
	* @param[in]     position 第几个二进制位需要被置1，从0开始
	* @return        置1后的值
	*/
	template<typename _T> constexpr _T set_bit(_T value, uint8_t position) 
	{
		return value | ((_T)1U << position);  
	}

	/**
	* @funtion        
	* @brief		 将某个二进制整数的某一位置0
	* @details	     这是一个模板函数,需要用<_T>指定数据类型
	* @tparam        _T 指定的数据类型,可以是uint8_t等整型
	* @param[in]	 value 原先的二进制整数
	* @param[out]	 position 第几个二进制位需要被置0,从0开始
	* @retval[out]   _T 置0后的值
	*/   
	template<typename _T> constexpr _T reset_bit(_T value,uint8_t position)
	{
		return value & ~((_T)1U << position); 
	}

	/**
	* @funtion          
	* @brief		 判断二进制整数的某一位置是否为1
	* @details	     这是一个模板函数,需要用<_T>指定数据类型
	* @tparam        _T 指定的数据类型,可以是uint8_t等整型
	* @param[in]	 value 原先的二进制整数
	* @param[out]	 position 第几个二进制位需要被判断,从0开始
	* @retval[out]   bool 
	*/   
	template<typename _T> constexpr bool is_bit(_T value,uint8_t position)
	{
		return (value & (static_cast<_T>(1) << position)) != 0;
	}
	
	/**
	*@class  RCS_allocator
	*@tparam _T 分配基本单元的数据类型
	*@brief  使用RTOS接口而非C标准Malloc接口的内存分配器,为C++STL容器提供FreeRTOS支持
	**/
	template <typename T> class RCS_allocator
	{		
		public:
		using value_type = T;                   ///此分配器功能仅为兼容std::allocator
        using pointer = T*;                     ///此分配器功能仅为兼容std::allocator
        using const_pointer = const T*;         ///此分配器功能仅为兼容std::allocator
        using size_type = std::size_t;          ///此分配器功能仅为兼容std::allocator
        using difference_type = std::ptrdiff_t; ///此分配器功能仅为兼容std::allocator
		
		template <typename U> struct rebind     ///此结构体功能仅为兼容std::allocator
        {
            using other = RCS_allocator<U>;
        };
		
		public:
			
		///RCS内存分配器的构造函数为空,因为构造函数被调用时RTOS并未启动,不应调用RTOS的接口
		RCS_allocator()=default;                
		template <typename U> RCS_allocator(const RCS_allocator<U>&) {}
			
		/**
		* @function
		* @brief         从RTOS管理的堆中申请一段内存
		* @tparam        T 指定的数据类型
		* @param[in]     n 申请多少个指定的数据
		* @retval        T* 申请到的空间的指针,没申请到会返回null
		*/
		T* allocate(std::size_t n)
		{
			return (T*)pvPortMalloc(n*sizeof(T));
		}
		
		/**
		* @function
		* @brief         释放一段申请过的内存
		* @tparam        T 指定的数据类型
		* @param[in]     p 指向申请到的内存的指针
		* @param[in]     n 多少个指定的数据
		* @retval        void
		*/
		void deallocate(T* p, std::size_t n)
		{
			vPortFree((void*)p);
		}
		
		///分配器运算符重载
		template <typename U> bool operator==(const RCS_allocator<U>&) const { return true; }
		///分配器运算符重载
        template <typename U> bool operator!=(const RCS_allocator<U>&) const { return false;}
	};
	
	/** 
	*@class  unordered_map
	*@tparam _KEY 哈希表键类型
	*@tparam _VALUE 哈希表值类型
	*@brief  使用RTOS接口替代std接口的哈希表容器
	*/
	template <typename _KEY,typename _VALUE> class unordered_map : public std::unordered_map<_KEY,_VALUE,std::hash<_KEY>,std::equal_to<_KEY>,RCS_CXX::RCS_allocator<std::pair<const _KEY,_VALUE>>>{};
	/**
	*@namespace RCS_CXX  
	*@class  RCS_CXX::vector
	*@brief  使用RTOS接口替代std接口的向量容器
	*@tparam _T 容器数据类型
	*/
	template <typename _T>                   class vector        : public std::vector<_T,RCS_CXX::RCS_allocator<_T>>{};
	/**
	*@namespace RCS_CXX 
	*@class  RCS_CXX::deque
	*@brief  使用RTOS接口替代std接口的双向队列容器
	*@tparam _T 容器数据类型
	*/
	template <typename _T>                   class deque         :public std::deque<_T,RCS_CXX::RCS_allocator<_T>>{};
	/**
	*@namespace RCS_CXX 
	*@class  RCS_CXX::stack
	*@brief  使用RTOS接口替代std接口的栈容器
	*@tparam _T 容器数据类型
	*/
	template <typename _T>                   class stack         :public std::stack<_T,RCS_CXX::deque<_T>>{};
	/**
	*@namespace RCS_CXX 
	*@class  RCS_CXX::queue
	*@brief  使用RTOS接口替代std接口的队列容器
	*@tparam _T 容器数据类型
	*/
	template <typename _T>                   class queue         :public std::queue<_T,RCS_CXX::deque<_T>>{};

	/**
	*@namespace RCS_CXX 
	*@class   vbuffer
	*@tparam  _T 缓冲区内数据的数据类型
	*@brief   RCSers编写的缓冲区必须继承并重写该类中所有的接口
	*/
	template <typename _T> class vbuffer
	{	
		public:
		_T& operator[](size_t pos);
		///获取队头元素的引用
		_T& front(void);
		///获取队尾元素的引用
		_T& back(void);
		///向队列中压入数据
		void push(_T data);
		///从队列中弹出数据
		void pop(void);
		///队列是否为空
		bool empty(void);
		///队列当前的长度
		size_t size(void);
		///队列的最大长度
		size_t capacity(void);
		///清空队列数据并释放空间
		void clear(void);
		///为没初始化的队列预留空间
		void reserve(size_t size);
		_T* get_handle(void);
	};
	
	/**
	*@namespace RCS_CXX 
	*@class   circle_queue
	*@tparam  _T 缓冲区内数据的数据类型
	*@brief   基于vbuffer部署的循环队列,可用于数据缓冲
	*/
	template <typename _T> class circle_queue : protected vbuffer<_T>
	{
		private:
		///队列最大的长度
		size_t max_size;
		///队列数据使用的地址
		_T*    ptr;
		
		public:
		///读指针所处的下标
		size_t r_pos; 
		///写指针所处的下标
		size_t w_pos; 
		
		public:
		///允许在构造的时候什么都不指定
		circle_queue():max_size(0),r_pos(0),w_pos(0){}
		///如果希望队列使用静态内存,可以在构造的时候就指定好
		circle_queue(size_t size,_T* static_addr):max_size(size),ptr(static_addr),r_pos(0),w_pos(0){}
		
		public:
		///获取队头元素的引用
		_T& front(void)
		{
			return ptr[r_pos];
		}
		///获取队尾元素的引用
		_T& back(void)
		{
			return ptr[w_pos];
		}
		///向队列中压入数据
		void push(_T data)
		{
			w_pos++;
			if (w_pos == max_size) w_pos=0;
			ptr[w_pos]=data;
		}
		///从队列中弹出数据
		void pop(void)
		{
			r_pos++;
			if (r_pos == max_size) r_pos=0;
		}
		///队列是否为空
		bool empty(void)
		{
			return (r_pos == w_pos);
		}
		///队列当前的长度
		size_t size(void)
		{
			if (w_pos >= r_pos) return w_pos - r_pos;
			return w_pos + max_size - r_pos;
		}
		///队列的最大长度
		size_t capacity(void)
		{
			return max_size;
		}
		///为没初始化的队列预留空间(从堆中申请)
		void reserve(size_t size)
		{
			if (ptr != nullptr) RCS_allocator<_T>().deallocate(ptr,max_size);
			ptr = RCS_CXX::RCS_allocator<_T>().allocate(size);
			max_size = size;
		}
		///为没初始化的队列预留空间(直接指定空间)
		void static_reserve(void* addr,size_t size)
		{
			if (ptr != nullptr) RCS_allocator<_T>().deallocate(ptr,max_size);
			ptr = addr;
			max_size = size;
		}
		///清空队列数据并释放空间
		void clear(void)
		{
			if (ptr != nullptr) RCS_allocator<_T>().deallocate(ptr,max_size);
		}
		///[]运算符重载,允许使用[]访问队列
		_T& operator[](size_t pos){return ptr[pos];}
	};
	/**
	*@namespace RCS_CXX 
	*@class   fixed_queue
	*@tparam  _T 缓冲区内数据的数据类型
	*@brief   基于vbuffer部署的定长队列,可用于数据缓冲
	*/
	template <typename _T> class fixed_queue : protected vbuffer<_T>
	{
		private:
		///队列最大的长度
		size_t max_size;
		///队列数据使用的地址
		_T*    ptr;
		///队列当前的长度
		size_t current_size;
		
		public:
		///允许在构造的时候什么都不指定
		fixed_queue():max_size(0),current_size(0){}
		///如果希望队列使用静态内存,可以在构造的时候就指定好
		fixed_queue(size_t size,_T* static_addr):max_size(size),ptr(static_addr),current_size(0){}
			
		public:
			
		_T& front(void)
		{
			if (current_size != 0)
				return ptr[current_size-1];
			return ptr[0];
		}
		
		_T& back(void)
		{
			return ptr[0];
		}
		
		void push(_T data)
		{
			if (current_size < max_size) current_size++;
			for(int i=current_size;i>=1;i--) ptr[i]=ptr[i-1];
			ptr[0]=data;	
		}
		
		void pop(void)
		{
			if (current_size > 0) current_size--;
		}
		
		bool empty(void)
		{
			return (current_size == 0);
		}
		
		size_t size(void)
		{
			return current_size;
		}
		
		size_t capacity(void)
		{
			return max_size;
		}
		
		void reserve(size_t size)
		{
			if (ptr != nullptr) RCS_allocator<_T>().deallocate(ptr,max_size);
			ptr = RCS_CXX::RCS_allocator<_T>().allocate(size);
			max_size = size;
		}
		
		void static_reserve(void* addr,size_t size)
		{
			if (ptr != nullptr) RCS_allocator<_T>().deallocate(ptr,max_size);
			ptr = addr;
			max_size = size;
		}
		
		void clear(void)
		{
			if (ptr != nullptr) RCS_allocator<_T>().deallocate(ptr,max_size);
		}
		
		_T& operator[](size_t pos){return ptr[pos];}
	};


/* C++类定义结束 ===========================================================*/
}
/* C++命名空间结束 =========================================================*/

#endif