#ifndef AUTOPTR_H
#define AUTOPTR_H
#include <assert.h>

namespace GMapping{
/**
 * GMapping是十几年前的项目了，那时C++11的标准还没有出来
 * 所以Gmapping自定义了类autoptr实现了智能指针的机制
 * 数据成员：
 * 		m_reference，自定义结构体reference类型
 * 函数成员：
 * 		=，重载运算符"="，实现智能指针赋值语句
 * 		*，重载运算符"*"，这样就能以类似指针的形式来访问对象了
 */
template <class X>
class autoptr{
	protected:
	
	public:
	struct reference{
		X* data;				// 指向实际对象的指针
		unsigned int shares;	// 计数器，记录了使用data所指对象的autoptr数量	
	};
	inline autoptr(X* p=(X*)(0));
	inline autoptr(const autoptr<X>& ap);
	inline autoptr& operator=(const autoptr<X>& ap);
	inline ~autoptr();
	inline operator int() const;
	inline X& operator*();
	inline const X& operator*() const;
	//p	
	reference * m_reference;
	protected:
};

/**
 * 构造函数，给定指向X类型对象的指针
 */ 
template <class X>
autoptr<X>::autoptr(X* p){
	m_reference=0;
	if (p){
		m_reference=new reference;
		m_reference->data=p;
		m_reference->shares=1;
	}
}

/**
 * 构造函数，给定智能指针autoptr类型的对象
 * 每当通过拷贝的形式构造新的autoptr，就意味着多了一个autoptr的指针引用目标对象
 * 此时需要增加对象的计数shares
 */
template <class X>
autoptr<X>::autoptr(const autoptr<X>& ap){
	m_reference=0;
	reference* ref=ap.m_reference;
	if (ap.m_reference){
		m_reference=ref;
		m_reference->shares++;
	}
}

/**
 * 重载运算符"="，实现智能指针赋值语句
 * ap，智能指针类型右值
 */ 
template <class X>
autoptr<X>& autoptr<X>::operator=(const autoptr<X>& ap){
	reference* ref=ap.m_reference;
	// 左值本来就等于右值
	if (m_reference==ref){
		return *this;
	}
	// 左值不为空时，且指向目标对象计数减一（若结果为0，则释放左值指向的资源）
	if (m_reference && !(--m_reference->shares)){
		delete m_reference->data;
		delete m_reference;
		m_reference=0;
	}	
	// 目标对象加一，并将右值赋给左值
	if (ref){
		m_reference=ref;
		m_reference->shares++;
	} 
//20050802 nasty changes begin
	else
		m_reference=0;
//20050802 nasty changes end
	return *this;
}

/**
 * 析构函数
 * 当一个autoptr对象的生命周期结束的时候，都意味着少了一个指向目标对象的指针，故减小目标对象的计数。 
 * 当计数值减小到0的时候，说明不再有指针指向目标对象，其生命周期应该结束了。
 */
template <class X>
autoptr<X>::~autoptr(){
	if (m_reference && !(--m_reference->shares)){
		delete m_reference->data;
		delete m_reference;
		m_reference=0;
	}	
}

template <class X>
autoptr<X>::operator int() const{
	return m_reference && m_reference->shares && m_reference->data;
}

/**
 * 重载运算符"*"，这样就能以类似指针的形式来访问对象了
 */
template <class X>
X& autoptr<X>::operator*(){
	assert(m_reference && m_reference->shares && m_reference->data);
	return *(m_reference->data);
}

/**
 * 重载运算符"*"，这样就能以类似指针的形式来访问对象了
 */
template <class X>
const X& autoptr<X>::operator*() const{
	assert(m_reference && m_reference->shares && m_reference->data);
	return *(m_reference->data);
}

};
#endif
