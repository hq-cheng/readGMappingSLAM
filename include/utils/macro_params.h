#ifndef MACRO_PARAMS_H
#define MACRO_PARAMS_H
/**
 * macro_params.h 使用自定义宏PARAM_SET_GET，PARAM_SET，PARAM_GET等快速设置一个C++类中数据成员的set和get属性
 */

#define PARAM_SET_GET(type, name, qualifier, setqualifier, getqualifier)\
qualifier: type m_##name;\
getqualifier: inline type get##name() const {return m_##name;}\
setqualifier: inline void set##name(type name) {m_##name=name;}

// PARAM_SET_GET(double, usableRange, protected, public, public)
// 上面等价于：
// protected: double m_usableRange;
// public: inline double getusableRange() const {return m_usableRange;}
// public: inline double setusableRange(double usableRange) {m_usableRange=usableRange;}
// 因此，PARAM_SET_GET等宏定义语句一般放置在类的开头或结尾

#define PARAM_SET(type, name, qualifier, setqualifier)\
qualifier: type m_##name;\
setqualifier: inline void set##name(type name) {m_##name=name;}

#define PARAM_GET(type, name, qualifier, getqualifier)\
qualifier: type m_##name;\
getqualifier: inline type get##name() const {return m_##name;}

#define MEMBER_PARAM_SET_GET(member, type, name, qualifier, setqualifier, getqualifier)\
getqualifier: inline type get##name() const {return member.get##name();}\
setqualifier: inline void set##name(type name) { member.set##name(name);}

#define MEMBER_PARAM_SET(member, type, name, qualifier, setqualifier, getqualifier)\
setqualifier: inline void set##name(type name) { member.set##name(name);}

#define MEMBER_PARAM_GET(member, type, name, qualifier, setqualifier, getqualifier)\
getqualifier: inline type get##name() const {return member.get##name();}

#define STRUCT_PARAM_SET_GET(member, type, name, qualifier, setqualifier, getqualifier)\
getqualifier: inline type get##name() const {return member.name;}\
setqualifier: inline void set##name(type name) {member.name=name;}

#define STRUCT_PARAM_SET(member, type, name, qualifier, setqualifier, getqualifier)\
setqualifier: inline void set##name(type name) {member.name=name;}

#define STRUCT_PARAM_GET(member, type, name, qualifier, setqualifier, getqualifier)\
getqualifier: inline type get##name() const {return member.name;}\

#define convertStringArgument(var,val,buf) if (!strcmp(buf,#val)) var=val
#endif
