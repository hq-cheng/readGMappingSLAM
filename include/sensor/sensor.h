#ifndef SENSOR_H
#define SENSOR_H
#include <string>
#include <map>

namespace Gmapping {
    
    /**
     * Sensor，传感器基类，包括里程计，激光雷达等传感器都可由此基类派生
     * 数据成员：
     *      m_name，传感器名字
     * 函数成员：
     *      getName()，获取传感器的名字
     *      setName(), 设置传感器的名字
     */
    class Sensor {
    public:
        Sensor( const std::string& name="" );
        virtual ~Sensor();

        // inline 内联函数减少开销，成员函数名后加const表示this指向的对象不能在函数内部被修改
        inline std::string getName() const { return m_name; }  
        inline void setName( const std::string& name ) { m_name = name; }
    protected:
        std::string m_name;
    };

    // 传感器的map容器类型别名定义
    // 车子上有多少传感器都可以存入map容器里面
    typedef std::map<std::string, Sensor*> SensorMap;

}  // end namespace


#endif