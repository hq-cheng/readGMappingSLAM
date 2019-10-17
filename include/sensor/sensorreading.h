#ifndef SENSORREADING_H
#define SENSORREADING_H
#include "sensor.h"

namespace Gmapping {

    /**
     * SensorReading，传感器数据读取基类
     * SLAM问题不太关心传感器的原理，只要能读取出对应时刻下的数据即可
     * 数据成员：
     *      m_time，传感器读取的时间
     *      m_sensor，传感器对象
     * 函数成员：
     *      getTime()，获取传感器读数的读取时刻
     *      setTime()，设置传感器读数的读取时刻
     *      getSensor()，获取当前传感器对象
     */
    class SensorReading {
    public:
        SensorReading( const Sensor* s=0, double time=0 );
        virtual ~SensorReading();

        inline double getTime() const { return m_time; }
        inline void setTime( double time ) { m_time=time; }
        inline Sensor* getSensor() const { return m_sensor; }
    protected
        double m_time;
        const Sensor* m_sensor;
    };

} // end namespace

#endif