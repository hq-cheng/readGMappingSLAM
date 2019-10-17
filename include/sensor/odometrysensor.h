#ifndef ODOMETRYSENSOR_H
#define ODOMETRYSENSOR_H
#include "sensor.h"

namespace Gmapping {
    
    /**
     * OdometrySensor，里程计类，继承自传感器基类
     * 数据成员：
     *      m_ideal，标明是否是理想的里程计
     * 函数成员：
     *      isIdeal()，判断当前里程计是否为理想（误差为0）的传感器
     */
    class OdometrySensor: public Sensor {
    public:
        OdometrySensor( const std::string& name, bool ideal=false );

        inline bool isIdeal() const { return m_ideal; }
    protected:
        bool m_ideal;
    };

} // end namespace

#endif