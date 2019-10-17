#ifndef ODOMETRYREADING_H
#define ODOMETRYREADING_H
#include "sensorreading.h"
#include "odometrysensor.h"
#include "utils/point.h"

namespace Gmapping {

    /**
     * OdometryReading，里程计数据读取类，继承自传感器数据读取基类
     * 读取一帧里程计信息
     * 数据成员：
     *      m_pose，位姿
     *      m_speed，速度
     *      m_acceleration，加速度
     * 函数成员：
     *      getPose()，获取里程计传递过来的位姿
     *      getSpeed()，获取里程计传递过来的速度
     *      getAcceleration()，获取里程计传递过来的加速度
     *      setPose()，设置位姿
     *      setSpeed()，设置速度
     *      setAcceleration()，设置加速度
     */
    class OdometryReading: public SensorReading {
    public:
        OdometryReading( const OdometrySensor* odo, double time=0 );

        inline const OrientedPoint& getPose() const { return m_pose; }
        // 成员函数名之前加const，表示返回的对象是一个const对象
        inline const OrientedPoint& getSpeed() const { return m_speed; }
        inline const OrientedPoint& getAcceleration() const { return m_acceleration; }
        inline void setPose( const OrientedPoint& pose ) { m_pose = pose; }
        inline void setSpeed( const OrientedPoint& speed ) { m_speed = speed; }
        inline void setAcceleration( const OrientedPoint& acceleration ) { m_acceleration = acceleration; }
    protected:
        OrientedPoint m_pose;
        OrientedPoint m_speed;
        OrientedPoint m_acceleration;
    };

} // end namespace

#endif