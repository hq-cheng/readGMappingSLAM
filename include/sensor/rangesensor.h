#ifndef RANGESENSOR_H
#define RANGESENSOR_H
#include <vector>
#include "sensor.h"
#include "utils/point.h"

namespace Gmapping {
    
    /**
     * RangeSensor，激光传感器类，继承自传感器基类
     * 数据成员：
     *      Beam，表示一束激光（又称作一个激光点）的struct结构体
     *      newFormat，标志位，默认为false（不知道有啥用）
     *      m_pose，激光传感器的位姿（相对于base_link）
     *      m_beams，激光数据的vector容器，里面包含 beams_num 束激光（Beam）
     * 函数成员：
     *      getPose()，获取激光传感器的位姿
     *      beams()，返回所有的激光数据（重载）
     *      updateBeamsLookup()，计算 m_beams 中每一束激光的sin值和cos值
     */
    class RangeSensor: public Sensor {
    public:
        struct Beam
        {
            /* 一束激光对应的数据 */
            OrientedPoint pose;     // 相对于激光传感器中心的坐标(x,y,theta)，默认为(0,0,angle)
            double span;            // span=0 表明是一个 0 线激光
            double maxRange;        // 激光传感器最大测量范围
            double s,c;             // 该束激光的sin值和cos值 
        };
        bool newFormat;
        
        RangeSensor( std::string name );
        RangeSensor( std::string name, unsigned int beams_num, double res, const OrientedPoint& position=OrientedPoint(0,0,0), double span=0, double maxrange=89.0 );
        RangeSensor( std::string name, unsigned int beams_num, double *angles, const OrientedPoint& position=OrientedPoint(0,0,0), double span=0, double maxrange=89.0 );
    
        inline OrientedPoint getPose() const { return m_pose; }
        inline const std::vector<Beam>& beams() const { return m_beams; }
        inline std::vector<Beam>& beams() const { return m_beams; }
        void updateBeamsLookup();
    protected:
        OrientedPoint m_pose;
        std::vector<Beam> m_beams;    
    };


} // end namespace

#endif