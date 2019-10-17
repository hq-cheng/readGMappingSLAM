#ifndef RANGEREADING_H
#define RANGEREADING_H
#include "sensorreading.h"
#include "rangesensor.h"
#include "utils/point.h"

namespace Gmapping {

    /**
     * RangeReading，激光传感器数据读取类，继承自传感器数据读取基类
     * 读取激光传感器的 距离数据 与 角度数据
     * 数据成员：
     *      m_pose，机器人位姿（不是激光传感器的位姿）
     *      m_dists，激光距离数据的 vector 容器，包含每一束激光所测的距离数据
     *      m_beams，激光束的数量
     *      m_angles，激光角度数据的 vector 容器，包含每一束激光对应的角度
     * 函数成员：
     *      getPose()，读取这一帧激光数据对应的机器人位姿
     *      setPose()，设置这一帧激光数据对应的机器人位姿
     *      getSize()，返回激光束的数量
     *      rawView()，返回density过滤之后的激光值
     *      cartesianForm()，转换激光数据到笛卡尔坐标系下
     *      activeBeams()，指定激光密度，有多少有效的激光     
     */
    class RangeReading: SensorReading {
    public:
        unsigned int m_beams;
        std::vector<double> m_dists;
        std::vector<double> m_angles;

        RangeReading( const RangeSensor* rs, double time=0 );
        RangeReading( unsigned int n_beams, const double* d, const RangeSensor* rs, double time=0 );
        RangeReading( unsigned int n_beams, const double* d, const double* angles, const RangeSensor* rs, double time=0 );
        virtual ~RangeReading();

        inline const OrientedPoint& getPose() const { return m_pose; }
        inline void setPose( const OrientedPoint& pose ) { m_pose = pose; }
        inline const unsigned int getSize() const { return m_beams; }
        unsigned int rawView( double* v, double density=0. ) const;
        std::vector<Point> cartesianForm( double maxRange=1e6 ) const;
        unsigned int activeBeams( double density=0. ) const;

    protected:
        OrientedPoint m_pose;
    }

} // end namespace

#endif