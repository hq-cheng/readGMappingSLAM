#include "sensor/rangesensor.h"

namespace Gmapping {

    RangeSensor::RangeSensor( std::string name ): Sensor( name ) {
    }

    /**
     * 构造函数，给定 res 激光的角度解析度
     * beams_num，激光束的数量
     * res，激光的角度解析度，用于求解激光角度
     */
    RangeSensor::RangeSensor( std::string name, unsigned int beams_num, double res, const OrientedPoint& position, 
        double span, double maxrange ): Sensor(name), m_pose(position), m_beams(beams_num)
    {   // m_beams(beams_num)，这是一种构造size指定的vector的初始化方法
        double angle = -.5*res*beams_num;
        for ( unsigned int i=0; i<beams_num; angle+=res ) {
            RangeSensor::Beam& beam( m_beams[i] );  // 定义了 m_beams[i] 的一个临时别名（引用）beam
            beam.span = span;
            beam.pose.x = 0;
            beam.pose.y = 0;
            beam.pose.theta = angle;
            beam.maxRange = maxrange;
        }
        newFormat = 0;
        updateBeamsLookup();
    }

    /**
     * 构造函数，给定 angles 激光束相对于传感器中心坐标中的角度数组
     * beams_num，激光束的数量
     * angles，存储有激光角度的数组
     */
    RangeSensor::RangeSensor( std::string name, unsigned int beams_num, double *angles, const OrientedPoint& position, 
        double span=0, double maxrange ): Sensor(name), m_pose(position), m_beams(beams_num) 
    {
        for ( unsigned int i=0; i<beams_num; i++ ) {
            RangeSensor::Beam& beam( m_beams[i] );
            beam.span = span;
            beam.pose.x = 0;
            beam.pose.y = 0;
            beam.pose.theta = angles[i];
            beam.maxRange = maxrange;
        }
        newFormat = 0;
        updateBeamsLookup();
    }

    RangeSensor::updateBeamsLookup() {
        for ( unsigned int i=0; i<m_beams.size(); i++ ) {
            RangeSensor::Beam& beam( m_beams[i] );
            beam.s = sin( m_beams[i].pose.theta );
            beam.c = cos( m_beams[i].pose.theta );
        }
    }

}