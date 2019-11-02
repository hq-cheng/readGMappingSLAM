#ifndef MOTIONMODEL_H
#define MOTIONMODEL_H
#include "utils/point.h"、
#include "utils/stat.h"

namespace Gmapping {

    /**
     * MotionModel，里程计运动模型类，主要是运动模型的更新和采样
     * 数据成员：
     *      srr，线性运动造成的线性误差的方差
     *      str，旋转运动造成的线性误差的方差
     *      srt，线性运动造成的角度误差的方差
     *      stt，旋转运动造成的角度误差的方差
     * 函数成员：
     *      drawFromMotion(), 里程计运动模型采样函数（重载，运动直接更新，或运动分解更新）
     *      
     */
    class MotionModel {
    public:
        double srr, str, srt, stt;

        OrientedPoint drawFromMotion( const OrientedPoint& p, const OrientedPoint& pnew, const OrientedPoint& pold ) const;
        OrientedPoint drawFromMotion( const OrientedPoint& p, double linearMove, double angularMove ) const;
        Covariance3 gaussianApproximation(const OrientedPoint& pnew, const OrientedPoint& pold) const;
    };
    
} // end namespace

#endif