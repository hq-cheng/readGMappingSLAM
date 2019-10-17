#include "motionmodel/motionmodel.h"

namespace Gmapping {

    /**
     * 里程计运动模型采样函数，参考 《Probabilistic Robotics》 5.4.2节
     * p，之前t-1时刻估计的机器人位姿（世界坐标系下）
     * pnew，当前t时刻里程计读数（里程计坐标系下）
     * pold，之前t-1时刻里程计读数（里程计坐标系下）
     */
    OrientedPoint MotionModel::drawFromMotion( const OrientedPoint& p, const OrientedPoint& pnew, const OrientedPoint& pold ) const {
        double sxy = 0.3*srr;

        // 观测值为u_t = ( pnew, pold )，先由观测值计算t-1时刻到t时刻的运动 (pnew - pold)
        // 并且由里程计坐标系转换到机器人底盘基准坐标系
        OrientedPoint delta = absoluteDifference( pnew, pold );
        
        // 求解误差分布，并进行高斯采样（返回误差值）
        OrientedPoint noisypoint(delta);
        // 最终得到一个带噪声的点（估算的“真值” = 观测值 + 误差采样值）
        noisypoint.x += sampleGaussian( srr*fabs(delta.x) + str*fabs(delta.theta) + sxy*fabs(delta.y) );
        noisypoint.y += sampleGaussian( srr*fabs(delta.y) + str*fabs(delta.theta) + sxy*fabs(delta.x) );
        noisypoint.theta += sampleGaussian( srt*sqrt(delta.x*delta.x+delta.y*delta.y) + stt * delta.theta );

        // 限制运动角度为(-pi, pi)
        noisypoint.theta = fmod( noisypoint.theta, 2*M_PI );
        if ( noisypoint.theta > M_PI )
            noisypoint.theta -= 2*M_PI;

        // 运动模型更新，得到当前t时刻估计的机器人位姿
        return absoluteSum(p, noisypoint);
    }

    /**
     * 里程计运动模型采样函数，参考 《Probabilistic Robotics》 5.4.2节
     * p，之前t-1时刻估计的机器人位姿（世界坐标系下）
     * linearMove，机器人的t-1时刻到t时刻的线性位移（世界坐标系下）
     * angularMove，机器人的t-1时刻到t时刻的角度位移（世界坐标系下）
     */
    OrientedPoint MotionModel::drawFromMotion( const OrientedPoint& p, double linearMove, double angularMove ) const {

        double lm=linearMove  + fabs( linearMove ) * sampleGaussian( srr ) + fabs( angularMove ) * sampleGaussian( str );
        double am=angularMove + fabs( linearMove ) * sampleGaussian( srt ) + fabs( angularMove ) * sampleGaussian( stt );
        n.x+=lm*cos(n.theta+.5*am);
        n.y+=lm*sin(n.theta+.5*am);
        n.theta+=am;
        n.theta=atan2(sin(n.theta), cos(n.theta));
        return n;

    }

    Covariance3 gaussianApproximation(const OrientedPoint& pnew, const OrientedPoint& pold) const {
        OrientedPoint delta=absoluteDifference(pnew,pold);
        
        /*两个位置的线性位移和角度位移*/
        double linearMove=sqrt(delta.x*delta.x+delta.y*delta.y);
        double angularMove=fabs(delta.x);
        
        
        double s11=srr*srr*linearMove*linearMove;
        double s22=stt*stt*angularMove*angularMove;
        double s12=str*angularMove*srt*linearMove;
        
        Covariance3 cov;
        double s=sin(pold.theta),c=cos(pold.theta);
        
        cov.xx=c*c*s11+MotionModelConditioningLinearCovariance;
        cov.yy=s*s*s11+MotionModelConditioningLinearCovariance;
        cov.tt=s22+MotionModelConditioningAngularCovariance;
        cov.xy=s*c*s11;
        cov.xt=c*s12;
        cov.yt=s*s12;
        
        return cov;
    }

} // end namespace