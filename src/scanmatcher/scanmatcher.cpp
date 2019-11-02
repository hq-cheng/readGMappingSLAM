#include "scanmatcher/scanmatcher.h"

namespace Gmapping {

    using namespace std;

    const double ScanMatcher::nullLikelihood = -.5;

    /**
     * 构造函数
     */
    ScanMatcher::ScanMatcher() : m_laserPose(0,0,0) {

        m_laserBeams = 0;
        //m_laserAngles=0;

        m_optRecursiveIterations=3;

        m_enlargeStep = 10.;
        m_fullnessThreshold = 0.1;

        m_angularOdometryReliability = 0.;
        m_linearOdometryReliability = 0.;

        m_freeCellRatio = sqrt(2.)

        m_initialBeamsSkip = 0;

        m_activeAreaComputed = false;

        // This  are the dafault settings for a grid map of 5 cm
        m_llsamplerange = 0.01;
        m_llsamplestep = 0.01;
        m_lasamplerange = 0.005;
        m_lasamplestep = 0.005;

        /*
        // This  are the dafault settings for a grid map of 10 cm
        m_llsamplerange=0.1;
        m_llsamplestep=0.1;
        m_lasamplerange=0.02;
        m_lasamplestep=0.01;
        */

        /*
        // This  are the dafault settings for a grid map of 20/25 cm
        m_llsamplerange=0.2;
        m_llsamplestep=0.1;
        m_lasamplerange=0.02;
        m_lasamplestep=0.01;
        m_generateMap=false;
        */

        m_linePoints = new IntPoint[20000];
    }    

    /**
     * 析构函数
     */
    ScanMatcher::~ScanMatcher() {
        delete [] m_linePoints;
    }

    /** 
     * 使地图有效区域的计算失效
     * 每次调用computeActiveArea()之前，都必须要调用这个函数
     */
    void ScanMatcher::invalidateActiveArea() {
        m_activeAreaComputed = false;
    }

    /** 设置激光雷达参数 */
    void ScanMatcher::setLaserParameters( unsigned int beams, double* angles, const OrientedPoint& lpose ) {
        assert(beams < LASER_MAXBEAMS);
        m_laserPose = lpose;
        m_laserBeams = beams;
        //m_laserAngles=new double[beams];
        memcpy( m_laserAngles, angles, sizeof(double)*m_laserBeams );
    }

    /** 设置匹配参数 */
    void ScanMatcher::setMatchingParameters( double urange, double range, 
        double sigma, int kernsize, double lopt, double aopt, int iterations, 
        double likelihoodSigma=1, unsigned int likelihoodSkip=0 )
    {
        m_usableRange = urange;
        m_laserMaxRange = range;
        m_gaussianSigma = sigma;
        m_kernelSize = kernsize;
        m_optLinearDelta = lopt;
        m_optAngularDelta = aopt;
        m_optRecursiveIterations = iterations;
        m_likelihoodSigma = likelihoodSigma;
        m_likelihoodSkip = likelihoodSkip;
    }

    /**
     * 给定scan（激光雷达观测数据）和map（地图，作参考帧），利用似然场观测模型，
     * 优化里程计运动模型更新得到的估计位姿，利用该位姿迭代求解得到一个最优位姿
     * 这个是Gmapping中主要的Scan-Matching扫描匹配算法
     * pnew，优化得到的最优位姿（输出）
     * map，地图（参考帧）
     * init，当前时刻估计位姿（当前帧）
     * readings，激光雷达观测数据
     */
    double ScanMatcher::optimize( OrientedPoint& pnew, const ScanMatcherMap& map, const OrientedPoint& init, const double* readings ) const {
        double bestScore = -1;
        OrientedPoint currentPose = init;
        // 利用激光雷达似然场观测模型进行扫描匹配，优化当前位姿
        // 即计算出一个初始的优化位姿（不一定是最优的）并返回对应的得分值
        double currentScore = score(map, currentPose, readings);

        // 每次迭代的角度增量，和线性位移增量
        double adelta = m_optAngularDelta, ldelta =m_optLinearDelta;
        // 无效迭代（这一次迭代算出来的得分比上一次要差）的次数
        unsigned int refinement = 0;
        // 搜索的方向，前，后，左，右，左转，右转
        enum Move { Front, Back, Left, Right, TurnLeft, TurnRight, Done };
        // 搜索次数（每个方向算一次，一次不论有效，或无效）
        int c_iterations = 0
        do {
            // 如果这一次(currentScore)算出来比上一次(bestScore)差
            // 则有可能是走太多了，要减少搜索步长（减半）
            if ( bestScore >= currentScore ) {
                refinement++;
                adelta *= .5;
                ldelta *= .5;
            }
            bestScore = currentScore;
            OrientedPoint bestLocalPose = currentPose;
            OrientedPoint localPose = currentPose;
            // 将这6个方向都搜索一次，得到这6个方向中最好的一个位姿及其对应得分
            Move move = Front;
            do {
                localPose = currentPose;
                switch ( move ) {
                    case Front:
                        localPose.x += ldelta;
                        move = Back;
                        break;
                    case Back:
                        localPose.x -= ldelta;
                        move = Left;
                        break;
                    case Left:
                        localPose.y -= ldelta;
                        move = Right;
                        break;
                    case Right:
                        localPose.y += ldelta;
                        move = TurnLeft;
                        break;
                    case TurnLeft:
                        localPose.theta += adelta;
                        move = TurnRight;
                        break;
                    case TurnRight:
                        localPose.theta -= adelta;
                        move = Done;
                        break;
                    default:;
                }
                // 计算当前方向的位姿（角度，线性位移）和原始估计位姿（init）的区别，区别越大增益越小
                // 若里程计比较可靠的话，则进行匹配时就需要对离原始估计位姿（init）比较远的位姿施加惩罚
                double odo_gain = 1;
                if ( m_angularOdometryReliability > 0. ) {
                    double dth=init.theta-localPose.theta; 	dth=atan2(sin(dth), cos(dth)); 	dth*=dth;
                    odo_gain*=exp(-m_angularOdometryReliability*dth);                    
                }
                if ( m_linearOdometryReliability > 0. ) {
                    double dx=init.x-localPose.x;
                    double dy=init.y-localPose.y;
                    double drho=dx*dx+dy*dy;
                    odo_gain*=exp(-m_linearOdometryReliability*drho);                    
                }
                // 计算当前方向的位姿对应的得分
                double localScore = odo_gain*score( map, localPose, readings );
                // 若得分更好，则更新
                if ( localScore > currentScore ) {
                    currentScore = localScore;
                    bestLocalPose = localPose;
                }
                c_iterations++;
            } while( move != Done );
            // 把当前6个方向中最好的位姿设置为最优位姿，若都6个方向无效的话，这个值不会被更新
            currentPose = bestLocalPose;
        // 这一次迭代得分更好，继续下一次迭代；
        // 或这一次迭代更差（无效迭代），修改角度和线性位移增量，继续下一次迭代，直至超出无效迭代次数的最大值
        } while ( currentScore > bestScore || refinement < m_optRecursiveIterations );
        // 返回最优位姿及其对应得分
        pnew = currentPose;
        return bestScore;
    }



} // end namespace