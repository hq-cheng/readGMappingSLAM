#ifndef SCANMATCHER_H
#define SCANMATCHER_H
#include "utils/macro_params.h"
#include "utils/point.h"
#include "utils/stat.h"
#include "smmap.h"
#include "sensor/rangereading.h"

#define LASER_MAXBEAMS 2048

namespace Gmapping {

    /**
     * ScanMatcher，扫描匹配类
     * 数据成员：
     *      m_laserPose，激光的位置，默认为(0,0,0)，也可由setLaserParameters函数成员设置
     *      m_laserBeams，激光束的数量，默认为0，也可由setLaserParameters函数成员设置
     *      m_laserAngles，激光束的夹角数据，也可由setLaserParameters函数成员设置
     * 
     *      m_laserMaxRange，激光的最大测距范围，由setMatchingParameters函数成员设置
     *      m_usableRange，（观测时，使用时）激光的最大测距范围（Zmax），由setMatchingParameters函数成员设置
     *      m_gaussianSigma，计算socre时的方差，在score函数成员中被使用，由setMatchingParameters函数成员设置
     *      m_likelihoodSigma，计算似然时的方差，在likelihoodAndScore函数成员中被使用，由setMatchingParameters函数成员设置
     *      m_kernelSize，主要用在计算score时搜索框的大小，由setMatchingParameters函数成员设置
     *      m_optAngularDelta，优化时的角度增量，由setMatchingParameters函数成员设置
     *      m_optLinearDelta，优化时的长度增量，由setMatchingParameters函数成员设置
     *      m_optRecursiveIterations，优化时的迭代次数，由setMatchingParameters函数成员设置
     *      m_likelihoodSkip，计算似然时，每隔m_likelihoodSkip个激光束，就跳过一个激光束，由setMatchingParameters函数成员设置
     *      
     *      m_llsamplerange，likelihood函数成员使用，不同解析度（如5cm/10cm/20cm/25cm）的栅格地图该参数值不一样
     *      m_llsamplestep，likelihood函数成员使用，不同解析度（如5cm/10cm/20cm/25cm）的栅格地图该参数值不一样
     *      m_lasamplerange，likelihood函数成员使用，不同解析度（如5cm/10cm/20cm/25cm）的栅格地图该参数值不一样
     *      m_lasamplestep，likelihood函数成员使用，不同解析度（如5cm/10cm/20cm/25cm）的栅格地图该参数值不一样
     *      m_generateMap，是否需要生成地图（true，会为了空闲区域分配内存；fasle，不会）
     * 
     *      m_enlargeStep，地图进行拓展的大小，默认为10
     *      m_fullnessThreshold，被认为是占用的阈值，默认为0.1
     *      m_angularOdometryReliability，里程计的角度可靠性，默认为0.0
     *      m_linearOdometryReliability，里程计的长度可靠性，默认为0.0
     *      m_freeCellRatio，free和occupany的阈值，默认为sqrt(2.0)（离激光点的空闲距离，即沿激光束方向离激光点这么远距离的栅格一定是空闲的。）
     *      m_initialBeamsSkip，去掉初始的几个激光束的数量，默认为0
     *      m_activeAreaComputed，地图有效区域是否被计算过，默认为false
     *      nullLikelihood，空似然值（激光雷达未击中，即观测模型没有输出），默认为-0.5，用作标志位
     *      m_linePoints，激光束起点到终点的路径点数组，只分配一次内存给该数组
     * 函数成员：
     *      icpOptimize，icp方法求解最优位姿
     *      optimize，似然场观测模型求解最优位姿（重载函数）
     *      registerScan，已知位置的覆盖栅格地图算法（使用的模型为Counting Model）
     *      setLaserParameters，设置激光参数，包括激光束，夹角值和激光位置
     *      setMatchingParameters，设置Scan-Matching扫描匹配参数
     *      invalidateActiveArea，使地图有效区域的计算失效，即设置m_activeAreaComputed为false
     *      computeActiveArea，计算有效区域，通过激光雷达的数据计算出来哪个地图栅格应该要被更新了
     *      icpStep，icpOptimize中的一步，这个函数目前在gmapping中没有被使用（不完善，还需修改）
     *      score，根据地图、机器人位置、激光雷达数据，计算出一个得分：原理为likelihood_field_range_finder_model
     *      likelihoodAndScore，根据地图、机器人位置、激光雷达数据，同时计算出一个得分和似然：原理为likelihood_field_range_finder_model
     *      likelihood，计算某一个机器人位姿的似然值（重载函数）
     *      laserAngles，获取激光束的夹角数据
     *      laserBeams，获取激光束的数量
     */
    class ScanMatcher {
    public:
        typedef Covariance3 CovarianceMatrix;
        // allocate this large array only once
        IntPoint* m_linePoints;
        static const double nullLikelihood;

        ScanMatcher();
        ~ScanMatcher();

        double icpOptimize( OrientedPoint& pnew, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings ) const;
        double optimize( OrientedPoint& pnew, const ScanMatcherMap& map, const OrientedPoint& init, const double* readings ) const;  // 已写
        double optimize( OrientedPoint& mean, CovarianceMatrix& cov, const ScanMatcherMap& map, const OrientedPoint& init, const double* readings ) const;
        double registerScan( ScanMatcherMap& map, const OrientedPoint& p, const double* readings );
        void setLaserParameters( unsigned int beams, double* angles, const OrientedPoint& lpose );// 已写
        void setMatchingParameters( double urange, double range, double sigma, int kernsize, double lopt, double aopt, int iterations, double likelihoodSigma=1, unsigned int likelihoodSkip=0 );// 已写
        void invalidateActiveArea();// 已写
        void computeActiveArea( ScanMatcherMap& map, const OrientedPoint& p, const double* readings );
        inline double icpStep( OrientedPoint& pret, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings ) const;
        inline double score( const ScanMatcherMap& map, const OrientedPoint& p, const double* readings ) const;// 已写
        inline unsigned int likelihoodAndScore( double& s, double& l, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings ) const;// 已写
        double likelihood( double& lmax, OrientedPoint& mean, CovarianceMatrix& cov, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings );
        double likelihood( double& _lmax, OrientedPoint& _mean, CovarianceMatrix& _cov, const ScanMatcherMap& map, const OrientedPoint& p, Gaussian3& odometry, const double* readings, double gain=180. );
        inline const double* laserAngles() const { return m_laserAngles; }
        inline unsigned int laserBeams() const { return m_laserBeams; }
        
    protected:
        bool m_activeAreaComputed;
        unsigned int m_laserBeams;
        double m_laserAngles;

        PARAM_SET_GET(OrientedPoint, laserPose, protected, public, public)
        PARAM_SET_GET(double, laserMaxRange, protected, public, public)
        PARAM_SET_GET(double, usableRange, protected, public, public)
        PARAM_SET_GET(double, gaussianSigma, protected, public, public)
        PARAM_SET_GET(double, likelihoodSigma, protected, public, public)
        PARAM_SET_GET(int,    kernelSize, protected, public, public)
        PARAM_SET_GET(double, optAngularDelta, protected, public, public)
        PARAM_SET_GET(double, optLinearDelta, protected, public, public)
        PARAM_SET_GET(unsigned int, optRecursiveIterations, protected, public, public)
        PARAM_SET_GET(unsigned int, likelihoodSkip, protected, public, public)
        PARAM_SET_GET(double, llsamplerange, protected, public, public)
        PARAM_SET_GET(double, llsamplestep, protected, public, public)
        PARAM_SET_GET(double, lasamplerange, protected, public, public)
        PARAM_SET_GET(double, lasamplestep, protected, public, public)
        PARAM_SET_GET(bool, generateMap, protected, public, public)
        PARAM_SET_GET(double, enlargeStep, protected, public, public)
        PARAM_SET_GET(double, fullnessThreshold, protected, public, public)
        PARAM_SET_GET(double, angularOdometryReliability, protected, public, public)
        PARAM_SET_GET(double, linearOdometryReliability, protected, public, public)
        PARAM_SET_GET(double, freeCellRatio, protected, public, public)
        PARAM_SET_GET(unsigned int, initialBeamsSkip, protected, public, public)
    };

    /**
     * 给定当前时刻机器人的估计位姿，利用激光雷达似然场模型与观测值
     * 去计算观测值与地图中可能击中障碍物的匹配程度
     * 相当于评价当前这个估计位姿是否准确，越准确得分越高
     * 原理为likelihood_field_range_finder_model
     * map，地图
     * p，当前t时刻估计的机器人位姿（世界坐标系）
     * readings，存储激光雷达读数的数组（观测值，一帧激光数据）
     */
    inline double ScanMatcher::score( const ScanMatcherMap& map, const OrientedPoint& p, const double* readings ) const {
        // 初始化score=0
        double s=0;
        // 将激光束的夹角数组，跳过m_initialBeamSkip个元素处的指针赋给指针变量 angle
        const double* angle = m_laserAngles + m_initialBeamSkip;
        
        // m_laserPose 表示 激光雷达坐标系 在 机器人底盘基准坐标系 中的坐标
        // 将激光雷达坐标系转换到世界坐标系
        OrientedPoint lp = p;
        lp.x += cos(p.theta) * m_laserPose.x - sin(p.theta) * m_laserPose.y;
        lp.y += sin(p.theta) * m_laserPose.x + cos(p.theta) * m_laserPose.y;
        lp.theta += m_laserPose.theta;
        
        unsigned int skip = 0;
        // 初始化地图中占用和空闲的阈值，并缩放到世界坐标系中
        double freeDelta = map.getDelta()*m_freeCellRatio;
        
        // 遍历一帧激光数据中的所有激光束
        for ( const double* r=readings+m_initialBeamSkip; r<readings+m_laserBeams; r++,angle++ ) 
        {
            skip++;
            // 每隔 m_likelihoodSkip 个激光束就跳过一个激光束
            // 或激光束超过观测时的最大测距值Zmax时，则跳过该激光束
            skip = skip > m_likehoodSkip?0:skip;
            if ( skip || *r > m_usableRange || *r == 0.0 ) 
                continue;
                
            // 计算当前激光束击中点（观测值）在世界坐标系下的坐标，再转换到地图坐标系下
            Point phit = lp;
            phit.x += *r * cos( lp.theta + *angle );
            phit.y += *r * sin( lp.theta + *angle );
            IntPoint iphit = map.world2map(phit);
            
            // 该激光束经过的最后一个空闲点在世界坐标系下的坐标
            // 原理：沿着激光束方向，击中点（观测值）的前面一个点必定是没有障碍物的
            Point pfree = lp;  
            pfree.x += (*r - freeDelta) * cos( lp.theta + *angle );
            pfree.y += (*r - freeDelta) * sin( lp.theta + *angle );
            pfree = pfree - phit;	// 两者距离
            IntPoint ipfree = map.world2map(pfree);
            
            bool found = false;
            Point bestMu(0., 0.);
            // 遍历地图中 m_kernelSize*m_kernelSize 大小的窗口
            // 搜索地图中当前激光束击中点附近距离最近的点（这个点表示一个障碍物）
            for ( int xx=-m_kernelSize; xx<=m_kernelSize; xx++ )
                for ( int yy=-m_kernelSize; yy<=m_kernelSize; yy++ ) {
                    // 击中点（观测值）和空闲点各自偏移(xx, yy)
                    IntPoint pr = iphit + IntPoint(xx, yy);
                    IntPoint pf = pr + ipfree;
                    // 得到各自对应的Cell
                    const PointAccumulator& cell = map.cell(pr);
                    const PointAccumulator& fcell = map.cell(pf);
                    // PointAccumulator类中运算符 double 重载
                    // 即(double)cell返回的是该cell被占用的概率
                    if ( ((double)cell) > m_fullnessThreshold && ((double)fcell) < m_fullnessThreshold ) {
                        // 如果地图中点pr对应的cell被占用，而pf对应的cell没有被占用，则说明找到了一个合法的点
                        Point mu = phit - cell.mean();
                        if ( !found ) {
                            bestMu = mu;
                            found = true;
                        } else {
                            bestMu = (mu*mu) < (bestMu*bestMu)?mu:bestMu;
                        }
                    }
                }
            // 计算socre，公式exp(-d^2 / sigma))，sigma表示方差
            if ( found ) {
                s += exp(-1.0/m_gaussianSigma*bestMu*bestMu);
            }
        }
        return s;
    }

    /**
     * 根据地图、机器人位置、激光雷达数据，同时计算出一个得分和似然：原理为likelihood_field_range_finder_model
     * 输出计算出来的得分（score），以及似然（粒子的权重，p(z|x,m)）
     * s，得分（score）
     * l，似然（粒子的权重，p(z|x,m)）
     * map，地图
     * p，当前t时刻估计的机器人位姿（世界坐标系）
     * readings，存储激光雷达读数的数组（观测值，一帧激光数据）
     */
    inline unsigned int likelihoodAndScore( double& s, double& l, 
        const ScanMatcherMap& map, const OrientedPoint& p, const double* readings ) const 
    {
        // 初始化s，l
        s = 0;
        l = 0;
        // 将激光束的夹角数组，跳过m_initialBeamSkip个元素处的指针赋给指针变量 angle
        const double* angle = m_laserAngles + m_initialBeamSkip;
        // m_laserPose 表示 激光雷达坐标系 在 机器人底盘基准坐标系 中的坐标
        // 将激光雷达系转换到世界坐标系
        OrientedPoint lp = p;
        lp.x += cos(p.theta) * m_laserPose.x - sin(p.theta) * m_laserPose.y;
        lp.y += cos(p.theta) * m_laserPose.y + sin(p.theta) * m_laserPose.x;
        lp.theta += m_laserPose.theta;

        // 如果没有击中的时候（没有观测到时），似然值设置为 nullLikelihood = -0.5
        double noHit = nullLikelihood / m_likelihoodSigma;

        
        unsigned int skip = 0;
        unsigned int c = 0;
        // 初始化地图中占用和空闲的阈值，并缩放到世界坐标系中
        double freeDleta = map.getDelta() * m_freeCellRatio;
        // 遍历一帧激光数据中的所有激光束（跳过了开始的m_initialBeamSkip个激光束）
        for ( const double* r = readings + m_initialBeamSkip; r<readings+m_laserBeams; r++; angle++ ) {
            skip++;
            // 每隔 m_likelihoodSkip 个激光束就跳过一个激光束
            // 或激光束超过观测时的最大测距值Zmax时，则跳过该激光束
            skip = skip > m_likelihoodSkip?0: skip;
            if ( *r > m_usableRange ) continue;
            if ( skip ) continue;

            // 计算当前激光束击中点（观测值）在世界坐标系下的坐标，再转换到地图坐标系下
            Point phit = lp;
            phit.x += *r * cos(lp.theta + *angle);
            phit.y += *r * sin(lp.theta + *angle);
            IntPoint iphit = map.world2map(phit);

            // 该激光束经过的最后一个空闲点在世界坐标系下的坐标
            // 原理：沿着激光束方向，击中点（观测值）的前面一个点必定是没有障碍物的
            Point pfree = lp;
            pfree.x += (*r - freeDleta) * cos(lp.theta + *angle);
            pfree.y += (*r - freeDleta) * sin(lp.theta + *angle);
            pfree = pfree - phit;  // 两者距离
            IntPoint ipfree = map.world2map(pfree);

            bool found = false;
            Point bestMu(0., 0.);
            // 遍历地图中 m_kernelSize*m_kernelSize 大小的窗口
            // 搜索地图中当前激光束击中点附近距离最近的点（这个点表示一个障碍物）
            for (int xx=-m_kernelSize; xx<=m_kernelSize; xx++)
                for (int yy=-m_kernelSize; yy<=m_kernelSize; yy++) {
                    // 击中点（观测值）和空闲点各自偏移(xx, yy)
                    IntPoint pr = iphit + IntPoint(xx, yy);
                    intPoint pf = pr + ipfree;
                    // 得到各自对应的Cell
                    const PointAccumulator& cell = map.cell(pr);
                    const PointAccumulator& cell = map.cell(pr);
                    // PointAccumulator类中运算符 double 重载
                    // 即(double)cell返回的是该cell被占用的概率
                    if ( ( (double)cell )>m_fullnessThreshold && ( (double)fcell )<m_fullnessThreshold ) {
                        // 如果地图中点pr对应的cell被占用，而pf对应的cell没有被占用，则说明找到了一个合法的点
                        Point mu=phit-cell.mean();
                        if (!found){
                            bestMu=mu;
                            found=true;
                        }
                        else{
                            bestMu=(mu*mu)<(bestMu*bestMu)?mu:bestMu;
                        }
                    }
                    
                }

            // 计算得分 得分是只计有用的激光束
            if (found) {
                s+=exp(-1./m_gaussianSigma*bestMu*bestMu);
                c++;
            }

            // 计算似然 似然是计算所有的激光束 如果某一个激光束打中了空地 那也需要计算进去
            if (!skip) {
                // 似然不是指数，似然只是指数的上标
                double f=(-1./m_likelihoodSigma)*(bestMu*bestMu);
                l+=(found)?f:noHit;
            }
        }
        return c;
    }

} // end namespace

#endif