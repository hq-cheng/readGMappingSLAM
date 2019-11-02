#ifndef GRIDSLAMPROCESSOR_H
#define GRIDSLAMPROCESSOR_H
#include <climits>
#include <limits>
#include <vector>
#include <deque>
#include <fstream>
#include "utils/macro_params.h"
#include "utils/point.h"
#include "scanmatcher/scanmatcher.h"
#include "motionmodel/motionmodel.h"
#include "sensor/odometryreading.h"
#include "sensor/rangesensor.h"
#include "sensor/rangereading.h"
#include "particlefilter/particlefilter.h"

namespace GMapping {

    /**
     * GridSlamProcessor，GridFastSLAM算法实现类，实现了一个RBPF，且每个粒子都拥有自己的地图和机器人位姿。
     * 
     * 算法流程：
     * 获取里程计下机器人位姿（两时刻测量的位姿差值为控制输入），对每一个粒子的上一时刻最优估计位姿进行运动模型来更新采样
     * 运动模型更新采样得到的位姿作为初始值，读取当前时刻激光雷达数据，利用观测模型进行Scan-Matching（扫描匹配）
     * （Scan-Matching实际是在观测模型的基础上，根据地图来进一步优化位姿，得到最优的粒子群，即proposal分布对应的粒子群）
     * 权重处理，权重更新，重采样
     * 数据成员：
     *      TNode，树的结点（结构体），一个树储存了一整条轨迹，一个节点表示这条轨迹中的其中一个点
     *      Particle，粒子滤波器中的粒子（结构体），每个粒子有对应的地图、位姿、权重、轨迹
     *      m_matcher，ScanMatcher类，扫描匹配类对象，用于Scan-Matching算法实现
     * 
     *      m_motionModel，MotionModel类，里程计运动模型类对象，用于运动模型更新采样算法的实现
     *      m_beams，一帧激光数据的激光束数量
     *      last_update_time_，上一次处理激光数据的时刻
     *      period_，用于判断两次处理激光数据时刻的间隔
     *      m_particles，vector数组，存储Particle类型的粒子
     *      m_indexes，vector数组，重采样之后，剩余的（或被保留的）粒子的下标（这个下标的是会重复的）
     *      m_weights，vector数组，所有粒子当前的权重
     *      m_count，判断processScan()函数的调用次数（是否是第0次调用）
     *      m_readingCount，每成功调用processScan()函数一次，该函数内部会是其处理激光数据的次数+1
     *      m_lastPartPose，将当前时刻里程计下机器人位姿赋给它，为下一次迭代做准备
     *      m_odoPose，上一时刻里程计下机器人位姿
     *      m_pose
     *      m_linearDistance，统计机器人在进行激光雷达更新之前 走了多远的距离 以及　平移了多少的角度（累计值）
     *      m_angularDistance，统计机器人在进行激光雷达更新之前 走了多远的距离 以及　平移了多少的角度（累计值）
     * 
     *      m_xmin，地图的尺寸（double类型，worldSize）
     *      m_xmax，地图的尺寸
     *      m_ymin，地图的尺寸
     *      m_ymax，地图的尺寸
     *      m_delta，地图的分辨率，即世界坐标系到地图坐标系的缩放比例关系（worldSize=mapSize*delta）
     * 
     *      m_neff，自适应重采样中的Neff值（有效粒子数个数）
     *      m_resampleThreshold，自适应重采样中Neff应判断的阈值
     *      m_minimumScore，在scanMatch()函数中评估扫描匹配结果，若score大于该值则优化成功，可更新最优位姿
     *      m_regScore
     *      m_critScore
     *      m_maxMove
     *      m_linearThresholdDistance，每当机器人走过一段距离后，处理一次激光数据
     *      m_angularThresholdDistance ，每当机器人转动一段角度后再，处理一次激光数据
     *      m_obsSigmaGain，权重归一化时使用的平滑因子
     * 
     * 函数成员：
     *      clone，对当前GridSlamProcessor类对象进行深拷贝
     *      setSensorMap，设置传感器地图
     *      init，GridFastSLAM初始化，主要用于初始化各个粒子的一些信息
     *      setMatchingParameters，设置扫描匹配的参数，主要是调用ScanMatcher::setMatchingParameters()函数
     *      setMotionModelParameters，设置里程计运动模型的噪声参数
     *      setUpdateDistances，设置机器人更新滤波器的阈值（机器人每走过一段距离会对激光数据进行处理）
     *      setUpdatePeriod，设置机器人更新滤波器的阈值（机器人每走过一段距离会对激光数据进行处理）
     * 
     *      processTruePos，处理真实位姿，仿真，当里程计为理想传感器（误差为0）时使用
     *      processScan，处理一帧激光雷达数据，核心算法函数
     * 
     *      getParticles，获取m_particles数组（存储Particle类型的粒子）
     *      getIndexes，获取m_indexes数组（存储重采样之后，剩余的粒子的下标）
     *      getBestParticleIndex，获取粒子群中 累计权重最大 的粒子的下标
     *      scanMatch，扫描匹配算法实现函数
     *      normalize，权重归一化处理，同时计算出有效粒子数neff值
     *      resample，重采样
     * 
     *      getTrajectories，得到所有粒子的轨迹的起点(得到了轨迹的起点，就可以得到一整条轨迹)
     *      integrateScanSequence，集成所有的scan（这个函数没有被调用过。）
     *      updateTreeWeights，更新权重（调用了normalize()函数进行权重归一化处理，同时计算出有效粒子数neff值）
     *      resetTree，把每个粒子对应的轨迹树中各个节点的 accWeight 清零
     *      propagateWeights，更新权重，被updateTreeWeights()函数调用
     */
    class GridSlamProcessor {
    public:
        /**
         * TNode，树的结点 结构体，一个树储存了一整条轨迹，一个节点表示这条轨迹中的其中一个点
         * 每个粒子对应一个树，从该树最近的一个node，遍历该树，可以得到该粒子对应的整条轨迹
         * 
         * 数据成员：
         *      pose，该节点对应的机器人位姿
         *      weight，该节点对应粒子的权重
         *      accWeight，之前所有节点对应粒子的权重之和（该节点的子节点对应粒子的权重之和）
         *      parent，指向父节点的指针，每个粒子的路径都是记录最近的一个点，然后通过parent指针遍历整条路径
         *      reading，该节点激光雷达的读数
         *      childs，该节点的子节点的数量
         */
        struct TNode {
            OrientedPoint pose;
            double weight;
            double accWeight;
            double gweight;
            TNode* parent; // 无指向子节点的指针，只有指向parent的指针
            const RangeReading* reading;
            unsigned int childs;
            mutable unsigned int visitCounter; // counter in visiting the node (internally used)
            mutable bool flag;  // visit flag (internally used)

            TNode(const OrientedPoint& pose, double weight, TNode* parent=0, unsigned int childs=0);
            ~TNode();
        };
        /**
         * Particle，粒子滤波器中的粒子结构体，每个粒子有对应的地图、位姿、权重、轨迹
         * 数据成员：
         *      map，地图
         *      running_scans，存储最近的N帧激光雷达的数据，用来生成临时地图，从而进行CSM
         *      pose，位姿
         *      previousPose，前一时刻机器人位姿，用于计算里程计位移
         *      weight，权重
         *      weightSum，累计权重（该粒子在各个时刻的权重之和，轨迹上的各个节点的权重之和）
         *      gweight，
         *      previousIndex，上一个粒子的下标
         *      node，该粒子对应的整条轨迹（树），存储的是粒子的最近的一个节点（树中没有孩子的那个节点）
         *      （每个粒子对应一个树，从该树最近的一个node，遍历该树，可以得到该粒子对应的整条轨迹）
         */
        struct Particle {
            ScanMatcherMap map;
            std::vector<GMapping::RangeReading*> running_scans;
            OrientedPoint pose;
            OrientedPoint previousPose;
            double weight;
            double weightSum;
            double gweight;
            int previousIndex;
            TNode* node;

            Particle(const ScanMatcherMap& map);
            inline operator double() const { return weight; }
            inline operator OrientedPoint() const { return pose; }
            inline void setWeight( double w ) { weight = w };
        };
        typedef std::vector<GridSlamProcessor::TNode*> TNodeVector;
        typedef std::deque<GridSlamProcessor::TNode*> TNodeDeque;
        typedef std::vector<Particle> ParticleVector;
        ScanMatcher m_matcher;

        GridSlamProcessor();
        GridSlamProcessor(std::ostream& infoStr);
        GridSlamProcessor* clone() const;
        virtual ~GridSlamProcessor();

        // methods for accessing the parameters
        void setSensorMap(const SensorMap& smap);
        void init(unsigned int size, double xmin, double ymin, double xmax, double ymax, double delta, 
            OrientedPoint initialPose=OrientedPoint(0,0,0));
        void setMatchingParameters(double urange, double range, double sigma, int kernsize, double lopt, double aopt, 
                    int iterations, double likelihoodSigma=1, double likelihoodGain=1, unsigned int likelihoodSkip=0);
        void setMotionModelParameters(double srr, double srt, double str, double stt);
        void setUpdateDistances(double linear, double angular, double resampleThreshold);
        void setUpdatePeriod(double p) {period_=p;}
        

        // the "core" algorithm 核心算法
        void processTruePos( const OdometryReading& odometry );
        bool processScan( const RangeReading& reading, int adaptParticles=0 );

        // callbacks 回调函数，什么都没做
        virtual void onOdometryUpdate();
        virtual void onResampleUpdate();
        virtual void onScanmatchUpdate();

        int getBestParticleIndex() const;
        inline ParticleVector& getParticles() const { return m_particles; }
        inline std::vector<unsigned int>& getIndexes() const { return m_indexes; }
        /**the stream used for writing the output of the algorithm*/
        std::ofstream& outputStream() { return m_outputStream; };
        /**the stream used for writing the info/debug messages*/
        std::ostream& infoStream( return m_infoStream; );

        // gridslamprocessor_tree.cpp 中定义
        TNodeVector getTrajectories() const;
        void integrateScanSequence(TNode* node);

        // 获取扫描匹配类对象m_matcher的属性
        /**the maxrange of the laser to consider */
        MEMBER_PARAM_SET_GET(m_matcher, double, laserMaxRange, protected, public, public);
        /**the maximum usable range of the laser. A beam is cropped to this value. [scanmatcher]*/
        MEMBER_PARAM_SET_GET(m_matcher, double, usableRange, protected, public, public);
        /**The sigma used by the greedy endpoint matching. [scanmatcher]*/
        MEMBER_PARAM_SET_GET(m_matcher,double, gaussianSigma, protected, public, public);
        /**The sigma  of a beam used for likelihood computation [scanmatcher]*/
        MEMBER_PARAM_SET_GET(m_matcher,double, likelihoodSigma, protected, public, public);
        /**The kernel in which to look for a correspondence[scanmatcher]*/
        MEMBER_PARAM_SET_GET(m_matcher, int,    kernelSize, protected, public, public);
        /**The optimization step in rotation [scanmatcher]*/
        MEMBER_PARAM_SET_GET(m_matcher, double, optAngularDelta, protected, public, public);
        /**The optimization step in translation [scanmatcher]*/
        MEMBER_PARAM_SET_GET(m_matcher, double, optLinearDelta, protected, public, public);
        /**The number of iterations of the scanmatcher [scanmatcher]*/
        MEMBER_PARAM_SET_GET(m_matcher, unsigned int, optRecursiveIterations, protected, public, public);
        /**the beams to skip for computing the likelihood (consider a beam every likelihoodSkip) [scanmatcher]*/
        MEMBER_PARAM_SET_GET(m_matcher, unsigned int, likelihoodSkip, protected, public, public);
        /**translational sampling range for the likelihood [scanmatcher]*/
        MEMBER_PARAM_SET_GET(m_matcher, double, llsamplerange, protected, public, public);
        /**angular sampling range for the likelihood [scanmatcher]*/
        MEMBER_PARAM_SET_GET(m_matcher, double, lasamplerange, protected, public, public);
        /**translational sampling range for the likelihood [scanmatcher]*/
        MEMBER_PARAM_SET_GET(m_matcher, double, llsamplestep, protected, public, public);
        /**angular sampling step for the likelihood [scanmatcher]*/
        MEMBER_PARAM_SET_GET(m_matcher, double, lasamplestep, protected, public, public);
        /**generate an accupancy grid map [scanmatcher]*/
        MEMBER_PARAM_SET_GET(m_matcher, bool, generateMap, protected, public, public);
        /**enlarge the map when the robot goes out of the boundaries [scanmatcher]*/
        MEMBER_PARAM_SET_GET(m_matcher, bool, enlargeStep, protected, public, public);
        /**pose of the laser wrt the robot [scanmatcher]*/
        MEMBER_PARAM_SET_GET(m_matcher, OrientedPoint, laserPose, protected, public, public);

        // 获取里程计运动模型类对象m_motionModel的属性
        /**odometry error in translation as a function of translation (rho/rho) [motionmodel]*/
        STRUCT_PARAM_SET_GET(m_motionModel, double, srr, protected, public, public);
        /**odometry error in translation as a function of rotation (rho/theta) [motionmodel]*/
        STRUCT_PARAM_SET_GET(m_motionModel, double, srt, protected, public, public);
        /**odometry error in rotation as a function of translation (theta/rho) [motionmodel]*/
        STRUCT_PARAM_SET_GET(m_motionModel, double, str, protected, public, public);
        /**odometry error in  rotation as a function of rotation (theta/theta) [motionmodel]*/
        STRUCT_PARAM_SET_GET(m_motionModel, double, stt, protected, public, public);

    protected:
        MotionModel m_motionModel;
        unsigned int m_beams;
        double last_update_time_;
        double period_;
        ParticleVector m_particles;
        std::vector<unsigned int> m_indexes;
        std::vector<double> m_weights;
        int m_count, m_readingCount;
        OrientedPoint m_lastPartPose;
        OrientedPoint m_odoPose;
        OrientedPoint m_pose;
        double m_linearDistance, m_angularDistance;

        // Copy constructor 拷贝构造函数
        GridSlamProcessor(const GridSlamProcessor& gsp);

        //processing parameters (size of the map)
        PARAM_GET(double, xmin, protected, public);
        PARAM_GET(double, ymin, protected, public);
        PARAM_GET(double, xmax, protected, public);
        PARAM_GET(double, ymax, protected, public);
        //processing parameters (resolution of the map)
        PARAM_GET(double, delta, protected, public);
        PARAM_GET(double, neff, protected, public);
        /**this sets the neff based resampling threshold*/
        PARAM_SET_GET(double, resampleThreshold, protected, public, public);
        /**minimum score for considering the outcome of the scanmatching good*/
        PARAM_SET_GET(double, minimumScore, protected, public, public);
        //registration score (if a scan score is above this threshold it is registered in the map)
        PARAM_SET_GET(double, regScore, protected, public, public);
        //registration score (if a scan score is below this threshold a scan matching failure is reported)
        PARAM_SET_GET(double, critScore, protected, public, public);
        //registration score maximum move allowed between consecutive scans
        PARAM_SET_GET(double, maxMove, protected, public, public);
        //process a scan each time the robot translates of linearThresholdDistance
        PARAM_SET_GET(double, linearThresholdDistance, protected, public, public);
        //process a scan each time the robot rotates more than angularThresholdDistance
        PARAM_SET_GET(double, angularThresholdDistance, protected, public, public);  
        //smoothing factor for the likelihood
        PARAM_SET_GET(double, obsSigmaGain, protected, public, public);
        //stream in which to write the gfs file
        std::ofstream m_outputStream;
        // stream in which to write the messages
        std::ostream& m_infoStream;

    private:
        // gridslamprocessor.hxx 中定义
        inline void scanMatch( const double *plainReading );
        inline void normalize();
        inline bool resample( const double* plainReading, int adaptParticles, const RangeReading* rr=0 );
        
        // gridslamprocessor_tree.cpp 中定义
        void updateTreeWeights(bool weightsAlreadyNormalized = false);
        void resetTree();
        double propagateWeights();
};

typedef std::multimap<const GridSlamProcessor::TNode*, GridSlamProcessor::TNode*> TNodeMultimap;

// scanMatch normalize resample 在此文件中定义
#include "gridslamprocessor.hxx"

} // end namespace

#endif