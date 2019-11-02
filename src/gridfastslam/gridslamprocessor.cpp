#include <string>
#include <deque>
#include <list>
#include <map>
#include <set>
#include <fstream>
#include <iomanip>
#include "utils/stat.h"
#include "gridfastslam/gridslamprocessor.h"
#include "utils/point.h"

namespace GMapping {

    // 若机器人在走了m_distanceThresholdCheck这么远的距离都没有进行激光雷达的更新，则需要进行报警
    const double m_distanceThresholdCheck = 5;
    
    using namespace std;

    /**
     * 构造函数
     */
    GridSlamProcessor::GridSlamProcessor() : m_infoStream(cout) {
        period_ = 5.0;
        m_obsSigmaGain=1;
        m_resampleThreshold=0.5;
        m_minimumScore=0.;
    }

    GridSlamProcessor(std::ostream& infoS) : m_infoStream(infoS){
        period_ = 5.0;
        m_obsSigmaGain=1;
        m_resampleThreshold=0.5;
        m_minimumScore=0.;
    }

    /**
     * 拷贝构造函数
     * gsp，被拷贝的GridSlamProcessor类对象
     */
    GridSlamProcessor::GridSlamProcessor(const GridSlamProcessor& gsp) {
        period_ = 5.0;
        
        m_obsSigmaGain = gsp.m_obsSigmaGain;
        m_resampleThreshold = gsp.m_resampleThreshold;
        m_minimumScore=gsp.m_minimumScore;

        m_beams=gsp.m_beams;
        m_indexes=gsp.m_indexes;
        m_motionModel=gsp.m_motionModel;
        m_resampleThreshold=gsp.m_resampleThreshold;
        m_matcher=gsp.m_matcher;

        m_count=gsp.m_count;
        m_readingCount=gsp.m_readingCount;
        m_lastPartPose=gsp.m_lastPartPose;
        m_pose=gsp.m_pose;
        m_odoPose=gsp.m_odoPose;
        m_linearDistance=gsp.m_linearDistance;
        m_angularDistance=gsp.m_angularDistance;
        m_neff=gsp.m_neff;

        cerr << "FILTER COPY CONSTRUCTOR" << endl;
        cerr << "m_odoPose=" << m_odoPose.x << " " <<m_odoPose.y << " " << m_odoPose.theta << endl;
        cerr << "m_lastPartPose=" << m_lastPartPose.x << " " <<m_lastPartPose.y << " " << m_lastPartPose.theta << endl;
        cerr << "m_linearDistance=" << m_linearDistance << endl;
        cerr << "m_angularDistance=" << m_linearDistance << endl;

        m_xmin=gsp.m_xmin;
        m_ymin=gsp.m_ymin;
        m_xmax=gsp.m_xmax;
        m_ymax=gsp.m_ymax;
        m_delta=gsp.m_delta;

        m_regScore=gsp.m_regScore;
        m_critScore=gsp.m_critScore;
        m_maxMove=gsp.m_maxMove;
        m_linearThresholdDistance=gsp.m_linearThresholdDistance;
        m_angularThresholdDistance=gsp.m_angularThresholdDistance;
        m_obsSigmaGain=gsp.m_obsSigmaGain;

#ifdef MAP_CONSISTENCY_CHECK
    cerr << __PRETTY_FUNCTION__ <<  ": trajectories copy.... ";
#endif
    TNodeVector v=gsp.getTrajectories();
    for (unsigned int i=0; i<v.size(); i++){
		m_particles[i].node=v[i];
    }
#ifdef MAP_CONSISTENCY_CHECK
    cerr <<  "end" << endl;
#endif

        cerr  << "Tree: normalizing, resetting and propagating weights within copy construction/cloneing ..." ;
        updateTreeWeights(false);
        cerr  << ".done!" <<endl;
    }

    /**
     * 对当前GridSlamProcessor类对象进行深拷贝
     */
    GridSlamProcessor* GridSlamProcessor::clone() const {
# ifdef MAP_CONSISTENCY_CHECK
    cerr << __PRETTY_FUNCTION__ << ": performing preclone_fit_test" << endl;
    typedef std::map<autoptr< Array2D<PointAccumulator> >::reference* const, int> PointerMap;
    PointerMap pmap;
	for (ParticleVector::const_iterator it=m_particles.begin(); it!=m_particles.end(); it++){
	  const ScanMatcherMap& m1(it->map);
	  const HierarchicalArray2D<PointAccumulator>& h1(m1.storage());
 	  for (int x=0; x<h1.getXSize(); x++){
	    for (int y=0; y<h1.getYSize(); y++){
	      const autoptr< Array2D<PointAccumulator> >& a1(h1.m_cells[x][y]);
	      if (a1.m_reference){
		PointerMap::iterator f=pmap.find(a1.m_reference);
		if (f==pmap.end())
		  pmap.insert(make_pair(a1.m_reference, 1));
		else
		  f->second++;
	      }
	    }
	  }
	}
	cerr << __PRETTY_FUNCTION__ <<  ": Number of allocated chunks" << pmap.size() << endl;
	for(PointerMap::const_iterator it=pmap.begin(); it!=pmap.end(); it++)
	  assert(it->first->shares==(unsigned int)it->second);

	cerr << __PRETTY_FUNCTION__ <<  ": SUCCESS, the error is somewhere else" << endl;
# endif
        GridSlamProcessor* cloned = new GridSlamProcessor(*this);
# ifdef MAP_CONSISTENCY_CHECK
	cerr << __PRETTY_FUNCTION__ <<  ": trajectories end" << endl;
	cerr << __PRETTY_FUNCTION__ << ": performing afterclone_fit_test" << endl;
	ParticleVector::const_iterator jt=cloned->m_particles.begin();
	for (ParticleVector::const_iterator it=m_particles.begin(); it!=m_particles.end(); it++){
	  const ScanMatcherMap& m1(it->map);
	  const ScanMatcherMap& m2(jt->map);
	  const HierarchicalArray2D<PointAccumulator>& h1(m1.storage());
	  const HierarchicalArray2D<PointAccumulator>& h2(m2.storage());
	  jt++;
 	  for (int x=0; x<h1.getXSize(); x++){
	    for (int y=0; y<h1.getYSize(); y++){
	      const autoptr< Array2D<PointAccumulator> >& a1(h1.m_cells[x][y]);
	      const autoptr< Array2D<PointAccumulator> >& a2(h2.m_cells[x][y]);
	      assert(a1.m_reference==a2.m_reference);
	      assert((!a1.m_reference) || !(a1.m_reference->shares%2));
	    }
	  }
	}
	cerr << __PRETTY_FUNCTION__ <<  ": SUCCESS, the error is somewhere else" << endl;
# endif

        return cloned;            
    }

    /**
     * 析构函数
     */
    GridSlamProcessor::~GridSlamProcessor() {
        cerr << __PRETTY_FUNCTION__ << ": Start" << endl;
        cerr << __PRETTY_FUNCTION__ << ": Deleting tree" << endl;
        for (std::vector<Particle>::iterator it=m_particles.begin(); it!=m_particles.end(); it++){
#ifdef TREE_CONSISTENCY_CHECK		
            TNode* node=it->node;
            while(node)
            node=node->parent;
            cerr << "@" << endl;
#endif
        if (it->node)
        delete it->node;
        //cout << "l=" << it->weight<< endl;
        }
# ifdef MAP_CONSISTENCY_CHECK
    cerr << __PRETTY_FUNCTION__ << ": performing predestruction_fit_test" << endl;
    typedef std::map<autoptr< Array2D<PointAccumulator> >::reference* const, int> PointerMap;
    PointerMap pmap;
    for (ParticleVector::const_iterator it=m_particles.begin(); it!=m_particles.end(); it++){
      const ScanMatcherMap& m1(it->map);
      const HierarchicalArray2D<PointAccumulator>& h1(m1.storage());
      for (int x=0; x<h1.getXSize(); x++){
	for (int y=0; y<h1.getYSize(); y++){
	  const autoptr< Array2D<PointAccumulator> >& a1(h1.m_cells[x][y]);
	  if (a1.m_reference){
	    PointerMap::iterator f=pmap.find(a1.m_reference);
	    if (f==pmap.end())
	      pmap.insert(make_pair(a1.m_reference, 1));
	    else
	      f->second++;
	  }
	}
      }
    }
    cerr << __PRETTY_FUNCTION__ << ": Number of allocated chunks" << pmap.size() << endl;
    for(PointerMap::const_iterator it=pmap.begin(); it!=pmap.end(); it++)
      assert(it->first->shares>=(unsigned int)it->second);
    cerr << __PRETTY_FUNCTION__ << ": SUCCESS, the error is somewhere else" << endl;
# endif        
    }

    /**
     * 设置传感器地图
     */
    void GridSlamProcessor::setSensorMap(const SensorMap& smap) {
        // 初始化一个迭代器const_iterator，指向键名为"FLASER"的Sensor对象
        SensorMap::const_iterator laser_it = smap.find( std::string("FLASER") );
        if ( laser_it == smap.end()) {
            cerr << "Attempting to load the new carmen log format" << endl;
            laser_it=smap.find(std::string("ROBOTLASER1"));
            assert(laser_it!=smap.end());
        }
        // 将laser_it指向的Sensor对象取出
        const RangeSensor* rangeSensor=dynamic_cast<const RangeSensor*>((laser_it->second));
        assert(rangeSensor && rangeSensor->beams().size());
        // 根据取出的Sensor对象，设置 m_beams 等扫描匹配需要的激光参数
        m_beams=static_cast<unsigned int>(rangeSensor->beams().size());
        double* angles=new double[rangeSensor->beams().size()];
        for ( unsigned int i=0; i<m_beams; i++ ) {
            angles[i]=rangeSensor->beams()[i].pose.theta;
        }
        m_matcher.setLaserParameters(m_beams, angles, rangeSensor->getPose());
        delete [] angles;
    }

    /**
     * GridFastSLAM初始化，主要用于初始化各个粒子的一些信息
     */
    void GridSlamProcessor::init(unsigned int size, double xmin, double ymin, double xmax, double ymax, 
        double delta, OrientedPoint initialPose)
    {
        //设置地图大小和分辨率
        m_xmin=xmin;
        m_ymin=ymin;
        m_xmax=xmax;
        m_ymax=ymax;
        m_delta=delta;        
        // 每个粒子初始化
        m_particles.clear();
        TNode* node = new TNode( initialPose, 0, 0, 0 );
        // 粒子对应的地图初始化
        // 高分辨率地图由自己指定，低分辨率地图固定为0.1m
        ScanMatcherMap lmap( Point( Point(xmin+xmax, ymin+ymax)*.5, xmax-xmin, ymax-ymin, delta ) );
        ScanMatcherMap lowMap(Point(xmin+xmax,ymin+ymax)*0.5,xmax-xmin,ymax-ymin,0.1);
        for ( unsigned int i=0; i<size; i++ ) {
            m_particles.push_back( Particle(lmap, lowMap) );
            //m_particles.push_back(Particle(lmap));
            m_particles.back().pose=initialPose;
            m_particles.back().previousPose=initialPose;
            m_particles.back().setWeight(0);
            m_particles.back().previousIndex=0;
            // we use the root directly
            m_particles.back().node= node;
        }
        m_neff=(double)size;
        m_count=0;
        m_readingCount=0;
        m_linearDistance=m_angularDistance=0;
    }

    /**
     * 设置扫描匹配的参数，主要是调用ScanMatcher::setMatchingParameters()函数
     */ 
    void GridSlamProcessor::setMatchingParameters(double urange, double range, double sigma, 
        int kernsize, double lopt, double aopt, int iterations, double likelihoodSigma=1, 
        double likelihoodGain, unsigned int likelihoodSkip) 
    {
        m_obsSigmaGain = likelihoodGain;
        m_matcher.setMatchingParameters( urange, range, sigma, kernsize, lopt, aopt, iterations, likelihoodSigma, likelihoodSkip );
    }

	/**
	 * 设置里程计运动模型的噪声参数
	 * srr，线性运动造成的线性误差的方差
	 * srt，线性运动造成的角度误差的方差
	 * str，旋转运动造成的线性误差的方差
	 * stt，旋转运动造成的角度误差的方差
	 */ 
    void GridSlamProcessor::setMotionModelParameters(double srr, double srt, double str, double stt) {
		m_motionmodel.srr = srr;
		m_motionmodel.srt = srt;
		m_motionmodel.str = str;
		m_motionmodel.stt = stt;
    }

	/**
	 * 设置机器人更新滤波器的阈值（机器人每走过一段距离会对激光数据进行处理）
	 */
	void setUpdateDistances(double linear, double angular, double resampleThreshold) {
		m_linearThresholdDistance = linear;
		m_angularThresholdDistance = angular;
		m_resampleThreshold = resampleThreshold;
	}

	//HERE STARTS THE BEEF

	/**
	 * Particle 构造函数
	 */
	GridSlamProcessor::Particle::Particle(const ScanMatcherMap& m)
		: map(m),pose(0,0,0), weight(0), weightSum(0), gweight(0), previousIndex(0)
	{
		node=0;
	}

	// the "core" algorithm 核心算法

	/**
	 * 处理真实位姿，仿真，当里程计为理想传感器（误差为0）时使用
	 */
	void GridSlamProcessor::processTruePos( const OdometryReading& odometry ) {
		const OdometrySensor* os=dynamic_cast<const OdometrySensor*>(o.getSensor());
		if (os && os->isIdeal() && m_outputStream){
		m_outputStream << setiosflags(ios::fixed) << setprecision(3);
		m_outputStream <<  "SIMULATOR_POS " <<  o.getPose().x << " " << o.getPose().y << " " ;
		m_outputStream << setiosflags(ios::fixed) << setprecision(6) << o.getPose().theta << " " <<  o.getTime() << endl;
		}
	}

	/**
	 * 处理一帧激光雷达数据，核心算法函数
	 */
	bool GridSlamProcessor::processScan( const RangeReading& reading, int adaptParticles ) {
		/**retireve the position from the reading, and compute the odometry*/
		// 获取当前时刻里程计下机器人的位姿
		OrientedPoint relPose = reading.getPose();
		if ( !m_count ) {
			// 如果processScan()是第0次调用，则所有位姿都是一样的
			m_lastPartPose = m_odoPose = relPose;
		}

		// 利用里程计运动模型对每个粒子进行运动学采样更新，求解proposal分布
		int tmp_size = m_particles.size();
		for ( int i=0; i<tmp_size; i++ ) {
			OrientedPoint& pose( m_particles[i].pose );
			// 对每个粒子进行运动学采样更新
			pose = m_motionmodel.drawFromMotion( m_particles[i], relPose, m_odoPose );
		}
		onOdometryUpdate();  // 实际上什么都没做

		// accumulate the robot translation and rotation
		// 两次里程计位姿差值（当前时刻 - 上一时刻），计算机器人的线性位移与角度位移各自的累计值
		OrientedPoint move = relPose - m_odoPose;
		move.theta = atan2( sin(move.theta), cos(move.theta) );
		// 统计机器人在更新一帧激光雷达数据之前 走了多远的距离 以及　平移了多少的角度
		// m_linearDistance 和 m_angularDistance 每次激光雷达数据更新之后会清零
		m_linearDistance += sqrt( move*move );
		m_angularDistance += fabs( move.theta );
		
		// if the robot jumps throw a warning
		// 如果机器人在走了m_distanceThresholdCheck这么远的距离都没有进行激光雷达的更新，则报警
		if (m_linearDistance>m_distanceThresholdCheck){
			cerr << "***********************************************************************" << endl;
			cerr << "********** Error: m_distanceThresholdCheck overridden!!!! *************" << endl;
			cerr << "m_distanceThresholdCheck=" << m_distanceThresholdCheck << endl;
			cerr << "Old Odometry Pose= " << m_odoPose.x << " " << m_odoPose.y 
			<< " " <<m_odoPose.theta << endl;
			cerr << "New Odometry Pose (reported from observation)= " << relPose.x << " " << relPose.y 
			<< " " <<relPose.theta << endl;
			cerr << "***********************************************************************" << endl;
			cerr << "** The Odometry has a big jump here. This is probably a bug in the   **" << endl;
			cerr << "** odometry/laser input. We continue now, but the result is probably **" << endl;
			cerr << "** crap or can lead to a core dump since the map doesn't fit.... C&G **" << endl;
			cerr << "***********************************************************************" << endl;
		}

		// 更新一帧激光雷达数据
		m_odoPose = relPose;  // 当前时刻的里程计下机器人位姿 更新为 前一时刻的位姿
		bool processed = false;
		// process a scan only if the robot has traveled a given distance or a certain amount of time has elapsed
		// 只有当机器人走过一定的距离  或者 旋转过一定的角度  或者过一段指定的时间才处理激光数据
		if ( !m_count 
			|| m_linearDistance>=m_linearThresholdDistance 
			|| m_angularDistance>=m_angularThresholdDistance
			|| (period_ >= 0.0 && (reading.getTime() - last_update_time_) > period_) )
		{	
			// this is for converting the reading in a scan-matcher feedable form
			// 复制一帧来自ROS的激光数据，转换为scan-matching需要的格式
			assert( reading.getSize()==m_beams );
			double* plainReading = new double[m_beams];
			for ( unsigned int i=0; i<m_beams; i++ ) {
				plainReading[i] = reading.m_dists[i]
			}
			RangeReading* reading_copy = new RangeReading( reading.getSize(),
														   &(reading.m_dists[0]), 
														   static_cast<const RangeSensor*>(reading.getSensor()),
													   reading.getTime() );
			// 如果不是第一帧数据
			if ( m_count>0 ) {

				// 扫描匹配，利用最近的一次观测来提高proposal分布
				// 利用proposal分布和激光雷达数据来确定各个粒子的权重
				scanMatching(plainReading);
				onScanmatchUpdate();
				// 更新各个粒子对应的轨迹树上的累计权重
				// （调用了normalize()函数进行权重归一化处理，同时计算出有效粒子数neff值）
				updateTreeWeights(false);
				// 对最终的proposal对应的粒子群进行（自适应）重采样
				// 根据neff的大小来进行重采样  不但进行了重采样，也对地图进行更新
				resample( plainReading, adaptParticles, reading_copy );
			} else {
				// 如果是第一帧数据，则可以直接计算activeArea。因为这个时候，对机器人的位置是非常确定的，就是(0,0,0)。
				for ( ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++ ) {
					m_matcher.invalidateActiveArea();
					m_matcher.computeActiveArea(it->map, it->pose, plainReading);
					m_matcher.registerScan(it->map, it->pose, plainReading);
					TNode* node = new TNode(it->pose, 0., it->node, 0);
					node->reading = reading_copy;
					it->node = node;
				}
			}
			// 进行重采样之后，粒子的权重又会发生变化，因此需要再次更新粒子轨迹的累计权重
			// （调用了normalize()函数进行权重归一化处理，同时计算出有效粒子数neff值）
			updateTreeWeights(false);
			
			delete [] plainReading;
			m_lastPartPose=m_odoPose; // update the past pose for the next iteration
			// m_linearDistance 和 m_angularDistance 每次激光雷达数据更新之后会清零
			m_linearDistance=0;
			m_angularDistance=0;
			m_count++;
			processed = true;		
          	// keep ready for the next step
			for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++) {
				it->previousPose=it->pose;
			}	
		}
		m_readingCount++;
		return processed;	
	}

	/**
	 * 获取粒子群中 累计权重最大 的粒子的下标
	 */
	int GridSlamProcessor::getBestParticleIndex() const {
		unsigned int bi = 0;
		// 返回编译器允许的double型数最大值
		double bw = -( std::numeric_limits<double>::max() );
		for ( unsigned int i=0; i<m_particles.size(); i++ ) {
			if ( bw < m_particles[i].weightSum ) {
				bw = m_particles[i].weightSum;
				bi = i;
			}
		}
		return (int) bi;
	}

	// callbacks 回调函数，什么都没做
	void GridSlamProcessor::onScanmatchUpdate(){}
	void GridSlamProcessor::onResampleUpdate(){}
	void GridSlamProcessor::onOdometryUpdate(){}




} // end namespace