#ifdef MACOSX
// This is to overcome a possible bug in Apple's GCC.
#define isnan(x) (x==FP_NAN)
#endif

/**
 * 扫描匹配算法实现函数
 * 对每个粒子而言，在里程计运动模型采样更新得到的机器人位姿的基础上，通过优化来求解最优估计位姿
 * 或者说，通过最近的一次激光雷达数据（观测值），来优化proposal分布
 * 除此之外，还会计算优化之后，每个粒子的权重（这里权重用似然表示）
 */
inline void GridSlamProcessor::scanMatch( const double *plainReading ) {

    double sumScore = 0;
    for ( ParticleVector::ietrator it=m_particles.begin(); it!=m_particles.end(); it++ ) {
        OrientedPoint corrected;
        double score, l, s;
        // 给定scan（激光雷达观测数据）和map（地图，作参考帧），利用似然场观测模型，
        // 优化里程计运动模型更新得到的估计位姿，利用该位姿迭代求解得到一个最优位姿
        score = m_matcher.optimize( corrected, it->map, it->pose, plainReading );
        // 如果优化成功，则更新该粒子的位姿为optimize()输出的最优估计位姿
        if ( score>m_minimumScore ) {
            it->pose = corrected;
        } else {
            // 如果优化失败，则仍使用之前里程计运动模型采样更新得到机器人位姿
            if (m_infoStream) {
                m_infoStream << "Scan Matching Failed, using odometry. Likelihood=" << l <<std::endl;
                m_infoStream << "lp:" << m_lastPartPose.x << " "  << m_lastPartPose.y << " "<< m_lastPartPose.theta <<std::endl;
                m_infoStream << "op:" << m_odoPose.x << " " << m_odoPose.y << " "<< m_odoPose.theta <<std::endl;
            }
        }
        // 优化粒子的估计位姿之后，重新计算粒子的权重值
        // 相当于粒子滤波器中的观测步骤，计算p(z|x,m)，粒子的权重由粒子的似然来表示
        m_matcher.likelihoodAndScore( s, l, it->map, it->pose, plainReading );
        sumScore += score;
        // 一个粒子的权重并不是当前时刻的最优位姿对应的似然值来表示，而是所有时刻的似然值之和来表示
        it->weight += l;
        it->weightSum += l;
        // 计算出来最优的位姿之后，进行地图的扩充  这里不会进行内存分配
        // 不进行内存分配的原因是这些粒子进行重采样之后有可能会消失掉，因此在后面进行冲采样的时候统一进行内存分配。
        // 理论上来说，这里的操作是没有必要的，因为后面的重采样的时候还会进行一遍
        m_matcher.invalidateActiveArea();
        m_matcher.computeActiveArea(m_particles[i].map, m_particles[i].pose, plainReading);
    }
    if (m_infoStream)
        m_infoStream << "Average Scan Matching Score=" << sumScore/m_particles.size() << std::endl;
}

/**
 * 权重归一化处理，同时计算出有效粒子数neff值
 */
inline void GridSlamProcessor::normalize() {
    
    // normalize the log m_weights
    double gain = 1./( m_obsSigmaGain*m_particles.size() );
    // 返回编译器允许的double型数最大值
    double lmax = -(std::numeric_limits<double>::max());
    // 求所有粒子中的最大的权重，并将这个最大权重值暂存到lmax中
    for ( ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++ ) {
        lmax = it->weight>lmax?it->weight:lmax;
    }
    // cout << "!!!!!!!!!!! maxwaight= "<< lmax << endl;

    // 以最大权重为中心的高斯分布
    m_weights.clear();
    double wcum = 0;
    m_neff = 0;
    for ( std::vector<Particle>::iterator it=m_particles.begin(); it!=m_particles.end(); it++ ) {
        m_weights.push_back( exp( gain*(it->weight - lmax) ) );
        wcum += m_weights.back();
        // cout << "l=" << it->weight<< endl;
    }
    // 计算有效粒子数 和 归一化权重
    m_neff=0;
    for ( std::vector<double>::iterator it=m_weights.begin(); it!=m_weights.end(); it++ ) {
        *it = *it/wcum;
        // 权重 wi = exp( (1/SigmaGain*N)*(wi - wmax) ) / sum( exp( (1/SigmaGain*N)*(wi - wmax) ) )
        double w = *it;
        m_neff += w*w;
    }
    // 有效粒子数 neff = 1./ sum( wi*wi )
    m_neff = 1./m_neff;
}

/**
 * 重采样（会用到 particlefilter.h 文件中的相关数据结构）
 * Neff值小于阈值时需要重采样，则所有保留下来的粒子的轨迹都加上一个新的节点，然后进行地图更新。
 * 否则不需要重采样，则所有的粒子的轨迹都加上一个新的节点，然后进行地图的更新
 */
inline bool GridSlamProcessor::resample(const double* plainReading, int adaptSize, const RangeReading* reading) {

    bool hasResampled = false;
    // 备份旧的粒子对应的轨迹树，即保留树的叶子节点，在增加新节点的时候使用
    TNodeVector oldGeneration;
    for ( unsigned int i=0; i<m_particles.size(); i++ ) {
        oldGeneration.push_back( m_particles[i].node );
    }

    // Neff值小于阈值时需要重采样
    if ( m_neff<m_resampleThreshold*m_particles.size() ) {
        // 重采样根据权值进行采样，采用度盘轮转算法，决定哪些粒子会保留，哪些会消失
        uniform_resampler<double, double> resampler;
        // 保留的粒子会返回下标.里面的下标可能会重复，因为有些粒子会重复采样
        m_indexes = resampler.resampleIndexes( m_weights, adaptSize );
        onResampleUpdate();
        
        //BEGIN: BUILDING TREE
        ParticleVector temp; // 该数组暂存重采样之后的粒子群
        unsigned int j = 0;
        std::vector<unsigned int> deletedParticles; // 要删除的粒子下标
        // 枚举每一个要被保留的粒子下标，并且找出要被删除的粒子下标
        for ( unsigned int i=0; i<m_indexes.size(); i++ ) {
            // 统计要被删除的粒子下标
            while ( j<m_indexes[i] ) {
                deletedParticles.push_back(j);
                j++;
            }
            // 遇到一个要被保留的粒子下标时
            if ( j==m_indexes[i]  )
                j++;
            // 得到当前这个被保留的粒子
            Particle& p = m_particles[ m_indexes[i] ];
            // 每一个被保留下来的粒子都需要在轨迹树中增加一个新的节点（即新增一个叶子节点）
            TNode* node = 0;
            TNode* oldNode = oldGeneration[ m_indexes[i] ];
            // 创建一个新的节点 它的父节点为oldNode
            node = new TNode( p.pose, 0, oldNode, 0 );  // 这里权重为0，故重采样后要再更新一次轨迹树上的权重
            node->reading = reading;
            // temp数组暂存重采样之后的粒子群
            temp.push_back(p);
            temp.back().node = node;
            temp.back().previousIndex = m_indexes[i];
        }
        // 处理那些重复的下标
        while ( j<m_indexes.size() ) {
            deletedParticles.push_back(j);
            j++;           
        }
        // 将要被删除的粒子对应的Node都删掉
        for ( unsigned int i=0; i<deletedParticles.size(); i++) {
            delete m_particles[ deletedParticles[i] ].node;
            m_particles[ deletedParticles[i] ].node = 0;
        }
        // 清楚全部的粒子 然后从temp中读取保留下来的粒子
        m_particles.clear();
        for (ParticleVector::iterator it=temp.begin(); it!=temp.end(); it++) {
            it->setWeight(0); // 重采样后，每个粒子的权重都设置为相同的值，这里为0
            // 增加了一帧激光数据 因此需要更新地图
            m_matcher.invalidateActiveArea();
            m_matcher.registerScan(it->map, it->pose, plainReading);
            m_particles.push_back(*it);
        }
        hasResampled = true;
    } else {
        // 否则不需要重采样，权值不变。只为轨迹创建一个新的节点
        int index = 0;
        TNodeVector::iterator node_it = oldGeneration.begin();
        for ( ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++ ) {
            //create a new node in the particle tree and add it to the old tree
            //BEGIN: BUILDING TREE  
            TNode* node = 0;
            node = new TNode( it->pose, 0.0, *node_it, 0 );
            node->reading = reading;
            it->node = node;
            //END: BUILDING TREE
            m_matcher.invalidateActiveArea();
            m_matcher.registerScan(it->map, it->pose, plainReading);
            it->previousIndex = index;
            index++;
            node_it++;
        }
        std::cerr  << "Done" <<std::endl;
    }
    //END: BUILDING TREE
    return hasResampled;
}