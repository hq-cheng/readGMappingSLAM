#ifndef SMMAP_H
#define SMMAP_H
#include "utils/point.h"
#include "grid/map.h"
#include "grid/harray2d.h"

#define SIGHT_INC 1

namespace Gmapping {
    /**
     * PointAccumulator，表示地图ScanMatcherMap的单元（栅格）数据类型
     * 即Map类中的Cell，表示地图中的一个cell
     * 数据成员：
     *      acc，累计被击中的坐标值，默认为(0,0)（每个cell被击中的时候，有可能不是在同一个位置被击中的）
     *      n，累计被击中的次数，默认为0（只有激光束击中之后才会累加）
     *      visits，累计被访问的次数，默认为0（只要激光束经过，不管击中与否，都会累加）
     *      unknown_ptr，静态PointAccumulator类型的指针，默认为0，作常量使用，用于标记未知的指针
     * 函数成员：
     *      update，更新某个Cell的状态，若被击中，则需要对cell的值进行累加
     *      mean，获取某个Cell的击中坐标累加值acc的平均值
     *      double()，运算符重载，获取某个Cell被占用的概率（击中次数/访问次数）
     *      add，给定一个PointAccumulator类型的Cell，与当前Cell的状态相加
     *      entropy，求熵运算，以二值分布的形式计算熵
     *      Unknown，利用静态指针unknown_ptr（默认为0）获取一个Cell（栅格）
     */
    struct PointAccumulator {
        typedef point<float> FloatPoint;

        /**构造函数 */
        PointAccumulator() : acc(0,0), n(0), visits(0) {}
        // 只有当i=-1时，返回一个默认的PointAccumulator类型对象
        PointAccumulator( int i ) : acc(0,0), n(0), visits(0) { assert(i==-1); }
        inline void update( bool value, const Point& p=Point(0,0) );
        inline Point mean() const { return 1./n*Point(acc.x, acc.y); }
        inline operator double() const { return visits?(double)n*SIGHT_INC/(double)visits:-1; }
        inline void add( const PointAccumulator& p ) { acc=acc+p.acc; n+=p.n; visits+=p.visits; }
        inline double entropy() const;

        static const PointAccumulator& Unknown();

        FloatPoint acc;
        int n, visits;
        static PointAccumulator* unknown_ptr;
    };

    /**
     * 更新某个Cell的状态
     * 若被击中，则需要利用当前击中坐标对Cell的acc坐标值进行累加
     * 未被击中，则只对Cell的访问次数进行累加
     * value，表示是否被击中
     * p，表示被击中的坐标（世界坐标系）
     */
    void PointAccumulator::update( bool value, const Point& p=Point(0,0) ) {
        if ( value ) {
            acc.x += static_cast<float>(p.x);
            acc.y += static_cast<float>(p.y);
            n++;
            visits += SIGHT_INC;
        } esle {
            visits ++;
        }
    }

    /**
     * 求熵运算
     * 先以计数总量n除以访问量visits，然后以二值分布的形式计算熵
     */
    double PointAccumulator::entropy() {
        // 从未被访问过
        if ( !visits ) {
            return -log(.5);
        }
        // 一直被击中（击中次数/访问次数=1）或从未被击中
        if ( n==visits || n==0 ) {
            return 0;
        }
        // 求解被占用的概率（击中次数/访问次数）
        double x = (double)n*SIGHT_INC / (double)visits;
        // 以二值分布的形式计算熵
        return -(x*log(x) + (1-x)*log(1-x));
    }

    // ScanMatcherMap，Scan-Matching中使用的地图数据类型
    // Cell类型为PointAccumulator，Storage存储数据类型为HierarchicalArray2D<PointAccumulator>
    typedef Map< PointAccumulator, HierarchicalArray2D<PointAccumulator> > ScanMatcherMap;
}

#endif