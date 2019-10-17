#ifndef MAP_H
#define MAP_H
#include <assert.h>
#include "harray2d.hpp"
#include "array2d.h"
#include "accessstate.hpp"
#include "utils/point.h"

namespace Gmapping {

    typedef Array2D<double> DoubleArray2D;

    /**
     * Map，栅格地图类（地图），在连续的物理世界和离散的栅格地图之间建立一个映射关系。
     * 在GMapping中每个粒子都维护了一个地图，并最终采用了最优粒子的地图。
     * 它们使用类GMapping::ScanMatcherMap来描述地图，参考smmap.hpp文件：
     * typedef Map<PointAccumulator,HierarchicalArray2D<PointAccumulator> > ScanMatcherMap;
     * 其中地图单元（栅格）类型为PointAccumulator， 存储结构为HierachicalArray2D<PointAccumulator>
     * 
     * 模板参数：
     * Cell，地图中每个单元（栅格）的数据类型
     * Storage，地图的存储结构类型
     * 
     * 数据成员：
     *      m_center，地图中心的坐标（世界坐标系下）
     *      m_worldSizeX，世界坐标系X轴大小（物理世界是一个连续的坐标空间，需要使用double类型）
     *      m_worldSizeY，世界坐标系Y轴大小
     *      m_delta，地图的分辨率，即世界坐标系到地图坐标系的缩放比例关系（worldSize=mapSize*delta）
     *      m_mapSizeX，地图长度（栅格地图的表示则是离散的，所以使用int类型）
     *      m_mapSizeY，地图宽度
     *      m_sizeX2，地图长度的一半
     *      m_sizeY2，地图宽度的一半
     *      m_storage，存储地图数据的对象，HierachicalArray2D<PointAccumulator>类型
     *      m_unknown，静态成员变量，返回一个默认的Cell类型对象（PointAccumulator类型默认输出的栅格
     * 
     * 函数成员：
     *      resize，重新设置地图大小
     *      grow，扩展地图大小
     *      world2map，世界坐标系转换到地图坐标系（重载函数）
     *      map2world，地图坐标系转换到世界坐标系（重载函数）
     *      getCenter，获取地图中心点的坐标
     *      getworldSizeX，获取世界坐标系X轴大小
     *      getworldSizeY，获取世界坐标系Y轴大小
     *      getMapSizeX，获取地图长度
     *      getMapSizeY，获取地图宽度
     *      getDelta，获取地图分辨率
     *      getMapResolution，获取地图分辨率
     *      getResolution，获取地图分辨率
     *      getSize，获取世界坐标系的X轴，Y轴的起点与终点（xmin, ymin, xmax, ymax）
     *      cell，获取给定坐标处的Cell（重载函数）
     *      isInside，给定一个点的坐标，判断是否在地图里面（重载函数）
     *      storage，获取存储地图数据的对象，即m_storage（重载函数）
     *      toDoubleArray，依据当前的Map对象构造DoubleArray（没有使用过）
     *      toDoubleMap，依据当前的Map对象构造DoubleMap（没有使用过）
     */
    template< class Cell, class Storage, const bool isClass=true >
    class Map {
    public:
        static const Cell m_unknown;

        Map( int mapSizeX, int mapSizeY, double delta );
        Map( const Point& center, double worldSizeX, double worldSizeY, double delta );
        Map( const Point& center, double xmin, double ymin, double xmax, double ymax, double delta );
        void resize( double xmin, double ymin, double xmax, double ymax );
        void grow( double xmin, double ymin, double xmax, double ymax );

        inline IntPoint world2map( const Point& p ) const;
        inline Point map2world( const IntPoint& p ) const;
        inline IntPoint world2map( double x, double y ) const { return world2map( Point(x,y) ); }
        inline Point map2world( int x, int y ) const { return map2world( Inpoint(x,y) ); }

        inline Point getCenter() const { return m_cenetr; }
        inline double getWorldSizeX() const { return m_worldSizeX; }
        inline double getWorldSizeY() const { return m_worldSizeY; }
        inline int getMapSizeX() const { return m_mapSizeX; }
        inline int getMapSizeY() const { return m_mapSizeY; }
        inline double getDelta() const { return m_delta; }
        inline double getMapResolution() const { return m_delta; }
        inline double getResolution() const { return m_delta; }
        inline void getSize( double& xmin, double& ymin, double& xmax, double& ymax ) const {
            Point min=map2world( 0, 0 ), max = map2world( IntPoint(m_mapSizeX-1, m_mapSizeY-1) );
            xmin = min.x, ymin = min.y, xmax = max.x, ymax = max.y;
        }

        inline Cell& cell( const IntPoint& p );
        inline Cell& cell( int x, int y ) {
            return cell( IntPoint(x, y) );
        }
        inline const Cell& cell( const IntPoint& p ) const;
        inline const Cell& cell( int x, int y ) const {
            return cell( IntPoint(x, y) );
        }
        inline Cell& cell( const Point& p );
        inline Cell& cell( double x, double y ) {
            return cell( Point(x, y) );
        }
        inline const Cell& cell( const Point& p ) const;
        inline const Cell& cell( double x, double y ) const {
            return cell( Point(x, y) );
        }

        inline bool isInside( int x, int y ) const {
            return m_storage.cellState(IntPoint(x, y))&Inside;
        }
        inline bool isInside( const IntPoint& p ) const {
            return m_storage.cellState(p)&Inside;
        }
        inline bool isInside( double x, double y ) const {
            return m_storage.cellState(world2map(x, y))&Inside;
        }
        inline bool isInside( const Point& p ) const {
            return m_storage.cellState(world2map(p))&Inside;
        }

        inline Storage& storage() { return m_storage; }
        inline const Storage& storage() const { return m_storage; }

        DoubleArray2D* toDoubleArray() cosnt; // 没有使用过
        Map< double, DoubleArray2D, false >* toDoubleMap() const; // 没有使用过
        
        

    protected:
        Point m_center;
        double m_worldSizeX, m_worldSizeY, m_delta;
        int m_mapSizeX, m_mapSizeY;
        int m_sizeX2, m_sizeY2;
        Storage m_storage;
    };

    // 浮点数地图
    typedef Map< double, DoubleArray2D, false > DoubleMap;

    /**
     * m_unknown，静态成员变量初始化，返回一个默认的Cell类型对象（PointAccumulator类型默认输出的栅格）
     */
    template< class Cell, class Storage, const bool isClass >
    const Cell Map<Cell, Storage, isClass>::m_unknown = Cell(-1);

    /**
     * 构造函数，给定地图的大小，分辨率（worldSize=mapSize*delta）
     */
    template< class Cell, class Storage, const bool isClass >
    Map<Cell, Storage, isClass>::Map( int mapSizeX, int mapSizeY, double delta )
        : m_storage( mapSizeX, mapSizeY )
    {
        // 这里是否有初始化m_mapSizeX，m_mapSizeY？？？
        // 我认为没有，应该添加：
        // m_mapSizeX = mapSizeX;
        // m_mapSizeY = mapSizeY;
        m_worldSizeX = mapSizeX * delta;
        m_worldSizeY = mapSizeY * delta;
        m_delta = delta;
        m_center = Point( 0.5*m_worldSizeX, 0.5*m_worldSizeY );
        m_sizeX2 = m_mapSizeX >> 1;
        m_sizeY2 = m_mapSizeY >> 1;
    }

    /**
     * 构造函数，给定世界坐标系的大小，地图的分辨率（worldSize=mapSize*delta）
     */
    template< class Cell, class Storage, const bool isClass >
    Map<Cell, Storage, isClass>::Map( const Point& center, 
        double worldSizeX, double worldSizeY, double delta )
        : m_storage( (int)ceil(worldSizeX/delta), (int)ceil(worldSizeY/delta) ) 
    {
        m_center = center;
        m_worldSizeX = worldSizeX;
        m_worldSizeY = worldSizeY;
        m_delta = delta;
        // 这里得注意，m_mapSizeX是原始地图的大小
        // 划分地图金字塔后，得到元素为Patch指针的二维数组大小（地图金字塔Patch数组层）：
        // m_storage.getXSize() = m_mapSizeX >> m_storage.m_patchMagnitude
        m_mapSizeX = m_storage.getXSize() << m_storage.getPatchSize();
        m_mapSizeY = m_storage.getYSize() << m_storage.getPatchSize();
        m_sizeX2 = m_mapSizeX >> 1;
        m_sizeY2 = m_mapSizeY >> 1;    
    }

    /**
     * 构造函数，给定世界坐标系的大小，地图的分辨率（worldSize=mapSize*delta）
     * worldSizeX = xmax-xmin
     * worldSizeY = ymax-ymin
     */
    template< class Cell, class Storage, const bool isClass >
    Map<Cell, Storage, isClass>::Map( const Point& center, double xmin, double ymin, 
        double xmax, double ymax, double delta )
        : m_storage( (int)ceil((xmax-xmin)/delta), (int)ceil((ymax-ymin)/delta) )
    {
        m_center = center;
        m_worldSizeX = xmax-xmin;
        m_worldSizeY = ymax-ymin;
        m_delta = delta;
        m_mapSizeX = m_storage.getXSize() << m_storage.getPatchSize();
        m_mapSizeY = m_storage.getYSize() << m_storage.getPatchSize();
        m_sizeX2 = (int)round((m_center.x - xmin)/m_delta);
        m_sizeY2 = (int)round((m_center.y - ymin)/m_delta);       
    }

    /**
     * 重新设置地图大小
     * worldSizeX = xmax-xmin
     * worldSizeY = ymax-ymin
     */
    template< class Cell, class Storage, const bool isClass >
    void Map<Cell, Storage, isClass>::resize( double xmin, double ymin, double xmax, double ymax ) {
        // 将尺寸由世界坐标系转换到地图坐标系
        IntPoint imin=world2map(xmin, ymin);
        IntPoint imax=world2map(xmax, ymax);
        // 获取元素为Patch指针的二维数组大小（地图金字塔Patch数组层）
        int pxmin, pymin, pxmax, pymax;
        pxmin=(int)floor((float)imin.x/(1<<m_storage.getPatchMagnitude()));
        pxmax=(int)ceil((float)imax.x/(1<<m_storage.getPatchMagnitude()));
        pymin=(int)floor((float)imin.y/(1<<m_storage.getPatchMagnitude()));
        pymax=(int)ceil((float)imax.y/(1<<m_storage.getPatchMagnitude()));
        // 重新修改地图金字塔的大小
        m_storage.resize(pxmin, pymin, pxmax, pymax);

        m_mapSizeX=m_storage.getXSize()<<m_storage.getPatchSize();
        m_mapSizeY=m_storage.getYSize()<<m_storage.getPatchSize();
        m_worldSizeX=xmax-xmin;
        m_worldSizeY=ymax-ymin;
        m_sizeX2-=pxmin*(1<<m_storage.getPatchMagnitude());
        m_sizeY2-=pymin*(1<<m_storage.getPatchMagnitude())
    }

    /**
     * 扩展地图大小
     * worldSizeX=xmax-xmin
     * worldSizeY=ymax-ymin
     */
    template < class Cell, class Storage, const bool isClass >
    void Map<Cell,Storage,isClass>::grow(double xmin, double ymin, double xmax, double ymax)
    {
        // 将尺寸由世界坐标系转换到地图坐标系
        IntPoint imin=world2map(xmin, ymin);
        IntPoint imax=world2map(xmax, ymax);
        //尺寸更大，才需要扩展
        if (isInside(imin) && isInside(imax))
            return;
        imin=min(imin, IntPoint(0,0));
        imax=max(imax, IntPoint(m_mapSizeX-1,m_mapSizeY-1));
        // 获取元素为Patch指针的二维数组大小（地图金字塔Patch数组层）
        int pxmin, pymin, pxmax, pymax;
        pxmin=(int)floor((float)imin.x/(1<<m_storage.getPatchMagnitude()));
        pxmax=(int)ceil((float)imax.x/(1<<m_storage.getPatchMagnitude()));
        pymin=(int)floor((float)imin.y/(1<<m_storage.getPatchMagnitude()));
        pymax=(int)ceil((float)imax.y/(1<<m_storage.getPatchMagnitude()));
        // 重新修改地图金字塔的大小
        m_storage.resize(pxmin, pymin, pxmax, pymax);

        m_mapSizeX=m_storage.getXSize()<<m_storage.getPatchSize();
        m_mapSizeY=m_storage.getYSize()<<m_storage.getPatchSize();
        m_worldSizeX=xmax-xmin;
        m_worldSizeY=ymax-ymin;
        m_sizeX2-=pxmin*(1<<m_storage.getPatchMagnitude());
        m_sizeY2-=pymin*(1<<m_storage.getPatchMagnitude());
    }

    /**
     * 世界坐标系转换到地图坐标系
     * p，世界坐标系中一个点的坐标
     */
    template< class Cell, class Storage, const bool isClass >
    IntPoint Map<Cell, Storage, isClass>::world2map( const Point& p ) const {
        return InPoint( (int)round((p.x-m_center.x)/m_delta)+m_sizeX2, (int)round((p.y-m_center.y)/m_delta)+m_sizeY2 );
    }

    /**
     * 地图坐标系转换到世界坐标系
     * p，地图坐标系中一个点的坐标
     */
    template< class Cell, class Storage, const bool isClass >
    Point Map<Cell, Storage, isClass>::map2world( const IntPoint& p ) const {
        return Point( (p.x-m_sizeX2)*m_delta, (p.y-m_sizeY2)*m_delta ) + m_center;
    }

    /**
     * 获取给定坐标处的Cell
     * p，地图坐标系下某个点的坐标
     */
    template< class Cell, class Storage, const bool isClass >
    Cell& Map<Cell, Storage, isClass>::cell( const IntPoint& p ) {
        AccessibilityState s = m_storage.cellState(p);
        // 如果点p不在地图内
        if ( !(s&Inside) )
            assert(0);
        return m_storage.cell(p);
    }

    template< class Cell, class Storage, const bool isClass >
    const Cell& Map<Cell, Storage, isClass>::cell( const IntPoint& p ) const {
        AccessibilityState s=m_storage.cellState(p);
        // 如果点p对应的栅格已经分配内存，则返回对应的Cell
        if (s&Allocated)	 
            return m_storage.cell(p);
        // 否则返回一个默认的Cell
        return m_unknown;       
    }
    
     /**
     * 获取给定坐标处的Cell
     * p，世界坐标系下某个点的坐标
     */   
    template< class Cell, class Storage, const bool isClass >
    Cell& Map<Cell, Storage, isClass>::cell( const Point& p ) {
        IntPoint ip = world2map(p);
        AccessibilityState s = m_storage.cellState(ip);
        // 如果点p不在地图内
        if ( !(s&Inside) )
            assert(0);
        return m_storage.cell(ip);
    }

    template< class Cell, class Storage, const bool isClass >
    const Cell& Map<Cell, Storage, isClass>::cell( const Point& p ) const {
        IntPoint ip = world2map(p);
        AccessibilityState s=m_storage.cellState(ip);
        // 如果点p对应的栅格已经分配内存，则返回对应的Cell
        if (s&Allocated)	 
            return m_storage.cell(ip);
        // 否则返回一个默认的Cell
        return m_unknown;       
    }

    //FIXME check why the last line of the map is corrupted.
    template < class Cell, class Storage, const bool isClass >
    DoubleArray2D* Map<Cell,Storage,isClass>::toDoubleArray() const
    {
            DoubleArray2D* darr=new DoubleArray2D(getMapSizeX()-1, getMapSizeY()-1);
        for(int x=0; x<getMapSizeX()-1; x++)
            for(int y=0; y<getMapSizeY()-1; y++)
            {
                IntPoint p(x,y);
                darr->cell(p)=cell(p);
            }
        return darr;
    }


    template < class Cell, class Storage, const bool isClass >
    Map<double, DoubleArray2D, false>* Map<Cell,Storage,isClass>::toDoubleMap() const
    {
    //FIXME size the map so that m_center will be setted accordingly
            Point pmin=map2world(IntPoint(0,0));
        Point pmax=map2world(getMapSizeX()-1,getMapSizeY()-1);
        Point center=(pmax+pmin)*0.5;
        Map<double, DoubleArray2D, false>*  plainMap=new Map<double, DoubleArray2D, false>(center, (pmax-pmin).x, (pmax-pmin).y, getDelta());
        for(int x=0; x<getMapSizeX()-1; x++)
            for(int y=0; y<getMapSizeY()-1; y++)
            {
                IntPoint p(x,y);
                plainMap->cell(p)=cell(p);
            }
        return plainMap;
    }

} // end namespace

#endif