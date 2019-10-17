#ifndef HARRAY2D_H
#define HARRAY2D_H
#include <set>
#include "array2d.h"
#include "utils/autoptr.h"
#include "utils/point.h"

namespace Gmapping {

    /**
     * HierarchicalArray2D，地图金字塔类，继承自二维数组Array2D类
     * 一种类似金字塔的形式组织地图内存（降低内存消耗），只是这个金字塔只有两层
     * 它本身是一个Array2D，其存储单元是一个指向Array2D对象的智能指针autoptr
     * 
     * 两层地图金字塔：第一层的元素为Patch，第二层的元素为Cell；
     * 首先把HierarchicalArray2D的元素为Patch(Array2D<cell>)，然后Patch的元素为Cell。
     * 一个Patch就是一个地图块，即一个(2^m_patchMagnitude, 2^m_patchMagnitude)大小的Array2D<Cell>的指针
     * 一个Patch的实际大小：m_patchSize = 1<<m_patchMagnitude，即 1*(2^m_patchMagnitude)
     * 一个Patch里面包含有2^m_patchMagnitude * 2^m_patchMagnitude 个Cell，即m_patchSize*m_patchSize个Cell
     * 
     * 数据成员：
     *      m_activeArea，存储地图中（有效区域）使用到的Cell的坐标集合（set）
     *      m_patchMagnitude，patch的大小等级，默认为5，即一个Patch默认大小为32*32
     *      m_patchSize，patch的实际大小，表示pathch里面包含的cell的个数，
     * 函数成员：
     *      =，重载运算符"="，实现两个地图的赋值运算
     *      resize，重新扩充地图的大小
     *      getPatchSize，获取Patch大小的等级
     *      getPatchMagnitude，获取Patch大小的等级，默认值为5
     *      patchIndexes，把原始xy坐标，转换为对应的Patch的坐标
     *      isAllocated，判断原始xy坐标处的Cell是否被分配了内存
     *      createPatch，创建一个Patch
     *      cellState，获取给定xy坐标处Cell的状态（重载函数）
     *      cell，获取给定xy坐标处的Cell（重载函数）
     *      setActiveArea，设置地图的有效区域（被激光扫过的区域）
     *      getActiveArea，获取地图的有效区域
     *      allocActiveArea，给地图的有效区域分配内存
     */
    template< class Cell >
    class HierarchicalArray2D: public Array2D< autoptr< Array2D<Cell> > > {

    public:
        typedef std::set< point<int>, pointcomparator<int> > PointSet;
        HierarchicalArray2D( int xsize, int ysize, int patchMagnitude=5 );
        HierarchicalArray2D( const HierarchicalArray2D& hg ); // 没看懂
        virtual ~HierarchicalArray2D(){}

        HierarchicalArray2D& operator = ( const HierarchicalArray2D& hg );

        void resize( int ixmin, int iymin, int ixmax, int iymax );  // 没看懂
        inline int getPatchSize() const { return m_patchMagnitude; }
        inline int getPatchMagnitude() const { return m_patchMagnitude; }
              
        inline patchIndexes( int x, int y ) const;
        inline patchIndexes( const IntPoint& p ) const { return patchIndexes(p.x, p.y); }

        inline bool isAllocated( int x, int y ) const;
        inline bool isAllocated( const IntPoint& p ) const { return isAllocated(p.x, p.y); }

        inline AccessibilityState cellState( int x, int y ) const;
        inline AccessibilityState cellState( const IntPoint& p ) const { return cellState(p.x, p.y); }

        inline const Cell& cell( int x, int y ) const;
        inline Cell& cell( int x, int y );
        inline const Cell& cell( const IntPoint& p ) const { return cell(p.x, p.y); }
        inline Cell& cell( const IntPoint& p ) { return cell(p.x, p.y); }

        inline void setActiveArea( const PointSet&, bool patchCoords=false );
        const PointSets& getActiveArea() const { return m_activeArea; }
        inline void allocActiveArea();
        
    protected:
        virtual Array2D<Cell>* createPatch( const IntPoint& p ) const;
        PointSet m_activeArea;
        int m_patchMagnitude;  
        int m_patchSize;

    };

    /**
     * 构造函数，构建地图金字塔
     * 在HierarchicalArray2D中把地图分为若干个Patch，其拆分的粒度由m_patchMagnitude来描述
     * 一个原始是(xsize, ysize)大小的Array2D数组，拆分为xsize/(2^patchMagnitude) * ysize/(2^patchMagnitude)个Patch
     * 这里只是构建了(xsize>>patchMagnitude, ysize>>patchMagnitude)大小的Array2D数组
     * 每个元素为指向Patch的智能指针，即autoptr< Array2D<Cell> >类型的指针，指向一个Patch元素
     * 一个Patch即为(2^patchMagnitude, 2^patchMagnitude)大小的Array2D数组，每个元素为Cell
     * 但这里并没有初始化一个Patch数组，而只是初始化了patchMagnitude供构建Patch数组使用
     */
    template< class Cell >
    HierarchicalArray2D<Cell>::HierarchicalArray2D( int xsize, int ysize, int patchMagnitude )
        : Array2D< autoptr< Array2D<Cell> > >::Array2D((xsize>>patchMagnitude), (ysize>>patchMagnitude))
    {
        // 
        m_patchMagnitude = patchMagnitude;
        m_patchSize = 1<<m_patchMagnitude; // 取巧使用左移/右移运算快一些，只能用于整型的数据
    }

    /**
     * 构造函数
     */ 
    template< class Cell >
    HierarchicalArray2D<Cell>::HierarchicalArray2D( const HierarchicalArray2D& hg ) 
        :Array2D< autoptr< Array2D<Cell> > >::Array2D( (hg.m_xsize>>hg.m_patchMagnitude), (hg.m_ysize>>hg.m_patchMagnitude) ) // added by cyrill: if you have a resize error, check this again
    {
        // 这两步应该和 hg.m_xsize>>hg.m_patchMagnitude，hg.m_ysize>>hg.m_patchMagnitude冲突了吧？
        // 个人认为只保留 this->m_xsize = hg.m_xsize; this->m_ysize = hg.m_ysize; 是正确的。
        // 因为参考下面重载运算符"="，可以发现this->m_xsize = hg.m_xsize，而不是this->m_xsize = hg.m_xsize >> hg.m_patchMagnitude
        this->m_xsize = hg.m_xsize;     
        this->m_ysize = hg.m_ysize;     
        this->m_cells = new autoptr< Array2D<Cell> >* [this->m_xsize]; 
        for ( int x=0; x<this->m_xsize; x++ ) {
            this->m_cells[x] = new autoptr< Array2D<Cell> >[this->m_ysize];
            for ( int y=0; y<this->m_ysize; y++ ) {
                this->m_cells[x][y] = hg.m_cells[x][y];
            }
        }
        this->m_patchMagnitude = hg.m_patchMagnitude;
        this->m_patchSize = hg.m_patchSize;
    }

    /**
     * 重载运算符"="，实现两个地图的赋值运算
     * 如果复制的两个地图的大小不一样 则需要把目前的地图删除，然后重新复制为新的地图
     */ 
    template< class Cell >
    HierarchicalArray2D& HierarchicalArray2D<Cell>::operator = ( const HierarchicalArray2D& hg ) {
        // 删除当前地图，并且重新分配内存
        if ( this->m_xsize!=hg.m_xsize || this->m_ysize!=hg.m_ysize  ) {
            for ( int i=0; i<this->m_xsize; i++ ) {
                delete [] this->m_cells[i];
            }
            delete [] this->m_celss;

            this->m_xsize = hg.m_xsize;
            this->m_ysize = hg.m_ysize;
            this->m_cells = new autoptr< Array2D<Cell> >* [this->m_xsize];
            for ( int i=0; i<this->m_xsize; i++ )
                this->m_cells[i] = new autoptr< Array2D<Cell> >[this->m_ysize];
        }
        // 赋值为新的地图
        for ( int x=0; x<this->m_xsize; x++ ) {
            for ( int y=0; y<this->m_ysize; y++ ) {
                this->m_cells[x][y] = hg.m_cells[x][y];
            }
        }

        m_activeArea.clear();
        m_patchMagnitude=hg.m_patchMagnitude;
        m_patchSize=hg.m_patchSize;
        return *this;
    }

    /**
     * 重新扩充地图的大小
     */ 
    template< class Cell >
    void HierarchicalArray2D<Cell>::resize( int xmin, int ymin, int xmax, int ymax ) {
        int xsize=xmax-xmin;
        int ysize=ymax-ymin;
        //分配一个xsize*ysize大小的patch的内存
        autoptr< Array2D<Cell> > ** newcells=new autoptr< Array2D<Cell> > *[xsize];
        for (int x=0; x<xsize; x++)
        {
            newcells[x]=new autoptr< Array2D<Cell> >[ysize];
            for (int y=0; y<ysize; y++)
            {
                newcells[x][y]=autoptr< Array2D<Cell> >(0);
            }
        }
        
        //把原来地图中存在的数据 拷贝到新分配的地图中 这样就完成了地图扩充
        int dx= xmin < 0 ? 0 : xmin;
        int dy= ymin < 0 ? 0 : ymin;
        int Dx=xmax<this->m_xsize?xmax:this->m_xsize;
        int Dy=ymax<this->m_ysize?ymax:this->m_ysize;
        
        for (int x=dx; x<Dx; x++)
        {
            for (int y=dy; y<Dy; y++)
            {
                newcells[x-xmin][y-ymin]=this->m_cells[x][y];
            }
            delete [] this->m_cells[x];
        }
        delete [] this->m_cells;
        this->m_cells=newcells;
        this->m_xsize=xsize;
        this->m_ysize=ysize; 
    }

    /**
     * 把原始xy坐标，转换为对应的Patch的坐标
     * 相当于原始坐标/m_patchMagnitude
     */
    template< class Cell >
    inline HierarchicalArray2D<Cell>::patchIndexes( int x, int y ) const {
        if ( x>=0 && y>=0 )
            return IntPoint( x>>m_patchMagnitude, y>>m_patchMagnitude);
        return IntPoint(-1, -1);
    }

    /**
     * 判断xy坐标处的Cell是否被分配了内存
     * 要访问某个具体的cell，首先转换到patch坐标系，再从对应的patch中访问对应的cell元素
     */
    template< class Cell >
    inline bool HierarchicalArray2D<Cell>::isAllocated( int x, int y ) const {
        IntPoint c = patchIndexes(x, y);
        autoptr< Array2D<Cell> >& ptr = this->m_cells[c.x][c.y];
        // 实际上是Patch是否被分配了内存，Patch被分配了内存，元素Cell自然也拥有内存
        return (ptr != 0);
    }

    /**
     * 创建一个Patch
     * 并没有初始化一个Patch数组，但初始化了patchMagnitude供这里构建Patch数组使用
     * 一个patch表示一个Array2D数组，大小为2^m_patchMagnitude * 2^m_patchMagnitude
     */
    template< class Cell >
    Array2D<Cell>* HierarchicalArray2D<Cell>::createPatch( const IntPoint& p ) const {
        // 这里形参 p 都没用到
        return new Array2D<Cell>(1<<m_patchMagnitude, 1<<m_patchMagnitude);
    }

    /** 
     * 获取给定xy坐标处Cell的状态
     * 要访问某个具体的cell，首先转换到patch坐标系，再从对应的patch中访问对应的cell元素
     */ 
    inline AccessibilityState cellState( int x, int y ) const {
        if ( this->isInside( patchIndexes(x, y) ) ) {
            if ( isAllocated(x, y) )
                return (AccessibilityState)((int)Inside|(int)Allocated);
            else
                return Inside;
        }
        return Outside;
    }

    /**
     * 获取给定xy坐标处的Cell（已分配内存了的）
     * 要访问某个具体的cell，首先转换到patch坐标系，再从对应的patch中访问对应的cell元素
     */
    template< class Cell >
    inline const Cell& HierarchicalArray2D<Cell>::cell( int x, int y ) const {
        assert( isAllocated(x, y) );
        IntPoint c = patchIndexes(x, y);
        const autoptr< Array2D<Cell> >& ptr = this->m_cells[c.x][c.y];
        return (*ptr).cell(IntPoint(x-(c.x<<m_patchMagnitude),y-(c.y<<m_patchMagnitude)));
    }

    /**
     * 获取给定xy坐标处的Cell（未分配内存）
     * 要访问某个具体的cell，首先转换到patch坐标系，再从对应的patch中访问对应的cell元素
     */
    template< class Cell >
    inline Cell& HierarchicalArray2D<Cell>::cell( int x, int y ) {
        IntPoint c = patchIndexes(x, y);
        assert(this->isInside(c.x, c.y));
        // 若没有对cell所在的Patch分配内存
        if ( !this->m_cells[c.x][c.y] ) {
            Array2D<Cell>* patch = createPatch(IntPoint(x, y));
            this->m_cells[c.x][c.y] = autoptr< Array2D<Cell> >(patch);
        }
        autoptr< Array2D<Cell> >& ptr = this->m_cells[c.x][c.y]
        return (*ptr).cell(IntPoint(x-(c.x<<m_patchMagnitude),y-(c.y<<m_patchMagnitude)));
    }    

    /**
     * 设置地图的有效区域（被激光扫过的区域） 
     * 对于HierarchicalArray2D来说，考虑的尺度都是patch。
     * 因此进行扩充地图或者分配内存的时候，每次也都是以patch为单位的。
     * aa，给定的一组坐标点的集合（set）
     * patchCoords，指示当前的PointSet的坐标是否为对应的Patch坐标，若不是应转换到pathch坐标后再插入
     */
    template< class Cell >
    inline void HierarchicalArray2D<Cell>::setActiveArea( const typename HierarchicalArray2D<Cell>::PointSet& aa, bool patchCoords ) {
        m_activeArea.clear();
        // 这里只是把点存储到对应的集合m_activeArea
        // 并没有为对应的Patch进行内存的分配，真正的内存分配在后面的allocActiveArea()函数
        for ( PointSet::const_iterator it=aa.begin(); it!=aa.end(); ++it ) {
            IntPoint p;
            if ( patchCoords )
                p = *it;
            else
                p = patchIndexes(*it);
            m_activeArea.insert(p);
        }
    }

    /**
     * 给地图的有效区域分配内存
     * 
     */ 
    template< class Cell >
    inline void HierarchicalArray2D<Cell>::allocActiveArea() {
        for ( PointSet::const_iterator it=m_activeArea.begin(); it!=m_activeArea.end(); ++it ) {
            const autoptr< Array2D<Cell> >& ptr = this->m_cells[it->x][it->y];
            Array2D<Cell>* patch = 0;
            if ( !ptr ) {
                // 如果对应的patch没有被分配内存 则进行内存分配
                patch = createPatch(*it);
            } else {
                //如果已经分配了还是赋原值
                patch = new Array2D<Cell>(*ptr);
            }
            this->m_cells[it->x][it->y] = autoptr< Array2D<Cell> >(patch);
        }
    }

} // end namespace

#endif