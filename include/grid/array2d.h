#ifndef ARRAY2D_H
#define ARRAY2D_H
#include <iostream>
#include "accessstate.h"
#include "assert.h"
#include "utils/point.h"

// 用于输出调试信息
#ifndef __PRETTY_FUNCTION__
#define __FUNCDNAME__
#endif

namespace Gmapping {

    /**
     * Array2D，二维数组类，用作栅格地图存储类型HierarchicalArray2D的基类
     * 栅格地图，由世界坐标系中xy坐标值映射得到，在栅格地图对应位置上可以用空闲或者被占用来标记
     * 很自然的，我们会想到使用一个矩阵或者说是二维数组的形式来描述地图。
     * 
     * 模板参数：
     * Cell，地图中每个单元（栅格）的数据类型
     * Gmapping中实例为PointAccumulator类型
     * 对于DoubleArray2D来说，实例是double类型
     * 
     * 数据成员：
     *      m_cells，二维数组指针
     *      m_xsize，二维数组的大小（栅格地图的长度）
     *      m_ysize，二维数组的大小（栅格地图的宽度）
     * 
     * 函数成员：
     *      =，重载运算符，两个Array2D之间赋值
     *      clear，清除二维数组中所有数据
     *      resize，重定义二维数组的大小
     *      isInside，判断给定坐标是否在二维数组内（x:0~m_xsize，y:0~m_ysize，重载函数）
     *      cell，获取给定xy坐标处的Cell（重载函数）
     *      cellState，获取给定xy坐标处Cell的状态（重载函数）
     *      getPatchSize，获取Patch的大小，参考harray2d.hpp文件
     *      getPatchMagnitude，获取Patch大小的等级，参考harray2d.hpp文件
     *      getXSize，获取二维数组的长度
     *      getYSize，获取二维数组的宽度
     *      cells，获取二维数组
     */
    template< class Cell, const bool debug=false >
    class Array2D {
    public:
        Cell** m_cells;

        Array2D( int xsize=0, int ysize=0 );
        Array2D( const Array2D<Cell, debug>& )
        ~Array2D();

        Array2D& operator=( const Array2D& );
        
        void clear();
        void resize( int xmin, int ymin, int xmax, int ymax );
        inline bool isInside( int x, int y ) const;
        inline bool isInside(const IntPoint& p) const { return isInside(p.x, p.y);}
        inline const Cell& cell( int x, int y ) const;
        inline Cell& cell( int x, int y );
        inline const Cell& cell(const IntPoint& p) const {return cell(p.x,p.y);}
		inline Cell& cell(const IntPoint& p) {return cell(p.x,p.y);}
        inline AccessibilityState cellState( int x, int y ) const {
            return (AccessibilityState) (isInside(x,y)?(Inside|Allocated):Outside);
        }	
		inline AccessibilityState cellState(const IntPoint& p) const { return cellState(p.x, p.y);}
        inline int getPatchSize() const { return 0; }  // 已经是地图金字塔底层了，无Patch可言
        inline int getPatchMagnitude() const { return 0; }
        inline int getXSize() const { return m_xsize; }
        inline int getYSize() const { return m_ysize; }
        inline Cell** cells() { return m_cells; }

    protected:
        int m_xsize, m_ysize;
    };

    /**
     * 构造函数，给二维数组（m_cells）分配存储空间
     */
    template< class Cell, const bool debug >
    Array2D<Cell, debug>::Array2D( int xsize, int ysize ) {
        m_xsize = xsize;
        m_ysize = ysize;
        if ( m_xsize>0 && m_ysize>0 ) {
            m_cells = new Cell*[m_xsize];
            for (int i=0; i<m_xsize; i++) {
                m_cells[i] = new Cell[m_ysize];
            } else {
                m_xsize = m_ysize = 0;
                m_cells = 0;
            }
            if (debug) {
                std::cerr << __PRETTY_FUNCTION__ << std::endl;
                std::cerr << "m_xsize= " << m_xsize<< std::endl;
                std::cerr << "m_ysize= " << m_ysize<< std::endl;
            }
        }
    }

    /**
     * 构造函数，给定一个Array2D对象
     */
    template< class Cell, const bool debug >
    Array2D<Cell, debug>::Array2D( const Array2D<Cell, debug>& g ) {
        m_xsize = g.m_xsize;
        m_ysize = g.m_ysize;
        m_cells = new Cell*[m_xsize];
        for ( int x=0; x<m_xsize; x++ ) {
            m_cells[x] = new Cell[m_ysize];
            for ( int y=0; y<m_ysize; y++ ) {
                m_cells[x][y] = g.m_cells[x][y];
            }
        }
        if (debug) {
            std::cerr << __PRETTY_FUNCTION__ << std::endl;
            std::cerr << "m_xsize= " << m_xsize<< std::endl;
            std::cerr << "m_ysize= " << m_ysize<< std::endl;
        }
    }

    /**
     * 析构函数，释放申请的所有的内存空间
     */
    template< class Cell, const bool debug >
    Array2D<Cell, debug>::~Array2D() {
        if ( debug ) {
            std::cerr << __PRETTY_FUNCTION__ << std::endl;
            std::cerr << "m_xsize= " << m_xsize<< std::endl;
            std::cerr << "m_ysize= " << m_ysize<< std::endl;
        }
        for ( int i=0; i<m_xsize; i++ ) {
            delete [] m_cells[i];
            m_cells[i] = 0
        }
        delete [] m_cells;
        m_cells = 0;
    }

    /**
     * 重载运算符“=”，两个Array2D之间赋值
     */
    template< class Cell, const bool debug >
    Array2D<Cell, debug>& Array2D<Cell, debug>::operator = ( const Array2D<Cell, debug>& g ) {
        if ( debug || m_xsize!=g.m_xsize || m_ysize!=g.m_ysize ) {
            for (int i=0; i<m_xsize; i++)
                delete [] m_cells[i];
            delete [] m_cells;
            m_xsize=g.m_xsize;
            m_ysize=g.m_ysize;
            m_cells=new Cell*[m_xsize];
            for (int i=0; i<m_xsize; i++)
                m_cells[i]=new Cell[m_ysize];
        }
        for ( int x=0; x<m_xsize; x++ )
            for ( int y=0; y<m_ysize; y++ )
                m_cells[x][y]=g.m_cells[x][y];       
        if ( debug ) {
            std::cerr << __PRETTY_FUNCTION__ << std::endl;
            std::cerr << "m_xsize= " << m_xsize<< std::endl;
            std::cerr << "m_ysize= " << m_ysize<< std::endl;
        }
        return *this;
    }

    /**
     * 清除函数，清除二维数组中的所有数据
     */
    template< class Cell, const bool debug >
    void Array2D<Cell, debug>::clear() {
        if ( debug ) {
            std::cerr << __PRETTY_FUNCTION__ << std::endl;
            std::cerr << "m_xsize= " << m_xsize<< std::endl;
            std::cerr << "m_ysize= " << m_ysize<< std::endl;
        }
        for ( int i=0; i<m_xsize; i++ ) {
            delete [] m_cells[i];
            m_cells[i] = 0
        }
        delete [] m_cells;
        m_cells = 0;
        m_xsize = 0;
        m_ysize = 0;
    }

    /**
     * 重新定义二维数组的大小
     * 这个算法是不是有点错误？？？
     */
    template< class Cell, const bool debug >
    void Array2D<Cell, debug>::resize( int xmin, int ymin, int xmax, int ymax ){
        int xsize = xmax - xmin;
        int ysize = ymax - ymin;
        Cell** newcells = new Cell* [xsize];
        for ( int x=0; x<xsize; x++ ) {
            newcells[x] = new Cell[ysize];
        }
        int dx = xmin < 0 ? 0 : xmin;
        int dy = ymin < 0 ? 0 : ymin;
        int Dx = xmax < this->m_xsize ? xmax : this->m_xsize;
        int Dy = ymax < this->m_ysize ? ymax : this->m_ysize;
        for ( int x=dx; x<Dx; x++ ) {
            for ( int y=dy; y<Dy; y++ ) {
                newcells[x-xmin][y-ymin] = this->m_cells[x][y];
            }
            delete [] this->m_cells[x]
        }
        delete [] this->m_cells;
        this->m_cells = newcells;
        this->m_xsize = xsize;
        this->m_ysize = ysize;
    }

    /**
     * 判断一个点是否在二维数组内
     */
    template< class Cell, const bool debug >
    inline bool Array2D<Cell, debug>::isInside( int x, int y ) const {
        return x>=0 && y>=0 && x<m_xsize && y<m_ysize;
    }

    /**
     * 获取给定xy坐标处的Cell
     */
    template< class Cell, const bool debug >
    inline const Cell& Array2D<Cell, debug>::cell( int x, int y ) const {
        assert(isInside(x, y));
        return m_cells[x][y];
    }

    /**
     * 获取给定xy坐标处的Cell（重载函数）
     */
    template< class Cell, const bool debug >
    inline Cell& Array2D<Cell, debug>::cell( int x, int y ) {
        assert(isInside(x, y));
        return m_cells[x][y];
    }


} // end namespace

#endif