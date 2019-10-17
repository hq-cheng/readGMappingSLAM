#include "scanmatcher/smmap.h"

namespace Gmapping {

    /**
     * 利用静态指针unknown_ptr（默认为0）获取一个Cell（栅格）
     * 有点像链式结构？？？
     */
    const PointAccumulator& PointAccumulator::Unknown() {
        if ( !unknown_ptr )
            unknown_ptr = new PointAccumulator;
        return *unknown_ptr;
    }

    PointAccumulator* PointAccumulator::unknown_ptr = 0;

} // end namespace