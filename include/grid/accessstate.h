#ifndef ACCESSTATE_H
#define ACCESSTATE_H

namespace GMapping {
    /**
     * AccessibilityState，枚举变量，表示Cell的状态
     * Cell，地图中每个单元（栅格）的数据类型
     */
    enum AccessibilityState{Outside=0x0, Inside=0x1, Allocated=0x2};
};

#endif