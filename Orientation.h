#ifndef __ORIENTATION_H_INCLUDE__
#define __ORIRNTATION_H_INCLUDE__

#include "Vec3d.h"
#include "Mat3x3d.h"

// オイラー角やサブベクトルの入力から3x3の回転マトリクスを作る関数、この辺をmat3x3dの派生クラスで作って最終的にアップキャストするほうが合理的なのだろうか？

Mat3x3d make_orientation(const Vec3d &eular321, const std::string &name_eular=std::string("Eular321")) {
    if(name_eular == std::string("Eular321")) {
        
    }

}
#endif