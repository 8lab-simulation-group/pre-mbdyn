#include "Vec3d.h"
#include "Mat3x3d.h"
#include "Mat6x6d.h"

//シフト演算子オーバーロードの定義はクラス定義外で定義する
std::ostream & operator << (std::ostream &os, const Vec3d &v) {
    os << v[0] <<", "<< v[1] <<", " << v[2] ;
    return os;
}

std::ostream & operator << (std::ostream &os, const Mat3x3d &m) {
    os << m[0] <<" "<< m[1] <<" " << m[2] <<" "<< m[3] <<" "<< m[4] <<" " << m[5]<<" "<< m[6] <<" "<< m[7] <<" " << m[8];
    return os;
}

std::ostream & operator << (std::ostream &os, const Mat6x6d &m) {
    os      << m.get(0,0) <<" "<< m.get(0,1) <<" " << m.get(0,2) <<" "<< m.get(0,3) <<" "<< m.get(0,4) <<" " << m.get(0,5)<<std::endl
       <<" "<< m.get(1,0) <<" "<< m.get(1,1) <<" " << m.get(1,2) <<" "<< m.get(1,3) <<" "<< m.get(1,4) <<" " << m.get(1,5)<<std::endl
       <<" "<< m.get(2,0) <<" "<< m.get(2,1) <<" " << m.get(2,2) <<" "<< m.get(2,3) <<" "<< m.get(2,4) <<" " << m.get(2,5)<<std::endl
       <<" "<< m.get(3,0) <<" "<< m.get(3,1) <<" " << m.get(3,2) <<" "<< m.get(3,3) <<" "<< m.get(3,4) <<" " << m.get(3,5)<<std::endl
       <<" "<< m.get(4,0) <<" "<< m.get(4,1) <<" " << m.get(4,2) <<" "<< m.get(4,3) <<" "<< m.get(4,4) <<" " << m.get(4,5)<<std::endl
       <<" "<< m.get(5,0) <<" "<< m.get(5,1) <<" " << m.get(5,2) <<" "<< m.get(5,3) <<" "<< m.get(5,4) <<" " << m.get(5,5)<<std::endl;
    return os;
}

