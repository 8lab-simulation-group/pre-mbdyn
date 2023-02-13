#include "InputData.h"
#include "Node.h"
#include "RigidBody.h"
#include "Joint.h"

int main(){

    Frame ground(0.,zero3,eye3x3,zero3,zero3);

    Vec3d offset(1.,2.,3.);
    Vec3d e1(0.,0.,1);
    Vec3d e3(1.,0.,0.);

    ReferenceFrame base(1.,ground, Frame(0.,offset, e1,e3,zero3,zero3));


    double theta = M_PI/3;

    e1 = Vec3d(std::cos(theta),std::sin(theta),0);
    e3 = Vec3d(0.,0.,1.);

    Vec3d euler123(0.,0.,theta);

    ReferenceFrame reffrm(2,base, Frame(1.,zero3,e1,e3,zero3,zero3));

    Node node(1, reffrm, 1);

    node.print_node();

    Frame dummy_offset(0.,Vec3d(1.,2.,3), Vec3d(0.,1.,0.), Vec3d(1.,0.,0.),zero3,zero3);
    
    DummyNode dummy1(2, node, dummy_offset,1);
    DummyNode dummy2(3, node, offset_null,1);

    dummy1.print_node();
    dummy2.print_node();
    return 0;

// 検証手順
// 使っているクラス Vec3 Frame,ReferenceFrame <= 宣言しているヘッダファイルをインクルードすると使える
// このヘッダファイル内では、クラスの宣言だけを行っているので、定義、実装部分がない
// クラスの定義、実装部分は、vec3d.ccとかの対応する.ccコードに書かれているので、それら全部一緒にコンパイルしないといけない
// g++ -c Vec3d.cc Frame.cc ReferenceFrame.cc test.cc でコンパイルを行った
// この-cの意味は、参照関係を無視して、とりあえずコンパイルして.oオブジェクトをつくる感じ
// g++ *.o -o out でワイルドカード*で指定した全部の.oオブジェクトを認識して、-oで名前をつけて、最終的な
// コンパイルをして、プログラムの実行形態を作っている。
// つくられた　.out　の実行形態を実行する
// ./out

}