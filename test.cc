#include "InputData.h"
#include "Node.h"
#include "RigidBody.h"
#include "Joint.h"

std::vector<double> 
Interplation(const std::vector<double> &xval , const std::vector<double> &xarray, const std::vector<double> &yarray);

int main(){

    std::string input_file = "test/pre-MBDyn.ipt";

    InputData inputdata(input_file);

    std::vector<double> BlFract = inputdata.get_vector("BlFract");
    

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

std::vector<double> 
Interplation(const std::vector<double> &xval , const std::vector<double> &xarray, const std::vector<double> &yarray) 
{
    if(xarray.size() != yarray.size() ){
        std::cerr << "The size of xarray and yarray must match in interplation function" << std::endl;
    }

    std::vector<double> result(xval.size());

    for (int i=0; i < xval.size() ; i++) {

        // check where you should refere to the value in xarray and yarray
        int range = 0;        
        while( xval[i] > xarray[range]) {
            range++;
            
            if(range==xarray.size()){
                break;
            }

        }
        if(range==0){
            result[i] = yarray[range];
        }
        else{
            
            result[i] = (yarray[range] - yarray[range-1]) * (xval[i] - xarray[range-1])/(xarray[range] - xarray[range-1]) + yarray[range-1]; 
        }
        if(range==xarray.size()){
            result[i] = yarray[range];
        }

    }
    return result;
}