#include "Blade.h"
#include <cmath>
#include <math.h>

Blade::Blade(int own_num,int label, const ReferenceFrame &Bladebase, InputData *ID) 
: own_bld_num(own_num),blade_label(label), bld_base_reference1(Bladebase), inputdata(ID) // const メンバ変数を引数で初期化
{
    num_bld = inputdata->get_value("NumBl");
    num = inputdata->get_value("BldNodes");
    //num_reference = (num *4 +2) * num_bld;
    // _P,CM,EA_CENTER,Aeroの4個の座標系が必要なので以下のようになっている。+2；PitchPlate,TipBrake
    num_reference = num *4 +2;
    num_nodes = num +2;
    num_rigidbodies = num_nodes;
    num_deformable_joints = num_nodes -1;
    num_total_joints = num_nodes -1;


    //vector変数を初期化
    references.resize(num_reference);
    nodes.resize(num_nodes);
    rigidbodies.resize(num_rigidbodies);
    deformable_joints.resize(num_deformable_joints);
    total_joints.resize(num_total_joints);

    InterpBMass.resize(num);
    InterpEAStff.resize(num);
    InterpFlpStff.resize(num);
    InterpEdgStff.resize(num);
    InterpGJStff.resize(num);
    InterpdgIner.resize(num);
    InterpFlpIner.resize(num);
    InterpAeroCent.resize(num);
    InterpEdgcgOf.resize(num);

    // BMass, StiffBF(FlpStff), StiffBEA(EAStff), StiffBE(EdgStff), StiffBGJ(GJStff), FlpIner, EdgInerの線形補間
    // x:BlFract, y:線形補間したい変数
    
    for(int i=0; i<num; i++) {
      BldFlexL = inputdata->get_value("TipRad") - inputdata->get_value("HubRad");
      double XVal = (inputdata->get_value("RNodes", i+1) - inputdata->get_value("HubRad"))/BldFlexL;
      double XAray1;
      double XAray2;
      double MAray1;
      double MAray2;
      double EAAray1;
      double EAAray2;
      double FlAray1;
      double FlAray2;
      double EdgAray1;
      double EdgAray2;
      double GJAray1;
      double GJAray2;
      double EdgInerAray1;
      double EdgInerAray2;
      double FlpInerAray1;
      double FlpInerAray2;
      double AeroCentAray1;
      double AeroCentAray2;
      double EdgcgOf1;
      double EdgcgOf2;

      if(i==0) {
        XAray1 = inputdata->get_value("BlFract",3);
        XAray2 = inputdata->get_value("BlFract",4);
        MAray1 = inputdata->get_value("BMassDen",3);
        MAray2 = inputdata->get_value("BMassDen",4);
        EAAray1 = inputdata->get_value("EAStff",3);
        EAAray2 = inputdata->get_value("EAStff",4);
        FlAray1 = inputdata->get_value("FlpStff",3);
        FlAray2 = inputdata->get_value("FlpStff",4);
        EdgAray1 = inputdata->get_value("EdgStff",3);
        EdgAray2 = inputdata->get_value("EdgStff",4);
        GJAray1 = inputdata->get_value("GJStff",3);
        GJAray2 = inputdata->get_value("GJStff",4);
        EdgInerAray1 = inputdata->get_value("EdgIner",3);
        EdgInerAray2 = inputdata->get_value("EdgIner",4);
        FlpInerAray1 = inputdata->get_value("FlpIner",3);
        FlpInerAray2 = inputdata->get_value("FlpIner",4); 
        AeroCentAray1 = inputdata->get_value("AeroCent",3);
        AeroCentAray2 = inputdata->get_value("AeroCent",4); 
        EdgcgOf1 = inputdata->get_value("EdgcgOf",3);
        EdgcgOf2 = inputdata->get_value("EdgcgOf",4);              
      }else if(i==1) {
        XAray1 = inputdata->get_value("BlFract",5);
        XAray2 = inputdata->get_value("BlFract",6);
        MAray1 = inputdata->get_value("BMassDen",5);
        MAray2 = inputdata->get_value("BMassDen",6);
        EAAray1 = inputdata->get_value("EAStff",5);
        EAAray2 = inputdata->get_value("EAStff",6);
        FlAray1 = inputdata->get_value("FlpStff",5);
        FlAray2 = inputdata->get_value("FlpStff",6);
        EdgAray1 = inputdata->get_value("EdgStff",5);
        EdgAray2 = inputdata->get_value("EdgStff",6);
        GJAray1 = inputdata->get_value("GJStff",5);
        GJAray2 = inputdata->get_value("GJStff",6); 
        EdgInerAray1 = inputdata->get_value("EdgIner",5);
        EdgInerAray2 = inputdata->get_value("EdgIner",6);
        FlpInerAray1 = inputdata->get_value("FlpIner",5);
        FlpInerAray2 = inputdata->get_value("FlpIner",6);  
        AeroCentAray1 = inputdata->get_value("AeroCent",5);
        AeroCentAray2 = inputdata->get_value("AeroCent",6);
        EdgcgOf1 = inputdata->get_value("EdgcgOf",5);
        EdgcgOf2 = inputdata->get_value("EdgcgOf",6);               
      }else if(i==2) {
        XAray1 = inputdata->get_value("BlFract",8);
        XAray2 = inputdata->get_value("BlFract",9);
        MAray1 = inputdata->get_value("BMassDen",8);
        MAray2 = inputdata->get_value("BMassDen",9);
        EAAray1 = inputdata->get_value("EAStff",8);
        EAAray2 = inputdata->get_value("EAStff",9);
        FlAray1 = inputdata->get_value("FlpStff",8);
        FlAray2 = inputdata->get_value("FlpStff",9);
        EdgAray1 = inputdata->get_value("EdgStff",8);
        EdgAray2 = inputdata->get_value("EdgStff",9);
        GJAray1 = inputdata->get_value("GJStff",8);
        GJAray2 = inputdata->get_value("GJStff",9); 
        EdgInerAray1 = inputdata->get_value("EdgIner",8);
        EdgInerAray2 = inputdata->get_value("EdgIner",9);
        FlpInerAray1 = inputdata->get_value("FlpIner",8);
        FlpInerAray2 = inputdata->get_value("FlpIner",9); 
        AeroCentAray1 = inputdata->get_value("AeroCent",8);
        AeroCentAray2 = inputdata->get_value("AeroCent",9);
        EdgcgOf1 = inputdata->get_value("EdgcgOf",8);
        EdgcgOf2 = inputdata->get_value("EdgcgOf",9);        
      }else if(i==3) {
        XAray1 = inputdata->get_value("BlFract",12);
        XAray2 = inputdata->get_value("BlFract",13);
        MAray1 = inputdata->get_value("BMassDen",12);
        MAray2 = inputdata->get_value("BMassDen",13);
        EAAray1 = inputdata->get_value("EAStff",12);
        EAAray2 = inputdata->get_value("EAStff",13);
        FlAray1 = inputdata->get_value("FlpStff",12);
        FlAray2 = inputdata->get_value("FlpStff",13);
        EdgAray1 = inputdata->get_value("EdgStff",12);
        EdgAray2 = inputdata->get_value("EdgStff",13);
        GJAray1 = inputdata->get_value("GJStff",12);
        GJAray2 = inputdata->get_value("GJStff",13); 
        EdgInerAray1 = inputdata->get_value("EdgIner",12);
        EdgInerAray2 = inputdata->get_value("EdgIner",13);
        FlpInerAray1 = inputdata->get_value("FlpIner",12);
        FlpInerAray2 = inputdata->get_value("FlpIner",13); 
        AeroCentAray1 = inputdata->get_value("AeroCent",12);
        AeroCentAray2 = inputdata->get_value("AeroCent",13); 
        EdgcgOf1 = inputdata->get_value("EdgcgOf",12);
        EdgcgOf2 = inputdata->get_value("EdgcgOf",13);      
      }else if(i==4) {
        XAray1 = inputdata->get_value("BlFract",16);
        XAray2 = inputdata->get_value("BlFract",17);
        MAray1 = inputdata->get_value("BMassDen",16);
        MAray2 = inputdata->get_value("BMassDen",17);
        EAAray1 = inputdata->get_value("EAStff",16);
        EAAray2 = inputdata->get_value("EAStff",17);
        FlAray1 = inputdata->get_value("FlpStff",16);
        FlAray2 = inputdata->get_value("FlpStff",17);
        EdgAray1 = inputdata->get_value("EdgStff",16);
        EdgAray2 = inputdata->get_value("EdgStff",17);
        GJAray1 = inputdata->get_value("GJStff",16);
        GJAray2 = inputdata->get_value("GJStff",17); 
        EdgInerAray1 = inputdata->get_value("EdgIner",16);
        EdgInerAray2 = inputdata->get_value("EdgIner",17);
        FlpInerAray1 = inputdata->get_value("FlpIner",16);
        FlpInerAray2 = inputdata->get_value("FlpIner",17);
        AeroCentAray1 = inputdata->get_value("AeroCent",16);
        AeroCentAray2 = inputdata->get_value("AeroCent",17);
        EdgcgOf1 = inputdata->get_value("EdgcgOf",16);
        EdgcgOf2 = inputdata->get_value("EdgcgOf",17);         
      }else if(i>4 && i<15) {
        XAray1 = inputdata->get_value("BlFract",2*(i-4)+17);
        XAray2 = inputdata->get_value("BlFract",2*(i-4)+18);
        MAray1 = inputdata->get_value("BMassDen",2*(i-4)+17);
        MAray2 = inputdata->get_value("BMassDen",2*(i-4)+18);
        EAAray1 = inputdata->get_value("EAStff",2*(i-4)+17);
        EAAray2 = inputdata->get_value("EAStff",2*(i-4)+18);
        FlAray1 = inputdata->get_value("FlpStff",2*(i-4)+17);
        FlAray2 = inputdata->get_value("FlpStff",2*(i-4)+18);
        EdgAray1 = inputdata->get_value("EdgStff",2*(i-4)+17);
        EdgAray2 = inputdata->get_value("EdgStff",2*(i-4)+18);
        GJAray1 = inputdata->get_value("GJStff",2*(i-4)+17);
        GJAray2 = inputdata->get_value("GJStff",2*(i-4)+18); 
        EdgInerAray1 = inputdata->get_value("EdgIner",2*(i-4)+17);
        EdgInerAray2 = inputdata->get_value("EdgIner",2*(i-4)+18);
        FlpInerAray1 = inputdata->get_value("FlpIner",2*(i-4)+17);
        FlpInerAray2 = inputdata->get_value("FlpIner",2*(i-4)+18);
        AeroCentAray1 = inputdata->get_value("AeroCent",2*(i-4)+17);
        AeroCentAray2 = inputdata->get_value("AeroCent",2*(i-4)+18);
        EdgcgOf1 = inputdata->get_value("EdgcgOf",2*(i-4)+17);
        EdgcgOf2 = inputdata->get_value("EdgcgOf",2*(i-4)+18);         
      }else if(i==15) {
        XAray1 = inputdata->get_value("BlFract",40);
        XAray2 = inputdata->get_value("BlFract",41);
        MAray1 = inputdata->get_value("BMassDen",40);
        MAray2 = inputdata->get_value("BMassDen",41);
        EAAray1 = inputdata->get_value("EAStff",40);
        EAAray2 = inputdata->get_value("EAStff",41);
        FlAray1 = inputdata->get_value("FlpStff",40);
        FlAray2 = inputdata->get_value("FlpStff",41);
        EdgAray1 = inputdata->get_value("EdgStff",40);
        EdgAray2 = inputdata->get_value("EdgStff",41);
        GJAray1 = inputdata->get_value("GJStff",40);
        GJAray2 = inputdata->get_value("GJStff",41);   
        EdgInerAray1 = inputdata->get_value("EdgIner",40);
        EdgInerAray2 = inputdata->get_value("EdgIner",41);
        FlpInerAray1 = inputdata->get_value("FlpIner",40);
        FlpInerAray2 = inputdata->get_value("FlpIner",41);
        AeroCentAray1 = inputdata->get_value("AeroCent",40);
        AeroCentAray2 = inputdata->get_value("AeroCent",41);
        EdgcgOf1 = inputdata->get_value("EdgcgOf",40);
        EdgcgOf2 = inputdata->get_value("EdgcgOf",41);               
      }else {
        XAray1 = inputdata->get_value("BlFract",45);
        XAray2 = inputdata->get_value("BlFract",46);
        MAray1 = inputdata->get_value("BMassDen",45);
        MAray2 = inputdata->get_value("BMassDen",46);
        EAAray1 = inputdata->get_value("EAStff",45);
        EAAray2 = inputdata->get_value("EAStff",46);
        FlAray1 = inputdata->get_value("FlpStff",45);
        FlAray2 = inputdata->get_value("FlpStff",46);
        EdgAray1 = inputdata->get_value("EdgStff",45);
        EdgAray2 = inputdata->get_value("EdgStff",46);
        GJAray1 = inputdata->get_value("GJStff",45);
        GJAray2 = inputdata->get_value("GJStff",46);
        EdgInerAray1 = inputdata->get_value("EdgIner",45);
        EdgInerAray2 = inputdata->get_value("EdgIner",46);
        FlpInerAray1 = inputdata->get_value("FlpIner",45);
        FlpInerAray2 = inputdata->get_value("FlpIner",46); 
        AeroCentAray1 = inputdata->get_value("AeroCent",45);
        AeroCentAray2 = inputdata->get_value("AeroCent",46); 
        EdgcgOf1 = inputdata->get_value("EdgcgOf",45);
        EdgcgOf2 = inputdata->get_value("EdgcgOf",46);
      }
      InterpBMass[i] = ((MAray2-MAray1) * (XVal-XAray1)/(XAray2-XAray1) + MAray1) * inputdata->get_value("AdjBlMs");
      InterpEAStff[i] = (EAAray2-EAAray1) * (XVal-XAray1)/(XAray2-XAray1) + EAAray1;
      InterpFlpStff[i] = (FlAray2-FlAray1) * (XVal-XAray1)/(XAray2-XAray1) + FlAray1;
      InterpEdgStff[i] = (EdgAray2-EdgAray1) * (XVal-XAray1)/(XAray2-XAray1) + EdgAray1;
      InterpGJStff[i] = (GJAray2-GJAray1) * (XVal-XAray1)/(XAray2-XAray1) + GJAray1;
      InterpdgIner[i] = (EdgInerAray2-EdgInerAray1) * (XVal-XAray1)/(XAray2-XAray1) + EdgInerAray1;
      InterpFlpIner[i] = (FlpInerAray2-FlpInerAray1) * (XVal-XAray1)/(XAray2-XAray1) + FlpInerAray1; 
      InterpEdgcgOf[i] = (EdgcgOf2-EdgcgOf1) * (XVal-XAray1)/(XAray2-XAray1) + EdgcgOf1;
      InterpEdgcgOf[i] = (EdgcgOf2-EdgcgOf1) * (XVal-XAray1)/(XAray2-XAray1) + EdgcgOf1;  

    }

    set_reference();
    set_nodes();
    set_rigidbodies();
    set_deformablejoint();
    set_total_joint();
}

void
Blade::set_reference() {

    std::vector<double> RNodes;
    std::vector<double> Twist;

    RNodes.resize(num);
    Twist.resize(num);

    for(int i=0; i<num_nodes; i++) {
        //int curr_ref_label = blade_label + i + 1;

        // 相対速度はなし
        Vec3d velocity_offset = zero3;
        Vec3d angular_velocity_offset = zero3;        

        // nodeのbld_base_reference1に対する相対座標を計算
        if(i==0) {
          references[i] = ReferenceFrame(blade_label+i+1,bld_base_reference1,offset_null);
        }
        // _P, Aero_M, CM_M, EA_M
        else if(i>0 && i<num+1) {

        Vec3d positon_offset = Vec3d(0.,0.,inputdata->get_value("RNodes",i) - inputdata->get_value("HubRad"));
        double theta = inputdata->get_value("AeroTwst",i);
        //　元の座標系(pitch plate)に対して、３軸周りにthetaだけ回転する
        //  そして、1軸と3軸を入れ替える

        Vec3d e3(std::cos(theta),std::sin(theta),0.);
        Vec3d e1(0.,0.,1.);
        Frame offset(blade_label+i,positon_offset, e1,e3,velocity_offset, angular_velocity_offset);
        ReferenceFrame Bld_sec_frame(blade_label+i, bld_base_reference1,offset);
        references[i] = ReferenceFrame(blade_label+i+1,bld_base_reference1,offset);
        
        // Aero
        double z1 = 0.25 - InterpAeroCent[i-1] * inputdata->get_value("Chord",i);
        Vec3d position_offset_Aero = Vec3d(0.,0.,z1);
        Vec3d e_1(1.,0.,0.);
        Vec3d e_3(0.,0.,1.);
        Frame offset3(i,position_offset_Aero,e_1,e_3, velocity_offset, angular_velocity_offset);
        references[3*i+16] = ReferenceFrame(blade_label+i+100,references[i],offset3);

        //CM_M  
        double CMflap = 0.0;
        double CMedge = -InterpEdgcgOf[i-1]; 
        Vec3d position_offset_CM = Vec3d(0.,CMflap,CMedge); 
        Frame offset4(i,position_offset_CM, e_1, e_3, velocity_offset, angular_velocity_offset);
        references[3*i+17] = ReferenceFrame(blade_label+i+200, references[i],offset4);

        //EA_M
        double EAflap = 0.0;
        double EAedge = 0.0;
        Vec3d position_offset_EA_M = Vec3d(0.,EAflap,EAedge);
        Frame offset5(i, position_offset_EA_M, e_1,e_3, velocity_offset,angular_velocity_offset);
        references[3*i+18] = ReferenceFrame(blade_label+i+300, references[i],offset5);
        }
        else if(i==18){
        Vec3d e1(1.,0.,0.);
        Vec3d e3(0.,0.,1.);
        Vec3d position_offset_TpBr = Vec3d(0.,0.,BldFlexL);
        Frame offset_TpBr(i, position_offset_TpBr,e1,e3,velocity_offset, angular_velocity_offset);
        references[i] = ReferenceFrame(blade_label+i+400,bld_base_reference1,offset_TpBr);
        }     
      
      }

    

  
    //上記の分でnodeの定義に使うrefereceが出力される。仮に追加で必要になったら、以下の方法で追加できる。
    // reference.push_back( ReferenceFrame(********) )
}
void
Blade::set_nodes(){
    for(int i=0; i<num_nodes; i++) {
        int curr_node_label = blade_label + i + 1;
        // set_reference()にて、nodeの個数分座標系が定義してあるので、それらを用いてnode定義を行う。
        // reference[i]からの相対座標系も入力可能となっているが、基本的にreferenceの位置とnodeの位置は一致するようにするので、追加のoffsetは単位座標系となる。
        // 最後の引数はMBDynソルバーが計算結果を出力するか制御するフラグ、現時点では全てのnodeの解析結果を出力するので1、今後出力を制御するためにこの機能をつけている。
        // offset_nullはグローバル変数として定義してある。右クリックで定義を参照可能。
        // node classのvector配列のメンバ変数に保存
        nodes[i] = Node(curr_node_label, references[i], offset_null, 1);
    }
}
void
Blade::set_rigidbodies() {
    for(int i=0; i<num_rigidbodies; i++) {
       // labelはnodeと同じものを使用
        int curr_body_label = blade_label + i + 1;
        int curr_node_label = nodes[i].get_label();
        double curr_length;
        double mass;
        double ThnBarI;
        double inerFA;
        double inerSS;
        double inerYaw;

        if(i==0) {
          mass = inputdata->get_value("SmllNmBr");
          inerFA = inputdata->get_value("SmllNmBr");
          inerSS = inputdata->get_value("SmllNmBr");
          inerYaw = inputdata->get_value("SmllNmBr");          
    
        }else if(i>0 && i<18) {
          curr_length = inputdata->get_value("DRNodes",i);
          mass =  InterpBMass[i-1] * curr_length;
          ThnBarI = InterpBMass[i-1] * pow(curr_length,3.0)/12.0;
          inerFA = InterpFlpIner[i-1] * curr_length + ThnBarI;
          inerSS = InterpdgIner[i-1] * curr_length + ThnBarI;
          inerYaw = (InterpFlpIner[i-1] + InterpdgIner[i-1]) * curr_length;     
        }else if(i<num_rigidbodies) {
          mass = inputdata->get_value("SmllNmBr");
          inerFA = inputdata->get_value("SmllNmBr");
          inerSS = inputdata->get_value("SmllNmBr");
          inerYaw = inputdata->get_value("SmllNmBr");           
        }

        // 重心位置のnodeからの相対位置を設定、Bladeはnodeの位置と重心位置は等しい
        Vec3d cm = zero3;
        // 慣性モーメントを取得
        Vec3d inertia_moment(inerYaw, inerSS, inerFA); 

        //最後の引数はMBDynの出力制御用、現在rigidbodyの出力は特に見ていない
        rigidbodies[i] = RigidBody(curr_body_label, curr_node_label, mass, cm, inertia_moment, 0);
    }

}

// deformablejointから再開　メンバ変数として使用する線形補間した値をname[i]の形に格納してある
void
Blade::set_deformablejoint() {

    int PolyOrd = 6;

    // KBF  KBE
    
    double KBF = 0.0;
    double KBE = 0.0;

    for(int j=0; j<num; j++) {

      double SHPKBF = 0.0;
      double SHPKBE = 0.0;
      double BlFract = (inputdata->get_value("RNodes", j+1)-inputdata->get_value("HubRad"))/BldFlexL;

      for(int i=2; i<PolyOrd+1; i++) {
 
        int Swtch0 = 0;
        int Swtch1 = 0;
        int Swtch2 = 1;
        double Deriv = 2.0;
        double CoefTmp = Swtch0 + Swtch1 * i + Swtch2 * i * (i-1);

        if(i==2) {
          SHPKBF = inputdata ->get_value("BldFl1Sh",2) * CoefTmp/pow(BldFlexL,Deriv);
          SHPKBE = inputdata ->get_value("BldEdgSh",2) * CoefTmp/pow(BldFlexL,Deriv);
        } else {
          SHPKBF = SHPKBF + inputdata ->get_value("BldFl1Sh",i) * CoefTmp * pow(BlFract,i-Deriv)/pow(BldFlexL,Deriv);
          SHPKBE = SHPKBE + inputdata ->get_value("BldEdgSh",i) * CoefTmp * pow(BlFract,i-Deriv)/pow(BldFlexL,Deriv);
        }
      }

      double ElStffFlp = InterpFlpStff[j] * inputdata->get_value("DRNodes", j+1);
      double ElStffBE = InterpEdgStff[j] * inputdata->get_value("DRNodes", j+1);
      KBF = KBF + ElStffFlp * SHPKBF * SHPKBF;  
      KBE = KBE + ElStffBE * SHPKBE * SHPKBE;  
    }
    //  MBF & MBE

    double MBF = 0.0;
    double MBE = 0.0;

    for(int j=0; j<num; j++) {

      double SHPMBF = 0.0;
      double SHPMBE = 0.0;
      double BlFract = (inputdata->get_value("RNodes", j+1)-inputdata->get_value("HubRad"))/BldFlexL;
      
      for(int i=2; i<PolyOrd+1; i++) {

        int Swtch0 = 1;
        int Swtch1 = 0;
        int Swtch2 = 0;
        int Deriv = 0;

        int CoefTmp = Swtch0 + Swtch1 * i + Swtch2 * i * (i-1);
        SHPMBF = SHPMBF + inputdata ->get_value("BldFl1Sh",i) * CoefTmp * pow(BlFract,i-Deriv)/pow(BldFlexL,Deriv); 
        SHPMBE = SHPMBE + inputdata ->get_value("BldEdgSh",i) * CoefTmp * pow(BlFract,i-Deriv)/pow(BldFlexL,Deriv); 
      }  
      
      double ElmntBldMass = InterpBMass[j] * inputdata->get_value("DRNodes", j+1);
      MBF = MBF + ElmntBldMass * SHPMBF * SHPMBF;
      MBE = MBE + ElmntBldMass * SHPMBE * SHPMBE;
    }
    
    double TipMass = 0.0;
    double FreqBF = 0.5/M_PI * sqrt(KBF/(MBF-TipMass));
    double FreqBE = 0.5/M_PI * sqrt(KBE/(MBE-TipMass));

    double CRatioBFl;
    double CRatioBEd;
    double CRatioBEA;
    double CRatioBGJ;

    for(int i=0; i<num_deformable_joints; i++) {
 
        int curr_node1;
        int curr_node2;

        int joint_label = blade_label + i + 1;
        // jointの位置を指定する際の座標系、node1の座標系を使用
        Frame base_frame = nodes[i].get_frame();
        //Frame base_frame;
        // その座標系からのoffset拘束点はnode1の場所とするので、offsetはなし
        Frame offset = offset_null;
        

        Mat6x6d kMatrix = zero6x6;
        Mat6x6d Cmatrix = zero6x6;

        CRatioBFl = 0.01*(inputdata ->get_value("BldFlDmp1"))/(M_PI * FreqBF);
        CRatioBEd = 0.01*(inputdata ->get_value("BldEdDmp1"))/(M_PI * FreqBE);
        CRatioBEA = 0.01;
        CRatioBGJ = 0.01;

        // set Kmatrix and Cmatrix
        if(i==0) {
          curr_node1 = nodes[i].get_label();
          curr_node2 = nodes[i+1].get_label();
          double TmpLength = 0.5 * inputdata->get_value("DRNodes",i+1);
          double TmpLength2 = TmpLength * TmpLength;
          double TmpLength3 = TmpLength2 * TmpLength;

          kMatrix.set(0,0) = InterpEAStff[i]/TmpLength;
          kMatrix.set(1,1) = 12.0*InterpFlpStff[i] /TmpLength3;
          kMatrix.set(2,2) = 12.0*InterpEdgStff[i]/TmpLength3;
          kMatrix.set(3,3) = InterpGJStff[i]/TmpLength;
          kMatrix.set(4,4) = 4.0*InterpEdgStff[i]/TmpLength;
          kMatrix.set(5,5) = 4.0*InterpFlpStff[i]/TmpLength;
          kMatrix.set(1,5) = -6.0*InterpFlpStff[i]/TmpLength2;
          kMatrix.set(2,4) = 6.0*InterpEdgStff[i]/TmpLength2;
          kMatrix.set(5,1) = kMatrix.set(1,5);
          kMatrix.set(4,2) = kMatrix.set(2,4);

          Cmatrix.set(0,0) = kMatrix.set(0,0) * CRatioBEA;
          Cmatrix.set(1,1) = kMatrix.set(1,1) * CRatioBFl;
          Cmatrix.set(2,2) = kMatrix.set(2,2) * CRatioBEd;
          Cmatrix.set(3,3) = kMatrix.set(3,3) * CRatioBGJ;
          Cmatrix.set(4,4) = kMatrix.set(4,4) * CRatioBEd;
          Cmatrix.set(5,5) = kMatrix.set(5,5) * CRatioBFl;
          Cmatrix.set(1,5) = kMatrix.set(1,5) * CRatioBFl;
          Cmatrix.set(2,4) = kMatrix.set(2,4) * CRatioBEd;
          Cmatrix.set(5,1) = Cmatrix.set(1,5);
          Cmatrix.set(4,2) = Cmatrix.set(2,4);

        // i  修正必要！！
        }else if(i>0 && i<17) {
          curr_node1 = nodes[i].get_label();
          curr_node2 = nodes[i+1].get_label();
          double TmpLength = 0.5 *(inputdata->get_value("DRNodes",i+1) + inputdata->get_value("DRNodes",i));
          double TmpLength2 = TmpLength * TmpLength;
          double TmpLength3 = TmpLength2 * TmpLength;

          kMatrix.set(0,0) = 0.5*(InterpEAStff[i] + InterpEAStff[i-1])/TmpLength;
          kMatrix.set(1,1) = 0.5*12.0*(InterpFlpStff[i] + InterpFlpStff[i-1])/TmpLength3;
          kMatrix.set(2,2) = 0.5*12.0*(InterpEdgStff[i]+InterpEdgStff[i-1])/TmpLength3;
          kMatrix.set(3,3) = 0.5*(InterpGJStff[i]+InterpGJStff[i-1])/TmpLength;
          kMatrix.set(4,4) = 0.25*4.0*(3.0*InterpEdgStff[i]+InterpEdgStff[i-1])/TmpLength;
          kMatrix.set(5,5) = 0.25*4.0*(3.0*InterpFlpStff[i] + InterpFlpStff[i-1])/TmpLength;
          kMatrix.set(1,5) = -6.0/3.0*(2.0*InterpFlpStff[i] + InterpFlpStff[i-1])/TmpLength2;
          kMatrix.set(2,4) = 6.0/3.0*(2.0*InterpEdgStff[i] + InterpEdgStff[i-1])/TmpLength2;
          kMatrix.set(5,1) = kMatrix.set(1,5);
          kMatrix.set(4,2) = kMatrix.set(2,4);

          Cmatrix.set(0,0) = kMatrix.set(0,0) * CRatioBEA;
          Cmatrix.set(1,1) = kMatrix.set(1,1) * CRatioBFl;
          Cmatrix.set(2,2) = kMatrix.set(2,2) * CRatioBEd;
          Cmatrix.set(3,3) = kMatrix.set(3,3) * CRatioBGJ;
          Cmatrix.set(4,4) = kMatrix.set(4,4) * CRatioBEd;
          Cmatrix.set(5,5) = kMatrix.set(5,5) * CRatioBFl;
          Cmatrix.set(1,5) = kMatrix.set(1,5) * CRatioBFl;
          Cmatrix.set(2,4) = kMatrix.set(2,4) * CRatioBEd;
          Cmatrix.set(5,1) = Cmatrix.set(1,5);
          Cmatrix.set(4,2) = Cmatrix.set(2,4);

        }else if(i==17) {
          curr_node1 = nodes[i].get_label();
          curr_node2 = nodes[i+1].get_label();
          double TmpLength = 0.5 * inputdata->get_value("DRNodes",i);
          double TmpLength2 = TmpLength * TmpLength;
          double TmpLength3 = TmpLength2 * TmpLength;

          kMatrix.set(0,0) = InterpEAStff[i-1]/TmpLength;
          kMatrix.set(1,1) = 12.0*InterpFlpStff[i-1] /TmpLength3;
          kMatrix.set(2,2) = 12.0*InterpEdgStff[i-1]/TmpLength3;
          kMatrix.set(3,3) = InterpGJStff[i-1]/TmpLength;
          kMatrix.set(4,4) = 4.0*InterpEdgStff[i-1]/TmpLength;
          kMatrix.set(5,5) = 4.0*InterpFlpStff[i-1]/TmpLength;
          kMatrix.set(1,5) = -6.0*InterpFlpStff[i-1]/TmpLength2;
          kMatrix.set(2,4) = 6.0*InterpEdgStff[i-1]/TmpLength2;
          kMatrix.set(5,1) = kMatrix.set(1,5);
          kMatrix.set(4,2) = kMatrix.set(2,4);

          Cmatrix.set(0,0) = kMatrix.set(0,0) * CRatioBEA;
          Cmatrix.set(1,1) = kMatrix.set(1,1) * CRatioBFl;
          Cmatrix.set(2,2) = kMatrix.set(2,2) * CRatioBEd;
          Cmatrix.set(3,3) = kMatrix.set(3,3) * CRatioBGJ;
          Cmatrix.set(4,4) = kMatrix.set(4,4) * CRatioBEd;
          Cmatrix.set(5,5) = kMatrix.set(5,5) * CRatioBFl;
          Cmatrix.set(1,5) = kMatrix.set(1,5) * CRatioBFl;
          Cmatrix.set(2,4) = kMatrix.set(2,4) * CRatioBEd;
          Cmatrix.set(5,1) = Cmatrix.set(1,5);
          Cmatrix.set(4,2) = Cmatrix.set(2,4);
        }

        // jointの位置を指定する座標系
        ReferenceFrame joint_position(joint_label, base_frame, offset);

        deformable_joints[i] = DeformableJoint(joint_label, curr_node1, curr_node2, joint_position, kMatrix,Cmatrix, 0);
    }

}

void
Blade::set_total_joint() {
    for(int i=0; i<num_total_joints; i++) {
        //連続するnodeをtotal jointで拘束、解析初期時間が終わると拘束を開放する。

        int curr_node1 = nodes[i].get_label();
        int curr_node2 = nodes[i+1].get_label();

        int joint_label = blade_label + i + 1 + 100; // jointのラベルがdeformable jointと一致しないようにする
       
        // jointの位置を指定する際の座標系、node1の座標系を使用
        Frame base_frame = nodes[i].get_frame();
        // その座標系からのoffset拘束点はnode1の場所とするので、offsetはなし
        Frame offset = offset_null;

        ReferenceFrame joint_position = ReferenceFrame(joint_label, base_frame, offset);
        total_joints[i] = TotalJoint(joint_label, curr_node1, curr_node2, joint_position, "Total", 0);
        
    }
}

void
Blade::write_reference_in(std::ofstream &output_file) const {

    output_file<<"#----Blade node reference----"<<std::endl;

    for(const ReferenceFrame &flm : references) {
        flm.write_reference(output_file);
    }
}

void
Blade::write_nodes_in(std::ofstream &output_file) const {
    output_file << "#----Blade node-----" << std::endl;
    for(const Node &nod : nodes) {
        nod.write_node(output_file);
    }

}

void
Blade::write_elements_in(std::ofstream &output_file) const {

    output_file << "#----Blade Rigid Bodies-----" <<std::endl;
    for(const RigidBody &rbd : rigidbodies){
        rbd.write_in_file(output_file);
    }
    output_file << "#----Blade defomable joint-----" <<std::endl;
    for(const DeformableJoint &dfj : deformable_joints) {
        dfj.write_in_file(output_file);
    }
    output_file << "#----Blade initial total joint-----" <<std::endl;
    for(const TotalJoint &ttj : total_joints ) {
        output_file << "driven :"<<ttj.get_label()<<", " <<"string, \"Time <= 0.0125\""  <<", " << std::endl;
        ttj.write_in_file(output_file);
    }

}

int 
Blade::get_num_nodes() const {
    // Blade nodes 
    return num_nodes ;
}

int 
Blade::get_num_rigid_bodies() const {
    return num_rigidbodies;
}

int
Blade::get_num_joints() const {
    // tower joints 
    return num_total_joints + num_deformable_joints ;
}