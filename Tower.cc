#include "Tower.h"
#include <cmath>
#include <math.h>

Tower::Tower(int label, const ReferenceFrame &towerbase, InputData *ID) 
: tower_label(label), tower_base_reference(towerbase), inputdata(ID) // const メンバ変数を引数で初期化
{
    num_reference = inputdata ->get_value("TwrNodes");
    num_nodes = num_reference;
    num_rigidbodies = num_reference;
    //platform_top_referenceと接続のため
    num_deformable_joints = num_reference + 1;
    num_total_joints = num_deformable_joints ;
    TwrHt = inputdata ->get_value("TowerHt");
    TwrDft = inputdata ->get_value("TwrDraft");
    TwrRBHt = inputdata ->get_value("TwrRBHt");
    TwrFlexL = TwrHt + TwrDft - TwrRBHt;
    DHNodes = TwrFlexL/num_reference;

    //vector変数を初期化
    references.resize(num_reference);
    nodes.resize(num_nodes);
    rigidbodies.resize(num_rigidbodies);
    deformable_joints.resize(num_deformable_joints);
    total_joints.resize(num_total_joints);

    Vec3d e1(0.,0.,1.);
    Vec3d e3(1.,0.,0.);

    tower_base_node = Node(tower_base_reference.get_label(), tower_base_reference, Frame(0.,zero3,e1,e3,zero3,zero3),1);

    // tower top　のReferanceFrameを作る、この座標系はClass RNAに渡す
    double tower_top_height = TwrFlexL; 
    Vec3d tower_top(0.,0.,tower_top_height);
    Frame offset(tower_label + 500, tower_top, eye3x3, zero3, zero3);
    tower_top_node = Node(tower_label + 500, tower_base_reference, offset, 1);
    tower_top_reference = ReferenceFrame(tower_label + 500, tower_base_reference, offset);
    tower_top_ref = ReferenceFrame(tower_label + 500 +1, tower_base_reference, offset_null);
    tower_top_joint = TotalJoint(tower_label + 500 + 1, tower_label + num_nodes, tower_label + 500, tower_top_ref, "Total", 0);

    set_reference();
    set_nodes();
    set_rigidbodies();
    set_deformablejoint();
    set_total_joint();
}

void
Tower::set_reference() {

    for(int i=0; i<num_nodes; i++) {
        int curr_ref_label = tower_label + i + 1;

        // nodeのtower_referenceに対する相対座標を計算
        double node_height = 0.5 * DHNodes * (2 * (i+1)-1); 
        Vec3d positon_offset = Vec3d(0.,0.,node_height);

        // nodeは1軸をptfm_referenceのz軸、３軸をptfm_referenceのx軸と一致するようにとる＝＞nodeの1軸がプラットフォームの軸方向、nodeの３軸が風下方向
        Vec3d e1(0.,0.,1);
        Vec3d e3(1.,0.,0.);

        // 相対速度はなし
        Vec3d velocity_offset = zero3;
        Vec3d angular_velocity_offset = zero3;

        // tower_referenceからの相対座標を　class Frameにて作製
        Frame offset(curr_ref_label, positon_offset, e1, e3, velocity_offset, angular_velocity_offset);

        // tower referenceから　offsetだけ変化した座標系をReferenceFrameクラスで作成し、vector配列のメンバ変数に保存
        references[i] = ReferenceFrame(curr_ref_label, tower_base_reference, offset);
    }
    //上記の分でnodeの定義に使うrefereceが出力される。仮に追加で必要になったら、以下の方法で追加できる。
    // reference.push_back( ReferenceFrame(********) )
}
void
Tower::set_nodes(){
    for(int i=0; i<num_nodes; i++) {
        int curr_node_label = tower_label + i + 1;
        // set_reference()にて、nodeの個数分座標系が定義してあるので、それらを用いてnode定義を行う。
        // reference[i]からの相対座標系も入力可能となっているが、基本的にreferenceの位置とnodeの位置は一致するようにするので、追加のoffsetは単位座標系となる。
        // 最後の引数はMBDynソルバーが計算結果を出力するか制御するフラグ、現時点では全てのnodeの解析結果を出力するので1、今後出力を制御するためにこの機能をつけている。
        // offset_nullはグローバル変数として定義してある。右クリックで定義を参照可能。
        // node classのvector配列のメンバ変数に保存
        nodes[i] = Node(curr_node_label, references[i], offset_null, 1);
    }
}
void
Tower::set_rigidbodies() {
    for(int i=0; i<num_rigidbodies; i++) {

      // Interpolution EMass & InerFA & InerSS
      double XAray1;
      double XAray2;
      double MAray1;
      double MAray2;
      double FAAray1;
      double FAAray2;
      double SSAray1;
      double SSAray2;
      double XVal = 0.5 * DHNodes * (2 * (i+1)-1);
      
      if(i<2) {
        XAray1 = inputdata ->get_value("HtFract", 1) * TwrFlexL;
        XAray2 = inputdata ->get_value("HtFract", 2) * TwrFlexL;
        MAray1 = inputdata ->get_value("TMassDen", 1);
        MAray2 = inputdata ->get_value("TMassDen", 2);
        FAAray1 = inputdata ->get_value("TwFAIner" ,1);
        FAAray2 = inputdata ->get_value("TwFAIner" ,2);
        SSAray1 = inputdata ->get_value("TwSSIner", 1);
        SSAray2 = inputdata ->get_value("TwSSIner", 2);
      } else if(i>1 && i<4) {
        XAray1 = inputdata ->get_value("HtFract", 2) * TwrFlexL;
        XAray2 = inputdata ->get_value("HtFract", 3) * TwrFlexL;
        MAray1 = inputdata ->get_value("TMassDen", 2);
        MAray2 = inputdata ->get_value("TMassDen", 3);
        FAAray1 = inputdata ->get_value("TwFAIner" ,2);
        FAAray2 = inputdata ->get_value("TwFAIner" ,3);
        SSAray1 = inputdata ->get_value("TwSSIner", 2);
        SSAray2 = inputdata ->get_value("TwSSIner", 3);
      } else if(i>3 && i<6) {
        XAray1 = inputdata ->get_value("HtFract", 3) * TwrFlexL;
        XAray2 = inputdata ->get_value("HtFract", 4) * TwrFlexL;
        MAray1 = inputdata ->get_value("TMassDen", 3);
        MAray2 = inputdata ->get_value("TMassDen", 4);
        FAAray1 = inputdata ->get_value("TwFAIner" ,3);
        FAAray2 = inputdata ->get_value("TwFAIner" ,4);
        SSAray1 = inputdata ->get_value("TwSSIner", 3);
        SSAray2 = inputdata ->get_value("TwSSIner", 4);
      } else if(i>5 && i<8) {
        XAray1 = inputdata ->get_value("HtFract", 4) * TwrFlexL;
        XAray2 = inputdata ->get_value("HtFract", 5) * TwrFlexL;
        MAray1 = inputdata ->get_value("TMassDen", 4);
        MAray2 = inputdata ->get_value("TMassDen", 5);
        FAAray1 = inputdata ->get_value("TwFAIner" ,4);
        FAAray2 = inputdata ->get_value("TwFAIner" ,5);
        SSAray1 = inputdata ->get_value("TwSSIner", 4);
        SSAray2 = inputdata ->get_value("TwSSIner", 5);
      } else if(i>7 && i<10) {
        XAray1 = inputdata ->get_value("HtFract", 5) * TwrFlexL;
        XAray2 = inputdata ->get_value("HtFract", 6) * TwrFlexL;
        MAray1 = inputdata ->get_value("TMassDen", 5);
        MAray2 = inputdata ->get_value("TMassDen", 6);
        FAAray1 = inputdata ->get_value("TwFAIner" ,5);
        FAAray2 = inputdata ->get_value("TwFAIner" ,6);
        SSAray1 = inputdata ->get_value("TwSSIner", 5);
        SSAray2 = inputdata ->get_value("TwSSIner", 6);
      } else if(i>9 && i<12) {
        XAray1 = inputdata ->get_value("HtFract", 6) * TwrFlexL;
        XAray2 = inputdata ->get_value("HtFract", 7) * TwrFlexL;
        MAray1 = inputdata ->get_value("TMassDen", 6);
        MAray2 = inputdata ->get_value("TMassDen", 7);
        FAAray1 = inputdata ->get_value("TwFAIner" ,6);
        FAAray2 = inputdata ->get_value("TwFAIner" ,7);
        SSAray1 = inputdata ->get_value("TwSSIner", 6);
        SSAray2 = inputdata ->get_value("TwSSIner", 7);
      } else if(i>11 && i<14) {
        XAray1 = inputdata ->get_value("HtFract", 7) * TwrFlexL;
        XAray2 = inputdata ->get_value("HtFract", 8) * TwrFlexL;
        MAray1 = inputdata ->get_value("TMassDen", 7);
        MAray2 = inputdata ->get_value("TMassDen", 8);
        FAAray1 = inputdata ->get_value("TwFAIner" ,7);
        FAAray2 = inputdata ->get_value("TwFAIner" ,8);
        SSAray1 = inputdata ->get_value("TwSSIner", 7);
        SSAray2 = inputdata ->get_value("TwSSIner", 8);
      } else if(i>13 && i<16) {
        XAray1 = inputdata ->get_value("HtFract", 8) * TwrFlexL;
        XAray2 = inputdata ->get_value("HtFract", 9) * TwrFlexL;
        MAray1 = inputdata ->get_value("TMassDen", 8);
        MAray2 = inputdata ->get_value("TMassDen", 9);
        FAAray1 = inputdata ->get_value("TwFAIner" ,8);
        FAAray2 = inputdata ->get_value("TwFAIner" ,9);
        SSAray1 = inputdata ->get_value("TwSSIner", 8);
        SSAray2 = inputdata ->get_value("TwSSIner", 9);
      } else if(i>15 && i<18) {
        XAray1 = inputdata ->get_value("HtFract", 9) * TwrFlexL;
        XAray2 = inputdata ->get_value("HtFract", 10) * TwrFlexL;
        MAray1 = inputdata ->get_value("TMassDen", 9);
        MAray2 = inputdata ->get_value("TMassDen", 10);
        FAAray1 = inputdata ->get_value("TwFAIner" ,9);
        FAAray2 = inputdata ->get_value("TwFAIner" ,10);
        SSAray1 = inputdata ->get_value("TwSSIner", 9);
        SSAray2 = inputdata ->get_value("TwSSIner", 10);
      } else {
        XAray1 = inputdata ->get_value("HtFract", 10) * TwrFlexL;
        XAray2 = inputdata ->get_value("HtFract", 11) * TwrFlexL;
        MAray1 = inputdata ->get_value("TMassDen", 10);
        MAray2 = inputdata ->get_value("TMassDen", 11);
        FAAray1 = inputdata ->get_value("TwFAIner" ,10);
        FAAray2 = inputdata ->get_value("TwFAIner" ,11);
        SSAray1 = inputdata ->get_value("TwSSIner", 10);
        SSAray2 = inputdata ->get_value("TwSSIner", 11);
      }
  
      double InterpTMass = (MAray2-MAray1) * (XVal-XAray1)/(XAray2-XAray1) + MAray1;
      double InterpFAIner = (FAAray2-FAAray1) * (XVal-XAray1)/(XAray2-XAray1) + FAAray1;
      double InterpSSIner = (SSAray2-SSAray1) * (XVal-XAray1)/(XAray2-XAray1) + SSAray1;
      
        // labelはnodeと同じものを使用
        int curr_body_label = tower_label + i + 1;
        int curr_node_label = nodes[i].get_label();

        double curr_length = DHNodes;
        // massを取得
        double mass =  InterpTMass * curr_length;

        // 重心位置のnodeからの相対位置を設定、platformはnodeの位置と重心位置は等しい
        Vec3d cm = zero3;
        // 慣性モーメントを取得
        double inerFA = InterpFAIner * curr_length;
        double inerSS = InterpSSIner * curr_length;
        double inerYaw = (InterpFAIner + InterpSSIner) * curr_length;

        Vec3d inertia_moment(inerYaw, inerSS, inerFA); // nodeの1軸が浮体軸

        //最後の引数はMBDynの出力制御用、現在rigidbodyの出力は特に見ていない
        rigidbodies[i] = RigidBody(curr_body_label, curr_node_label, mass, cm, inertia_moment, 0);
    }

}

void
Tower::set_deformablejoint() {

    double RotMass = inputdata ->get_value("HubMass");
    double NacMass = inputdata ->get_value("NacMass");
    double YawBrMass = inputdata ->get_value("YawBrMass");
    double RFrlMass = inputdata ->get_value("RFrlMass");
    double BoomMass = inputdata ->get_value("BoomMass");
    double TFinMass = inputdata ->get_value("TFinMass");
    double TwrTpMass = RotMass + NacMass + YawBrMass + RFrlMass + BoomMass + TFinMass;
    double FAStTunr = inputdata ->get_value("FAStTunr1");
    double SSStTunr = inputdata ->get_value("SSStTunr1");
    int PolyOrd = 6;

    // Interpolution TwFAStif & TwSSStif    KTFA  KTSS
    
    double KTFA1 = 0.0;
    double KTSS1 = 0.0;

    for(int j=0; j<num_reference; j++) {

      double TwrFASF1 = 0.0;
      double TwrSSSF1 = 0.0;

      for(int i=2; i<PolyOrd+1; i++) {
 
        int Swtch0 = 0;
        int Swtch1 = 0;
        int Swtch2 = 1;
        double Deriv = 2.0;
        double CoefTmp = Swtch0 + Swtch1 * i + Swtch2 * i * (i-1);

        if(i==2) {
          TwrFASF1 = inputdata ->get_value("TwFAM1Sh",2) * CoefTmp/pow(TwrFlexL,Deriv);
          TwrSSSF1 = inputdata ->get_value("TwSSM1Sh",2) * CoefTmp/pow(TwrFlexL,Deriv);
        } else {
          TwrFASF1 = TwrFASF1 + inputdata ->get_value("TwFAM1Sh",i) * CoefTmp * pow((0.5 * DHNodes * (2 * (j+1)-1))/TwrFlexL,i-Deriv)/pow(TwrFlexL,Deriv);
          TwrSSSF1 = TwrSSSF1 + inputdata ->get_value("TwSSM1Sh",i) * CoefTmp * pow((0.5 * DHNodes * (2 * (j+1)-1))/TwrFlexL,i-Deriv)/pow(TwrFlexL,Deriv);
        }
      }

      double XAray1;
      double XAray2;
      double YAray1;
      double YAray2;
      double ZAray1;
      double ZAray2;
      double XVal = 0.5 * DHNodes * (2 * (j+1)-1);
      
      if(j<2) {
        XAray1 = inputdata ->get_value("HtFract", 1) * TwrFlexL;
        XAray2 = inputdata ->get_value("HtFract", 2) * TwrFlexL;
        YAray1 = inputdata ->get_value("TwFAStif", 1);
        YAray2 = inputdata ->get_value("TwFAStif", 2);
        ZAray1 = inputdata ->get_value("TwSSStif", 1);
        ZAray2 = inputdata ->get_value("TwSSStif", 2);
      } else if(j>1 && j<4) {
        XAray1 = inputdata ->get_value("HtFract", 2) * TwrFlexL;
        XAray2 = inputdata ->get_value("HtFract", 3) * TwrFlexL;
        YAray1 = inputdata ->get_value("TwFAStif", 2);
        YAray2 = inputdata ->get_value("TwFAStif", 3);
        ZAray1 = inputdata ->get_value("TwSSStif", 2);
        ZAray2 = inputdata ->get_value("TwSSStif", 3);
      } else if(j>3 && j<6) {
        XAray1 = inputdata ->get_value("HtFract", 3) * TwrFlexL;
        XAray2 = inputdata ->get_value("HtFract", 4) * TwrFlexL;
        YAray1 = inputdata ->get_value("TwFAStif", 3);
        YAray2 = inputdata ->get_value("TwFAStif", 4);
        ZAray1 = inputdata ->get_value("TwSSStif", 3);
        ZAray2 = inputdata ->get_value("TwSSStif", 4);
      } else if(j>5 && j<8) {
        XAray1 = inputdata ->get_value("HtFract", 4) * TwrFlexL;
        XAray2 = inputdata ->get_value("HtFract", 5) * TwrFlexL;
        YAray1 = inputdata ->get_value("TwFAStif", 4);
        YAray2 = inputdata ->get_value("TwFAStif", 5);
        ZAray1 = inputdata ->get_value("TwSSStif", 4);
        ZAray2 = inputdata ->get_value("TwSSStif", 5);
      } else if(j>7 && j<10) {
        XAray1 = inputdata ->get_value("HtFract", 5) * TwrFlexL;
        XAray2 = inputdata ->get_value("HtFract", 6) * TwrFlexL;
        YAray1 = inputdata ->get_value("TwFAStif", 5);
        YAray2 = inputdata ->get_value("TwFAStif", 6);
        ZAray1 = inputdata ->get_value("TwSSStif", 5);
        ZAray2 = inputdata ->get_value("TwSSStif", 6);
      } else if(j>9 && j<12) {
        XAray1 = inputdata ->get_value("HtFract", 6) * TwrFlexL;
        XAray2 = inputdata ->get_value("HtFract", 7) * TwrFlexL;
        YAray1 = inputdata ->get_value("TwFAStif", 6);
        YAray2 = inputdata ->get_value("TwFAStif",7);
        ZAray1 = inputdata ->get_value("TwSSStif", 6);
        ZAray2 = inputdata ->get_value("TwSSStif", 7);
      } else if(j>11 && j<14) {
        XAray1 = inputdata ->get_value("HtFract", 7) * TwrFlexL;
        XAray2 = inputdata ->get_value("HtFract", 8) * TwrFlexL;
        YAray1 = inputdata ->get_value("TwFAStif", 7);
        YAray2 = inputdata ->get_value("TwFAStif", 8);
        ZAray1 = inputdata ->get_value("TwSSStif", 7);
        ZAray2 = inputdata ->get_value("TwSSStif", 8);
      } else if(j>13 && j<16) {
        XAray1 = inputdata ->get_value("HtFract", 8) * TwrFlexL;
        XAray2 = inputdata ->get_value("HtFract", 9) * TwrFlexL;
        YAray1 = inputdata ->get_value("TwFAStif", 8);
        YAray2 = inputdata ->get_value("TwFAStif", 9);
        ZAray1 = inputdata ->get_value("TwSSStif", 8);
        ZAray2 = inputdata ->get_value("TwSSStif", 9);
      } else if(j>15 && j<18) {
        XAray1 = inputdata ->get_value("HtFract", 9) * TwrFlexL;
        XAray2 = inputdata ->get_value("HtFract", 10) * TwrFlexL;
        YAray1 = inputdata ->get_value("TwFAStif", 9);
        YAray2 = inputdata ->get_value("TwFAStif", 10);
        ZAray1 = inputdata ->get_value("TwSSStif", 9);
        ZAray2 = inputdata ->get_value("TwSSStif", 10);
      } else {
        XAray1 = inputdata ->get_value("HtFract", 10) * TwrFlexL;
        XAray2 = inputdata ->get_value("HtFract", 11) * TwrFlexL;
        YAray1 = inputdata ->get_value("TwFAStif", 10);
        YAray2 = inputdata ->get_value("TwFAStif", 11);
        ZAray1 = inputdata ->get_value("TwSSStif", 10);
        ZAray2 = inputdata ->get_value("TwSSStif", 11);
      }
  
      double InterpTwFAStif = (YAray2-YAray1) * (XVal-XAray1)/(XAray2-XAray1) + YAray1;
      double InterpTwSSStif = (ZAray2-ZAray1) * (XVal-XAray1)/(XAray2-XAray1) + ZAray1;
      double ElstffFA = InterpTwFAStif * DHNodes;
      double ElstffSS = InterpTwSSStif * DHNodes;
      KTFA1 = KTFA1 + ElstffFA * TwrFASF1 * TwrFASF1;  
      KTSS1 = KTSS1 + ElstffSS * TwrSSSF1 * TwrSSSF1;  
    }

    // Interpolution TMass & MTFA & MTSS

    double MTFA = TwrTpMass;
    double MTSS = TwrTpMass;

    for(int j=0; j<num_reference; j++) {

      double TwrFASF2 = 0.0;
      double TwrSSSF2 = 0.0;
      
      for(int i=2; i<PolyOrd+1; i++) {

        int Swtch0 = 1;
        int Swtch1 = 0;
        int Swtch2 = 0;
        int Deriv = 0;

        int CoefTmp = Swtch0 + Swtch1 * i + Swtch2 * i * (i-1);
        TwrFASF2 = TwrFASF2 + inputdata ->get_value("TwFAM1Sh",i) * CoefTmp * pow((0.5 * DHNodes * (2* (j+1)-1))/TwrFlexL,i-Deriv)/pow(TwrFlexL,Deriv); 
        TwrSSSF2 = TwrSSSF2 + inputdata ->get_value("TwSSM1Sh",i) * CoefTmp * pow((0.5 * DHNodes * (2* (j+1)-1))/TwrFlexL,i-Deriv)/pow(TwrFlexL,Deriv); 
      }  
      
      double XAray1;
      double XAray2;
      double YAray1;
      double YAray2;
      double XVal = 0.5 * DHNodes * (2 * (j+1)-1);

      if(j<2) {
        XAray1 = inputdata ->get_value("HtFract", 1) * TwrFlexL;
        XAray2 = inputdata ->get_value("HtFract", 2) * TwrFlexL;
        YAray1 = inputdata ->get_value("TMassDen", 1);
        YAray2 = inputdata ->get_value("TMassDen", 2);
        
      } else if(j>1 && j<4) {
        XAray1 = inputdata ->get_value("HtFract", 2) * TwrFlexL;
        XAray2 = inputdata ->get_value("HtFract", 3) * TwrFlexL;
        YAray1 = inputdata ->get_value("TMassDen", 2);
        YAray2 = inputdata ->get_value("TMassDen", 3);
      } else if(j>3 && j<6) {
        XAray1 = inputdata ->get_value("HtFract", 3) * TwrFlexL;
        XAray2 = inputdata ->get_value("HtFract", 4) * TwrFlexL;
        YAray1 = inputdata ->get_value("TMassDen", 3);
        YAray2 = inputdata ->get_value("TMassDen", 4);
      } else if(j>5 && j<8) {
        XAray1 = inputdata ->get_value("HtFract", 4) * TwrFlexL;
        XAray2 = inputdata ->get_value("HtFract", 5) * TwrFlexL;
        YAray1 = inputdata ->get_value("TMassDen", 4);
        YAray2 = inputdata ->get_value("TMassDen", 5);
      } else if(j>7 && j<10) {
        XAray1 = inputdata ->get_value("HtFract", 5) * TwrFlexL;
        XAray2 = inputdata ->get_value("HtFract", 6) * TwrFlexL;
        YAray1 = inputdata ->get_value("TMassDen", 5);
        YAray2 = inputdata ->get_value("TMassDen", 6);
      } else if(j>9 && j<12) {
        XAray1 = inputdata ->get_value("HtFract", 6) * TwrFlexL;
        XAray2 = inputdata ->get_value("HtFract", 7) * TwrFlexL;
        YAray1 = inputdata ->get_value("TMassDen", 6);
        YAray2 = inputdata ->get_value("TMassDen", 7);
      } else if(j>11 && j<14) {
        XAray1 = inputdata ->get_value("HtFract", 7) * TwrFlexL;
        XAray2 = inputdata ->get_value("HtFract", 8) * TwrFlexL;
        YAray1 = inputdata ->get_value("TMassDen", 7);
        YAray2 = inputdata ->get_value("TMassDen", 8);
      } else if(j>13 && j<16) {
        XAray1 = inputdata ->get_value("HtFract", 8) * TwrFlexL;
        XAray2 = inputdata ->get_value("HtFract", 9) * TwrFlexL;
        YAray1 = inputdata ->get_value("TMassDen", 8);
        YAray2 = inputdata ->get_value("TMassDen", 9);
      } else if(j>15 && j<18) {
        XAray1 = inputdata ->get_value("HtFract", 9) * TwrFlexL;
        XAray2 = inputdata ->get_value("HtFract", 10) * TwrFlexL;
        YAray1 = inputdata ->get_value("TMassDen", 9);
        YAray2 = inputdata ->get_value("TMassDen", 10);
      } else {
        XAray1 = inputdata ->get_value("HtFract", 10) * TwrFlexL;
        XAray2 = inputdata ->get_value("HtFract", 11) * TwrFlexL;
        YAray1 = inputdata ->get_value("TMassDen", 10);
        YAray2 = inputdata ->get_value("TMassDen", 11);
      }

      double InterpTMass = (YAray2-YAray1) * (XVal-XAray1)/(XAray2-XAray1) + YAray1;
      double ElmntMass = InterpTMass * DHNodes;
      MTFA = MTFA + ElmntMass * TwrFASF2 * TwrFASF2;
      MTSS = MTSS + ElmntMass * TwrSSSF2 * TwrSSSF2;
    }

    

    double KTFA = KTFA1 * sqrt(FAStTunr * FAStTunr);
    double FreqTFA = 0.5/M_PI * sqrt(KTFA/(MTFA-TwrTpMass));

    double KTSS = KTSS1 * sqrt(SSStTunr * SSStTunr);
    double FreqTSS = 0.5/M_PI * sqrt(KTSS/(MTSS-TwrTpMass));

    // Interpolution TwFAStif & TwSSStif & TwGJStif & TwEAStif
    std::vector<double> InterpTwFAStif(num_reference);
    std::vector<double> InterpTwSSStif(num_reference);
    std::vector<double> InterpTwGJStif(num_reference);
    std::vector<double> InterpTwEAStif(num_reference);

    for (int i=0; i<num_reference; i++) {
    
        double XAray1;
        double XAray2;
        double YAray1;
        double YAray2;
        double ZAray1;
        double ZAray2;
        double AAray1;
        double AAray2;
        double BAray1;
        double BAray2;
        double XVal = 0.5 * DHNodes * (2 * (i+1)-1);
           
        if(i<2) {
            XAray1 = inputdata ->get_value("HtFract", 1) * TwrFlexL;
            XAray2 = inputdata ->get_value("HtFract", 2) * TwrFlexL;
            YAray1 = inputdata ->get_value("TwFAStif", 1);
            YAray2 = inputdata ->get_value("TwFAStif", 2);
            ZAray1 = inputdata ->get_value("TwSSStif", 1);
            ZAray2 = inputdata ->get_value("TwSSStif", 2);
            AAray1 = inputdata ->get_value("TwGJStif", 1);
            AAray2 = inputdata ->get_value("TwGJStif", 2);
            BAray1 = inputdata ->get_value("TwEAStif", 1);
            BAray2 = inputdata ->get_value("TwEAStif", 2);
        } else if(i>1 && i<4) {
            XAray1 = inputdata ->get_value("HtFract", 2) * TwrFlexL;
            XAray2 = inputdata ->get_value("HtFract", 3) * TwrFlexL;
            YAray1 = inputdata ->get_value("TwFAStif", 2);
            YAray2 = inputdata ->get_value("TwFAStif", 3);
            ZAray1 = inputdata ->get_value("TwSSStif", 2);
            ZAray2 = inputdata ->get_value("TwSSStif", 3);
            AAray1 = inputdata ->get_value("TwGJStif", 2);
            AAray2 = inputdata ->get_value("TwGJStif", 3);
            BAray1 = inputdata ->get_value("TwEAStif", 2);
            BAray2 = inputdata ->get_value("TwEAStif", 3);
        } else if(i>3 && i<6) {
            XAray1 = inputdata ->get_value("HtFract", 3) * TwrFlexL;
            XAray2 = inputdata ->get_value("HtFract", 4) * TwrFlexL;
            YAray1 = inputdata ->get_value("TwFAStif", 3);
            YAray2 = inputdata ->get_value("TwFAStif", 4);
            ZAray1 = inputdata ->get_value("TwSSStif", 3);
            ZAray2 = inputdata ->get_value("TwSSStif", 4);
            AAray1 = inputdata ->get_value("TwGJStif", 3);
            AAray2 = inputdata ->get_value("TwGJStif", 4);
            BAray1 = inputdata ->get_value("TwEAStif", 3);
            BAray2 = inputdata ->get_value("TwEAStif", 4);
        } else if(i>5 && i<8) {
            XAray1 = inputdata ->get_value("HtFract", 4) * TwrFlexL;
            XAray2 = inputdata ->get_value("HtFract", 5) * TwrFlexL;
            YAray1 = inputdata ->get_value("TwFAStif", 4);
            YAray2 = inputdata ->get_value("TwFAStif", 5);
            ZAray1 = inputdata ->get_value("TwSSStif", 4);
            ZAray2 = inputdata ->get_value("TwSSStif", 5);
            AAray1 = inputdata ->get_value("TwGJStif", 4);
            AAray2 = inputdata ->get_value("TwGJStif", 5);
            BAray1 = inputdata ->get_value("TwEAStif", 4);
            BAray2 = inputdata ->get_value("TwEAStif", 5);
        } else if(i>7 && i<10) {
            XAray1 = inputdata ->get_value("HtFract", 5) * TwrFlexL;
            XAray2 = inputdata ->get_value("HtFract", 6) * TwrFlexL;
            YAray1 = inputdata ->get_value("TwFAStif", 5);
            YAray2 = inputdata ->get_value("TwFAStif", 6);
            ZAray1 = inputdata ->get_value("TwSSStif", 5);
            ZAray2 = inputdata ->get_value("TwSSStif", 6);
            AAray1 = inputdata ->get_value("TwGJStif", 5);
            AAray2 = inputdata ->get_value("TwGJStif", 6);
            BAray1 = inputdata ->get_value("TwEAStif", 5);
            BAray2 = inputdata ->get_value("TwEAStif", 6);
        } else if(i>9 && i<12) {
            XAray1 = inputdata ->get_value("HtFract", 6) * TwrFlexL;
            XAray2 = inputdata ->get_value("HtFract", 7) * TwrFlexL;
            YAray1 = inputdata ->get_value("TwFAStif", 6);
            YAray2 = inputdata ->get_value("TwFAStif",7);
            ZAray1 = inputdata ->get_value("TwSSStif", 6);
            ZAray2 = inputdata ->get_value("TwSSStif", 7);
            AAray1 = inputdata ->get_value("TwGJStif", 6);
            AAray2 = inputdata ->get_value("TwGJStif", 7);
            BAray1 = inputdata ->get_value("TwEAStif", 6);
            BAray2 = inputdata ->get_value("TwEAStif", 7);
        } else if(i>11 && i<14) {
            XAray1 = inputdata ->get_value("HtFract", 7) * TwrFlexL;
            XAray2 = inputdata ->get_value("HtFract", 8) * TwrFlexL;
            YAray1 = inputdata ->get_value("TwFAStif", 7);
            YAray2 = inputdata ->get_value("TwFAStif", 8);
            ZAray1 = inputdata ->get_value("TwSSStif", 7);
            ZAray2 = inputdata ->get_value("TwSSStif", 8);
            AAray1 = inputdata ->get_value("TwGJStif", 7);
            AAray2 = inputdata ->get_value("TwGJStif", 8);
            BAray1 = inputdata ->get_value("TwEAStif", 7);
            BAray2 = inputdata ->get_value("TwEAStif", 8);
        } else if(i>13 && i<16) {
            XAray1 = inputdata ->get_value("HtFract", 8) * TwrFlexL;
            XAray2 = inputdata ->get_value("HtFract", 9) * TwrFlexL;
            YAray1 = inputdata ->get_value("TwFAStif", 8);
            YAray2 = inputdata ->get_value("TwFAStif", 9);
            ZAray1 = inputdata ->get_value("TwSSStif", 8);
            ZAray2 = inputdata ->get_value("TwSSStif", 9);
            AAray1 = inputdata ->get_value("TwGJStif", 8);
            AAray2 = inputdata ->get_value("TwGJStif", 9);
            BAray1 = inputdata ->get_value("TwEAStif", 8);
            BAray2 = inputdata ->get_value("TwEAStif", 9);
        } else if(i>15 && i<18) {
            XAray1 = inputdata ->get_value("HtFract", 9) * TwrFlexL;
            XAray2 = inputdata ->get_value("HtFract", 10) * TwrFlexL;
            YAray1 = inputdata ->get_value("TwFAStif", 9);
            YAray2 = inputdata ->get_value("TwFAStif", 10);
            ZAray1 = inputdata ->get_value("TwSSStif", 9);
            ZAray2 = inputdata ->get_value("TwSSStif", 10);
            AAray1 = inputdata ->get_value("TwGJStif", 9);
            AAray2 = inputdata ->get_value("TwGJStif", 10);
            BAray1 = inputdata ->get_value("TwEAStif", 9);
            BAray2 = inputdata ->get_value("TwEAStif", 10);
        } else {
            XAray1 = inputdata ->get_value("HtFract", 10) * TwrFlexL;
            XAray2 = inputdata ->get_value("HtFract", 11) * TwrFlexL;
            YAray1 = inputdata ->get_value("TwFAStif", 10);
            YAray2 = inputdata ->get_value("TwFAStif", 11);
            ZAray1 = inputdata ->get_value("TwSSStif", 10);
            ZAray2 = inputdata ->get_value("TwSSStif", 11);
            AAray1 = inputdata ->get_value("TwGJStif", 10);
            AAray2 = inputdata ->get_value("TwGJStif", 11);
            BAray1 = inputdata ->get_value("TwEAStif", 10);
            BAray2 = inputdata ->get_value("TwEAStif", 11);
        }
        InterpTwFAStif[i] = (YAray2-YAray1) * (XVal-XAray1)/(XAray2-XAray1) + YAray1;
        InterpTwSSStif[i] = (ZAray2-ZAray1) * (XVal-XAray1)/(XAray2-XAray1) + ZAray1;
        InterpTwGJStif[i] = (AAray2-AAray1) * (XVal-XAray1)/(XAray2-XAray1) + AAray1;
        InterpTwEAStif[i] = (BAray2-BAray1) * (XVal-XAray1)/(XAray2-XAray1) + BAray1;
    }

    double CRatioTFA;
    double CRatioTSS;
    double CRatioTEA;
    double CRatioTGJ;


    for(int i=0; i<num_deformable_joints; i++) {
 
        int curr_node1;
        int curr_node2;

        int joint_label = tower_label + i + 1;
        // jointの位置を指定する際の座標系、node1の座標系を使用
        //Frame base_frame = nodes[i].get_frame();
        Frame base_frame;
        // その座標系からのoffset拘束点はnode1の場所とするので、offsetはなし
        Frame offset = offset_null;
        

        Mat6x6d kMatrix = zero6x6;
        Mat6x6d Cmatrix = zero6x6;

        CRatioTFA = 0.01*(inputdata ->get_value("TwrFADmp1"))/(M_PI * FreqTFA);
        CRatioTSS = 0.01*(inputdata ->get_value("TwrSSDmp1"))/(M_PI * FreqTSS);
        CRatioTEA = 0.01*1.0;
        CRatioTGJ = 0.01*1.0;

        // set Kmatrix and Cmatrix
        if(i==0) {
          curr_node1 = tower_base_node.get_label();
          curr_node2 = nodes[i].get_label();
          base_frame = tower_base_reference;
          double TmpLength = 0.5 * DHNodes;
          double TmpLength2 = TmpLength * TmpLength;
          double TmpLength3 = TmpLength2 * TmpLength;
          kMatrix.set(0,0) = InterpTwEAStif[i]/TmpLength;
          kMatrix.set(1,1) = 12.0*InterpTwSSStif[i] /TmpLength3;
          kMatrix.set(2,2) = 12.0*InterpTwFAStif[i]/TmpLength3;
          kMatrix.set(3,3) = InterpTwGJStif[i]/TmpLength;
          kMatrix.set(4,4) = 4.0*InterpTwFAStif[i]/TmpLength;
          kMatrix.set(5,5) = 4.0*InterpTwSSStif[i]/TmpLength;
          kMatrix.set(1,5) = -6.0*InterpTwSSStif[i]/TmpLength2;
          kMatrix.set(2,4) = 6.0*InterpTwFAStif[i]/TmpLength2;
          kMatrix.set(5,1) = kMatrix.set(1,5);
          kMatrix.set(4,2) = kMatrix.set(2,4);
          Cmatrix.set(0,0) = kMatrix.set(0,0) * CRatioTEA;
          Cmatrix.set(1,1) = kMatrix.set(1,1) * CRatioTSS;
          Cmatrix.set(2,2) = kMatrix.set(2,2) * CRatioTFA;
          Cmatrix.set(3,3) = kMatrix.set(3,3) * CRatioTGJ;
          Cmatrix.set(4,4) = kMatrix.set(4,4) * CRatioTFA;
          Cmatrix.set(5,5) = kMatrix.set(5,5) * CRatioTSS;
          Cmatrix.set(1,5) = kMatrix.set(1,5) * CRatioTSS;
          Cmatrix.set(2,4) = kMatrix.set(2,4) * CRatioTFA;
          Cmatrix.set(5,1) = Cmatrix.set(1,5);
          Cmatrix.set(4,2) = Cmatrix.set(2,4);
        }else if(i>0 && i<num_deformable_joints-1) {
          curr_node1 = nodes[i-1].get_label();
          curr_node2 = nodes[i].get_label();
          base_frame = nodes[i-1].get_frame();
          double TmpLength = DHNodes;
          double TmpLength2 = TmpLength * TmpLength;
          double TmpLength3 = TmpLength2 * TmpLength;
          kMatrix.set(0,0) = 0.5*(InterpTwEAStif[i] + InterpTwEAStif[i-1])/TmpLength;
          kMatrix.set(1,1) = 6.0*(InterpTwSSStif[i] + InterpTwSSStif[i-1])/TmpLength3;
          kMatrix.set(2,2) = 6.0*(InterpTwFAStif[i] + InterpTwFAStif[i-1])/TmpLength3;
          kMatrix.set(3,3) = 0.5*(InterpTwGJStif[i] + InterpTwGJStif[i-1])/TmpLength;
          kMatrix.set(4,4) = (3.0*InterpTwFAStif[i] + InterpTwFAStif[i-1])/TmpLength;
          kMatrix.set(5,5) = (3.0*InterpTwSSStif[i] + InterpTwSSStif[i-1])/TmpLength;
          kMatrix.set(1,5) = -2.0*(2.0*InterpTwSSStif[i] + InterpTwSSStif[i-1])/TmpLength2;
          kMatrix.set(2,4) = 2.0*(2.0*InterpTwFAStif[i] + InterpTwFAStif[i-1])/TmpLength2;
          kMatrix.set(5,1) = kMatrix.set(1,5);
          kMatrix.set(4,2) = kMatrix.set(2,4);
          Cmatrix.set(0,0) = kMatrix.set(0,0) * CRatioTEA;
          Cmatrix.set(1,1) = kMatrix.set(1,1) * CRatioTSS;
          Cmatrix.set(2,2) = kMatrix.set(2,2) * CRatioTFA;
          Cmatrix.set(3,3) = kMatrix.set(3,3) * CRatioTGJ;
          Cmatrix.set(4,4) = kMatrix.set(4,4) * CRatioTFA;
          Cmatrix.set(5,5) = kMatrix.set(5,5) * CRatioTSS;
          Cmatrix.set(1,5) = kMatrix.set(1,5) * CRatioTSS;
          Cmatrix.set(2,4) = kMatrix.set(2,4) * CRatioTFA;
          Cmatrix.set(5,1) = Cmatrix.set(1,5);
          Cmatrix.set(4,2) = Cmatrix.set(2,4);
        }else{
          curr_node1 = nodes[i-1].get_label();
          curr_node2 = 2500;
          base_frame = nodes[i-1].get_frame();
          double TmpLength = 0.5 * DHNodes;
          double TmpLength2 = TmpLength * TmpLength;
          double TmpLength3 = TmpLength2 * TmpLength;
          kMatrix.set(0,0) = InterpTwEAStif[i-1]/TmpLength;
          kMatrix.set(1,1) = 12.0*InterpTwSSStif[i-1] /TmpLength3;
          kMatrix.set(2,2) = 12.0*InterpTwFAStif[i-1]/TmpLength3;
          kMatrix.set(3,3) = InterpTwGJStif[i-1]/TmpLength;
          kMatrix.set(4,4) = 4.0*InterpTwFAStif[i-1]/TmpLength;
          kMatrix.set(5,5) = 4.0*InterpTwSSStif[i-1]/TmpLength;
          kMatrix.set(1,5) = -6.0*InterpTwSSStif[i-1]/TmpLength2;
          kMatrix.set(2,4) = 6.0*InterpTwFAStif[i-1]/TmpLength2;
          kMatrix.set(5,1) = kMatrix.set(1,5);
          kMatrix.set(4,2) = kMatrix.set(2,4);
          Cmatrix.set(0,0) = kMatrix.set(0,0) * CRatioTEA;
          Cmatrix.set(1,1) = kMatrix.set(1,1) * CRatioTSS;
          Cmatrix.set(2,2) = kMatrix.set(2,2) * CRatioTFA;
          Cmatrix.set(3,3) = kMatrix.set(3,3) * CRatioTGJ;
          Cmatrix.set(4,4) = kMatrix.set(4,4) * CRatioTFA;
          Cmatrix.set(5,5) = kMatrix.set(5,5) * CRatioTSS;
          Cmatrix.set(1,5) = kMatrix.set(1,5) * CRatioTSS;
          Cmatrix.set(2,4) = kMatrix.set(2,4) * CRatioTFA;
          Cmatrix.set(5,1) = Cmatrix.set(1,5);
          Cmatrix.set(4,2) = Cmatrix.set(2,4);
        }

        // jointの位置を指定する座標系
        ReferenceFrame joint_position(joint_label, base_frame, offset);

        deformable_joints[i] = DeformableJoint(joint_label, curr_node1, curr_node2, joint_position, kMatrix,Cmatrix, 0);
        total_joints[i] = TotalJoint(joint_label+100, curr_node1, curr_node2, joint_position,"Total",0);
    }
}

void
Tower::set_total_joint() {

    int twrtop_to_ptfm_label = tower_base_reference.get_label() + 100;
    
    ptfmtop_to_twrbase = TotalJoint(twrtop_to_ptfm_label, 1001, tower_base_node.get_label(), tower_base_reference, "Total",0);
    /*
    for(int i=0; i<num_total_joints; i++) {
        //連続するnodeをtotal jointで拘束、解析初期時間が終わると拘束を開放する。

        int curr_node1 = nodes[i].get_label();
        int curr_node2 = nodes[i+1].get_label();

        int joint_label = tower_label + i + 1 + 100; // jointのラベルがdeformable jointと一致しないようにする

        // jointの位置を指定する際の座標系、node1の座標系を使用
        Frame base_frame = nodes[i].get_frame();
        // その座標系からのoffset拘束点はnode1の場所とするので、offsetはなし
        Frame offset = offset_null;

        ReferenceFrame joint_position = ReferenceFrame(joint_label, base_frame, offset);
        total_joints[i] = TotalJoint(joint_label, curr_node1, curr_node2, joint_position, "Total", 0);
    }
    */
}

void
Tower::write_reference_in(std::ofstream &output_file) const {
    output_file<<"#-----Tower Base Reference------"<<std::endl;
    tower_base_reference.write_reference(output_file);

    output_file<<"#----Tower node reference----"<<std::endl;

    int i = 1;
    for(const ReferenceFrame &flm : references) {
        output_file<<"# Tower section "<<i<<std::endl;
        flm.write_reference(output_file);
        i++;
    }
    output_file<<"#-----Tower Top Reference------"<<std::endl;
    tower_top_reference.write_reference(output_file);
}

void
Tower::write_nodes_in(std::ofstream &output_file) const {
    output_file << "#----Tower base node----" << std::endl;
    tower_base_node.write_node(output_file);

    output_file << "#----Tower node-----" << std::endl;
    for(const Node &nod : nodes) {
        nod.write_node(output_file);
    }

    output_file << "#----Tower top node----" << std::endl;
    tower_top_node.write_node(output_file);
}

void
Tower::write_elements_in(std::ofstream &output_file) const {

    output_file << "#----Tower Rigid Bodies-----" <<std::endl;
    for(const RigidBody &rbd : rigidbodies){
        rbd.write_in_file(output_file);
    }
    output_file << "#----Tower defomable joint-----" <<std::endl;
    for(const DeformableJoint &dfj : deformable_joints) {
        dfj.write_in_file(output_file);
    }
    output_file << "#----Tower initial total joint-----" <<std::endl;
    for(const TotalJoint &ttj : total_joints ) {
        output_file << "driven :"<<ttj.get_label()<<", " <<"string, \"Time <= 0.0125\""  <<", " << std::endl;
        ttj.write_in_file(output_file);
    }

    output_file << "#----Tower top initial total joint-----" <<std::endl;
    output_file << "driven :"<<tower_label + 500 + 1<<", " <<"string, \"Time <= 0.0125\""  <<", " << std::endl;
    tower_top_joint.write_in_file(output_file);
}

void
Tower::write_rigidbodies_in(std::ofstream &output_file) const {

    output_file << "#----Tower Rigid Bodies-----" <<std::endl;
    int i=1;
    for(const RigidBody &rbd : rigidbodies){
        output_file<<"# Tower Seciton "<<i<<std::endl;
        rbd.write_in_file(output_file);
        i++;
    }
}

void
Tower::write_joints_in(std::ofstream &output_file) const {
    output_file << "#----platform top to tower base joint-----" <<std::endl;
    ptfmtop_to_twrbase.write_in_file(output_file);

    output_file << "#----Tower defomable joint-----" <<std::endl;
    for(const DeformableJoint &dfj : deformable_joints) {
        dfj.write_in_file(output_file);
    }
    output_file << "#----Tower initial total joint-----" <<std::endl;
    for(const TotalJoint &ttj : total_joints ) {
        output_file << "driven :"<<ttj.get_label()<<", " <<"string, \"Time <= 0.0125\""  <<", " << std::endl;
        ttj.write_in_file(output_file);
    }
}

int 
Tower::get_num_nodes() const {
    // tower nodes + tower base + tower top
    return num_nodes + 2 ;
}

int 
Tower::get_num_rigid_bodies() const {
    return num_rigidbodies;
}

int
Tower::get_num_joints() const {
    // tower joints + towertop_to_paltform;
    return num_total_joints + num_deformable_joints + 1;
}