#include "Platform.h"
#include <cmath>
#include <math.h>

Platform::Platform(int label, InputData *ID)
: platform_label(label), inputdata(ID) // const メンバ変数を引数で初期化
{
    num_reference = inputdata->get_value("NPtfmDivH");
    num_nodes = num_reference;
    num_rigidbodies = num_reference;
    num_deformable_joints = num_reference - 1;
    num_total_joints = num_reference -1 ;

    //vector変数を初期化
    references.resize(num_reference);
    nodes.resize(num_nodes);
    rigidbodies.resize(num_rigidbodies);
    deformable_joints.resize(num_deformable_joints);
    total_joints.resize(num_total_joints);

    // surge等の初期値を取得
    init_displacement = Vec3d(inputdata->get_value("PtfmSurge"), inputdata->get_value("PtfmSway"), inputdata->get_value("PtfmHeave"));
    // euler角は回転順番で入力する。　roll pitch yawはEular321なので、yaw, pitch rollの順番で入力する。
    init_euler321     = Vec3d(inputdata->get_value("PtfmYaw"),   inputdata->get_value("PtfmPitch"),inputdata->get_value("PtfmRoll"));

    // platformのノードが参照する座標系、取得した初期値から座標系を作製
    // 初期値が全て０ならグローバル座標系と等価
    ptfm_reference = Frame(platform_label, init_displacement, init_euler321, "Euler321", zero3, zero3);

    // glound reference, gound node, ground joint
    const int ground_label = 1;
    ground_reference = Frame(ground_label, zero3, eye3x3, zero3, zero3);
    ground_node = StaticNode(ground_label, ground_reference.get_label(), 0);
    ground_joint = ClampJoint(ground_label, ground_node.get_label(), 0);

    // platform top　のReferanceFrameを作る、この座標系はClass Towerに渡す
    double ptfm_top_height = -1*inputdata->get_value("TwrDraft"); //mslからのdraftで入力しているので、座標系を換算しておく
    Vec3d ptfm_top(0.,0.,ptfm_top_height);
    Frame offset(platform_label + 500, ptfm_top, eye3x3, zero3, zero3);
    ptfm_top_reference = ReferenceFrame(platform_label + 500, ptfm_reference, offset);

    set_reference();
    set_nodes();
    set_rigidbodies();
    set_deformablejoint();
    set_total_joint();
}

void
Platform::set_reference() {
    for(int i=0; i<num_nodes; i++) {
        int curr_ref_label = platform_label + i + 1;

        // nodeのptfm_referenceに対する相対座標を計算
        double node_height = inputdata->get_value("EZCOORD", i+1); // inputdataでのインデックスは1から始まる
        Vec3d positon_offset = Vec3d(0.,0.,node_height);

        // nodeは1軸をptfm_referenceのz軸、３軸をptfm_referenceのx軸と一致するようにとる＝＞nodeの1軸がプラットフォームの軸方向、nodeの３軸が風下方向
        Vec3d e1(0.,0.,1);
        Vec3d e3(1.,0.,0.);

        // 相対速度はなし
        Vec3d velocity_offset = zero3;
        Vec3d angular_velocity_offset = zero3;

        // ptfm_referenceからの相対座標を　class Frameにて作製
        Frame offset(curr_ref_label, positon_offset, e1, e3, velocity_offset, angular_velocity_offset);

        // platform referenceから　offsetだけ変化した座標系をReferenceFrameクラスで作成し、vector配列のメンバ変数に保存
        references[i] = ReferenceFrame(curr_ref_label, ptfm_reference, offset);
    }
    //上記の分でnodeの定義に使うrefereceが出力される。仮に追加で必要になったら、以下の方法で追加できる。
    // reference.push_back( ReferenceFrame(********) )
}

void
Platform::set_nodes(){
    for(int i=0; i<num_nodes; i++) {
        int curr_node_label = platform_label + i + 1;
        // set_reference()にて、nodeの個数分座標系が定義してあるので、それらを用いてnode定義を行う。
        // reference[i]からの相対座標系も入力可能となっているが、基本的にreferenceの位置とnodeの位置は一致するようにするので、追加のoffsetは単位座標系となる。
        // 最後の引数はMBDynソルバーが計算結果を出力するか制御するフラグ、現時点では全てのnodeの解析結果を出力するので1、今後出力を制御するためにこの機能をつけている。
        // offset_nullはグローバル変数として定義してある。右クリックで定義を参照可能。
        // node classのvector配列のメンバ変数に保存
        nodes[i] = Node(curr_node_label, references[i], offset_null, 1);
    }
}

void
Platform::set_rigidbodies() {
    for(int i=0; i<num_rigidbodies; i++) {
        // labelはnodeと同じものを使用
        int curr_body_label = platform_label + i + 1;
        int curr_node_label = nodes[i].get_label();

        double curr_length = inputdata->get_value("ELENGTH", i+1);
        // massを取得
        double mass = inputdata->get_value("EMASS", i+1) * curr_length;

        // 重心位置のnodeからの相対位置を設定、platformはnodeの位置と重心位置は等しい
        Vec3d cm = zero3;
        // 慣性モーメントを取得
        double inerFA = inputdata->get_value("InerFA", i+1) * curr_length;
        double inerSS = inputdata->get_value("InerSS", i+1) * curr_length;
        double inerYaw = inputdata->get_value("InerYaw", i+1) * curr_length;

        Vec3d inertia_moment(inerYaw, inerSS, inerFA); // nodeの1軸が浮体軸

        //最後の引数はMBDynの出力制御用、現在rigidbodyの出力は特に見ていない
        rigidbodies[i] = RigidBody(curr_body_label, curr_node_label, mass, cm, inertia_moment, 0);
    }
}

void
Platform::set_deformablejoint() {
    //CMatrixの係数の計算に使用する変数

    // TwrTpMass
    double RotMass = inputdata ->get_value("HubMass");
    double NacMass = inputdata ->get_value("NacMass");
    double YawBrMass = inputdata ->get_value("YawBrMass");
    double RFrlMass = inputdata ->get_value("RFrlMass");
    double BoomMass = inputdata ->get_value("BoomMass");
    double TFinMass = inputdata ->get_value("TFinMass");
    double TwrTpMass = RotMass + NacMass + YawBrMass + RFrlMass + BoomMass + TFinMass;
    
    double TwrHt = inputdata ->get_value("TowerHt");
    double TwrDft = inputdata ->get_value("TwrDraft");
    double TwrRBHt = inputdata ->get_value("TwrRBHt");
    double TwrFlexL = TwrHt + TwrDft - TwrRBHt;
    double TwrNodes = inputdata ->get_value("TwrNodes");
    double DHNodes = TwrFlexL/TwrNodes;
    double num_reference_Tower = inputdata ->get_value("NTwInpSt");
    double FAStTunr = inputdata ->get_value("FAStTunr1");
    double SSStTunr = inputdata ->get_value("SSStTunr1");
    int PolyOrd = 6;

    // Interpolution TwFAStif & TwSSStif    KTFA KTSS
    
    double KTFA1 = 0.0;
    double KTSS1 = 0.0;

    for(int j=0; j<TwrNodes; j++) {

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

    for(int j=0; j<TwrNodes; j++) {

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
    
    for(int i=0; i<num_deformable_joints; i++) {

        int curr_node1 = nodes[i].get_label();
        int curr_node2 = nodes[i+1].get_label();

        int joint_label = platform_label + i + 1;

        // jointの位置を指定する際の座標系、node1の座標系を使用
        Frame base_frame = nodes[i].get_frame();
        // その座標系からのoffset拘束点はnode1の場所とするので、offsetはなし
        Frame offset = offset_null;
        // jointの位置を指定する座標系
        ReferenceFrame joint_position(joint_label, base_frame, offset);

        Mat6x6d kMatrix = zero6x6;
        Mat6x6d Cmatrix = zero6x6;

        // Tower　node間距離
        double TmpLength = 0.5*(inputdata ->get_value("ELENGTH", i+1) + inputdata->get_value("ELENGTH", (i+2)));
        double TmpLength2 = TmpLength*TmpLength;
        double TmpLength3 = TmpLength2*TmpLength;
        // 剛性
        double StiffPEA1 = inputdata ->get_value("EAStif", i+1);
        double StiffPEA2 = inputdata ->get_value("EAStif", i+2);
        double StiffPSS1 = inputdata ->get_value("SSStif", i+1);
        double StiffPSS2 = inputdata ->get_value("SSStif", i+2);
        double StiffPFA1 = inputdata ->get_value("FAStif", i+1);
        double StiffPFA2 = inputdata ->get_value("FAStif", i+2);
        double StiffPGJ1 = inputdata ->get_value("GJStif", i+1);
        double StiffPGJ2 = inputdata ->get_value("GJStif", i+2);

        //ベータ減衰　CRatio = 減衰係数/(π*固有周波数)
        double CRatioTFA = 0.01*(inputdata ->get_value("TwrFADmp1"))/(M_PI * FreqTFA);
        double CRatioTSS = 0.01*(inputdata ->get_value("TwrSSDmp1"))/(M_PI * FreqTSS);
        double CRatioTEA = 0.01*1.0;
        double CRatioTGJ = 0.01*1.0;

        // set Kmatrix and Cmatrix
        kMatrix.set(0,0) =  0.001*0.5*(StiffPEA1+StiffPEA2)/TmpLength;
        kMatrix.set(1,1) =  0.001*6.0*(StiffPSS1+StiffPSS2)/TmpLength3;
        kMatrix.set(2,2) =  0.001*6.0*(StiffPFA1+StiffPFA2)/TmpLength3;
        kMatrix.set(3,3) =  0.001*0.5*(StiffPGJ1+StiffPGJ2)/TmpLength;
        kMatrix.set(4,4) =  0.001*(3.0*StiffPFA1+StiffPFA2)/TmpLength;
        kMatrix.set(5,5) =  0.001*(3.0*StiffPSS1+StiffPSS2)/TmpLength;
        kMatrix.set(1,5) = -0.001*2.0*(2.0*StiffPSS1+StiffPSS2)/TmpLength2;
        kMatrix.set(2,4) =  0.001*2.0*(2.0*StiffPFA1+StiffPFA2)/TmpLength2;
        kMatrix.set(5,1) = kMatrix.set(1,5);
        kMatrix.set(4,2) = kMatrix.set(2,4);
        Cmatrix.set(0,0) = kMatrix.set(0,0)*CRatioTEA;
        Cmatrix.set(1,1) = kMatrix.set(1,1)*CRatioTSS;
        Cmatrix.set(2,2) = kMatrix.set(2,2)*CRatioTFA;
        Cmatrix.set(3,3) = kMatrix.set(3,3)*CRatioTGJ;
        Cmatrix.set(4,4) = kMatrix.set(4,4)*CRatioTFA;
        Cmatrix.set(5,5) = kMatrix.set(5,5)*CRatioTSS;
        Cmatrix.set(1,5) = kMatrix.set(1,5)*CRatioTSS;
        Cmatrix.set(2,4) = kMatrix.set(2,4)*CRatioTFA;
        Cmatrix.set(5,1) = Cmatrix.set(1,5);
        Cmatrix.set(1,5) = Cmatrix.set(1,5);

        deformable_joints[i] = DeformableJoint(joint_label, curr_node1, curr_node2, joint_position, kMatrix,Cmatrix, 0);
    }
}

void
Platform::set_total_joint() {
    for(int i=0; i<num_total_joints; i++) {
        //連続するnodeをtotal jointで拘束、解析初期時間が終わると拘束を開放する。

        int curr_node1 = nodes[i].get_label();
        int curr_node2 = nodes[i+1].get_label();

        int joint_label = platform_label + i + 1 + 100; // jointのラベルがdeformable jointと一致しないようにする

        // jointの位置を指定する際の座標系、node1の座標系を使用
        Frame base_frame = nodes[i].get_frame();
        // その座標系からのoffset拘束点はnode1の場所とするので、offsetはなし
        Frame offset = offset_null;

        ReferenceFrame joint_position = ReferenceFrame(joint_label, base_frame, offset);
        total_joints[i] = TotalJoint(joint_label, curr_node1, curr_node2, joint_position, "Total", 0);
    }
}

void
Platform::write_reference_in(std::ofstream &output_file) const {
    output_file<<"#-----Ground Reference------"<<std::endl;
    ground_reference.write_reference(output_file);

    output_file<<"#-----Platform Reference------"<<std::endl;
    ptfm_reference.write_reference(output_file);

    output_file<<"#-----Platform Top Reference------"<<std::endl;
    ptfm_top_reference.write_reference(output_file);

    output_file<<"#----Platform node reference----"<<std::endl;

    for(const ReferenceFrame &flm : references) {
        flm.write_reference(output_file);
    }
}

void
Platform::write_nodes_in(std::ofstream &output_file) const {
    output_file << "#----Ground node-----" << std::endl;
    ground_node.write_node(output_file);

    output_file << "#----Platform node-----" << std::endl;
    for(const Node &nod : nodes) {
        nod.write_node(output_file);
    }
}

void
Platform::write_elements_in(std::ofstream &output_file) const {
    output_file << "#----Ground Clamp Joint -------" <<std::endl;
    ground_joint.write_in_file(output_file);

    output_file << "#----Platfrom Rigid Bodies-----" <<std::endl;
    for(const RigidBody &rbd : rigidbodies){
        rbd.write_in_file(output_file);
    }
    output_file << "#----Platfrom defomable joint-----" <<std::endl;
    for(const DeformableJoint &dfj : deformable_joints) {
        dfj.write_in_file(output_file);
    }
    output_file << "#----Platfrom initial total joint-----" <<std::endl;
    for(const TotalJoint &ttj : total_joints ) {
        output_file << "driven :"<<ttj.get_label()<<", " <<"string, \"Time <= InitConstraint\""<<std::endl;
        ttj.write_in_file(output_file);
    }
}

int
Platform::get_num_nodes() const {
    // platform nodes + ground
    return num_nodes + 1 ;
}

int
Platform::get_num_rigid_bodies() const {
    return num_rigidbodies;
}

int
Platform::get_num_joints() const {
    // platform joints + ground
    return num_total_joints + num_deformable_joints + 1;
}