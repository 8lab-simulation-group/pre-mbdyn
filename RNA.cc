#include "RNA.h"
#include <cmath>
#include <math.h>

RNA::RNA(int label, const ReferenceFrame &nacellebase, InputData *ID) 
: nacelle_label(label), nacelle_base_reference(nacellebase), inputdata(ID) // const メンバ変数を引数で初期化
{

    num_reference = inputdata ->get_value("NacelleNodes");
    num_nodes = num_reference;
    num_rigidbodies = num_reference;
    num_deformable_hinge = 1;
    num_total_joints = 5;
    num_revolute_hinges = 9;
    num_blds = inputdata->get_value("NumBl");

    //vector変数を初期化
    references.resize(num_reference);
    nodes.resize(num_nodes);
    rigidbodies.resize(num_rigidbodies);
    deformable_hinge.resize(num_deformable_hinge);
    //revolute_hinges.resize(num_revolute_hinges);
    total_joints.resize(num_total_joints);

    // Pitch Blade Bottom(3個)　の ReferanceFrameを作る、この座標系はClass Bladeに渡す
    // Nacelle base reference からの相対座標を計算

    Ri_gprime.set(0,0) = cos(-inputdata->get_value("PreCone") * M_PI/180.0);
    Ri_gprime.set(0,1) = 0.0;
    Ri_gprime.set(0,2) = sin(-inputdata->get_value("PreCone") * M_PI/180.0);
    Ri_gprime.set(1,0) = 0.0;
    Ri_gprime.set(1,1) = 1.0;
    Ri_gprime.set(1,2) = 0.0;
    Ri_gprime.set(2,0) = -Ri_gprime.set(0,2);
    Ri_gprime.set(2,1) = 0.0;
    Ri_gprime.set(2,2) = Ri_gprime.set(0,0);

    Rshft_rotorfurl.set(0,0) = cos(-inputdata->get_value("shftTilt") * M_PI/180.0);
    Rshft_rotorfurl.set(0,1) = 0.0;
    Rshft_rotorfurl.set(0,2) = sin(-inputdata->get_value("shftTilt") * M_PI/180.0);
    Rshft_rotorfurl.set(1,0) = 0.0;
    Rshft_rotorfurl.set(1,1) = 1.0;
    Rshft_rotorfurl.set(1,2) = 0.0;
    Rshft_rotorfurl.set(2,0) = -Rshft_rotorfurl.set(0,2);
    Rshft_rotorfurl.set(2,1) = 0.0;
    Rshft_rotorfurl.set(2,2) = Rshft_rotorfurl.set(0,0);

    Vec3d X_Overhang = Vec3d(inputdata->get_value("Overhang"), 0. ,0.);
    Vec3d Twr2Shft = Vec3d(0.0, 0.0, inputdata->get_value("Twr2Shft"));
    Vec3d Hub_Rad = Vec3d(0.0, 0.0, inputdata->get_value("HubRad"));

    Xhub = Twr2Shft + Rshft_rotorfurl * X_Overhang;
    double RotAngle = 2.0 * M_PI/num_blds;
    

    for (int i=0; i<num_blds; i++) {
        
        Rgprime_Shft.set(0,0) = 1.0;
        Rgprime_Shft.set(0,1) = 0.0;
        Rgprime_Shft.set(0,2) = 0.0;
        Rgprime_Shft.set(1,0) = 0.0;
        Rgprime_Shft.set(1,1) = cos(RotAngle * i);
        Rgprime_Shft.set(1,2) = -sin(RotAngle * i);
        Rgprime_Shft.set(2,0) = 0.0;
        Rgprime_Shft.set(2,1) = -Rgprime_Shft.set(1,2);
        Rgprime_Shft.set(2,2) = Rgprime_Shft.set(1,1);

        if(i==0) {
            R1 = Rgprime_Shft * Ri_gprime;
            PitchPlate1 = Xhub + R1 * Hub_Rad;
            Bld_downwind_distance1 = PitchPlate1[0];
            Bld_y1 = PitchPlate1[1];
            Bld_height1 = PitchPlate1[2]; 
            Vec3d Bld_bottom1(Bld_downwind_distance1,Bld_y1,Bld_height1);
            Frame offset1(nacelle_label + 100 + i, Bld_bottom1, eye3x3, zero3, zero3);
            Bld_base_node1 = Node(nacelle_label + 500 + i + 1, nacelle_base_reference, offset_null, 1);
            Bld_base_reference1 = ReferenceFrame(nacelle_label + 500, nacelle_base_reference, offset1);
        }

        if(i==1) {
            R2 = Rgprime_Shft * Ri_gprime;
            PitchPlate2 = Xhub + R2 * Hub_Rad;
            Bld_downwind_distance2 = PitchPlate2[0];
            Bld_y2 = PitchPlate2[1];
            Bld_height2 = PitchPlate2[2]; 
            Vec3d Bld_bottom2(Bld_downwind_distance2,Bld_y2,Bld_height2);
            Frame offset2(nacelle_label + 100 + i, Bld_bottom2, eye3x3, zero3, zero3);
            Bld_base_node2 = Node(nacelle_label + 500 + i + 1, nacelle_base_reference, offset_null, 1);
            Bld_base_reference2 = ReferenceFrame(nacelle_label + 500, nacelle_base_reference, offset2);
        }

        if(i==2) {
            R3 = Rgprime_Shft * Ri_gprime;
            PitchPlate3 = Xhub + R3 * Hub_Rad;
            Bld_downwind_distance3 = PitchPlate3[0];
            Bld_y3 = PitchPlate3[1];
            Bld_height3 = PitchPlate3[2]; 
            Vec3d Bld_bottom3(Bld_downwind_distance3,Bld_y3,Bld_height3);
            Frame offset3(nacelle_label + 100 + i, Bld_bottom3, eye3x3, zero3, zero3);
            Bld_base_node3 = Node(nacelle_label + 500 + i + 1, nacelle_base_reference, offset_null, 1);
            Bld_base_reference3 = ReferenceFrame(nacelle_label + 500, nacelle_base_reference, offset3);
        }
    }


    set_reference();
    set_nodes();
    set_rigidbodies();
    //set_deformableHinge();
    //set_revoluteHinge();
    set_total_joint();
}

void
RNA::set_reference() {

    for(int i=0; i<num_nodes; i++) {
        int curr_ref_label = nacelle_label + i + 1;
        double node_height;
        double node_Downwind_distance;
        double node_vertical_distance;
        Vec3d position_offset;
        // 相対速度・加速度はなし
        Vec3d velocity_offset = zero3;
        Vec3d angular_velocity_offset = zero3;
        //Vec3d e1;
        //Vec3d e2;
        Frame offset;

        // nodeのNacelle_base_referenceに対する相対座標を計算
        // 回転姿勢：i=0~8まではShaftのblade方向とは反対向きにx軸、y軸そのまま、z軸はx軸と直交するようにとる
        // 回転姿勢：i=9~11(Blade)についてはBladeの長さ方向にz軸をとるようにする
        // tower_top_referenceからの相対座標を　class Frameにて作製

        // Nacelle
        if(i==0) {
            node_Downwind_distance = inputdata->get_value("NacCMxn");
            node_height = inputdata->get_value("NacCMzn"); 
            position_offset = Vec3d(node_Downwind_distance, 0., node_height);
            Vec3d e1(1.,0.,0.);
            Vec3d e3(0.,0.,1);
            offset(curr_ref_label, position_offset, e1, e3, velocity_offset, angular_velocity_offset);
        }

        // TailBoom, TailFin, RotoFur1
        if(i>0 && i<4) {
            position_offset = Vec3d(0.,0.,0.);
            Vec3d e1(1.,0.,0.);
            Vec3d e2(0.,0.,1);
            offset(curr_ref_label, position_offset, e1, e2, velocity_offset, angular_velocity_offset);
        }

        // LSS
        if(i==4) {
            node_Downwind_distance = inputdata->get_value("Overhang") + 0.5 * inputdata->get_value("LSSLength");
            node_height = inputdata->get_value("Twr2Shft");
            position_offset = Vec3d(node_Downwind_distance, 0., node_height);
            Vec3d e1(Rshft_rotorfurl.set(0,0), Rshft_rotorfurl.set(1,0), Rshft_rotorfurl.set(2,0));
            Vec3d e3(Rshft_rotorfurl.set(0,2), Rshft_rotorfurl.set(1,2), Rshft_rotorfurl.set(2,2));
            offset(curr_ref_label, position_offset, e1, e3, velocity_offset, angular_velocity_offset);
        }

        // HSS
        if(i==5) {
            node_Downwind_distance = inputdata->get_value("Overhang") + 0.5 * inputdata->get_value("HSSLength");
            node_height = inputdata->get_value("Twr2Shft");
            position_offset = Vec3d(node_Downwind_distance, 0., node_height);
            Vec3d e1(Rshft_rotorfurl.set(0,0), Rshft_rotorfurl.set(1,0), Rshft_rotorfurl.set(2,0));
            Vec3d e3(Rshft_rotorfurl.set(0,2), Rshft_rotorfurl.set(1,2), Rshft_rotorfurl.set(2,2));
            offset(curr_ref_label, position_offset, e1, e3, velocity_offset, angular_velocity_offset);
        }

        // Generator
        if(i==6) {
            node_Downwind_distance =inputdata->get_value("Overhang") +-inputdata->get_value("LSSLength") + inputdata->get_value("HSSLength") + 0.5 * inputdata->get_value("GenLength");
            position_offset = Vec3d(node_Downwind_distance, 0., 0.);
            Vec3d e1(Rshft_rotorfurl.set(0,0), Rshft_rotorfurl.set(1,0), Rshft_rotorfurl.set(2,0));
            Vec3d e3(Rshft_rotorfurl.set(0,2), Rshft_rotorfurl.set(1,2), Rshft_rotorfurl.set(2,2));
            offset(curr_ref_label, position_offset, e1, e3, velocity_offset, angular_velocity_offset);
        }

        // TeeterPin, Hub
        if(i>6 && 1<9) {
            node_Downwind_distance = Xhub[0];
            node_height = Xhub[2];
            position_offset = Vec3d(node_Downwind_distance, 0., node_height);
            Vec3d e1(Rshft_rotorfurl.set(0,0) , Rshft_rotorfurl.set(1,0), Rshft_rotorfurl.set(2,0));
            Vec3d e3(Rshft_rotorfurl.set(0,2), Rshft_rotorfurl.set(1,2), Rshft_rotorfurl.set(2,2));
            offset(curr_ref_label, position_offset, e1, e3, velocity_offset, angular_velocity_offset);
        }

        // PitchBlade1Bottom
        if(i==9) {
            node_Downwind_distance = Bld_downwind_distance1;
            node_height = Bld_height1;
            position_offset = Vec3d(node_Downwind_distance, 0., node_height);
            Vec3d e1(Rshft_rotorfurl.set(0,0) + R1.set(0,0), Rshft_rotorfurl.set(1,0) + R1.set(1,0), Rshft_rotorfurl.set(2,0) + R1.set(2,0));
            Vec3d e3(Rshft_rotorfurl.set(0,2) + R1.set(0,2), Rshft_rotorfurl.set(1,2) + R1.set(1,2), Rshft_rotorfurl.set(2,2) + R1.set(2,2));
            offset(curr_ref_label, position_offset, e1, e3, velocity_offset, angular_velocity_offset);
        }

        // PitchBlade2Bottom
        if(i==10) {
            node_Downwind_distance = Bld_downwind_distance2;
            node_height = Bld_height2;
            node_vertical_distance = Bld_y2;
            position_offset = Vec3d(node_Downwind_distance, node_vertical_distance, node_height);
            Vec3d e1(Rshft_rotorfurl.set(0,0) + R2.set(0,0), Rshft_rotorfurl.set(1,0) + R2.set(1,0), Rshft_rotorfurl.set(2,0) + R2.set(2,0));
            Vec3d e3(Rshft_rotorfurl.set(0,2) + R2.set(0,2), Rshft_rotorfurl.set(1,2) + R2.set(1,2), Rshft_rotorfurl.set(2,2) + R2.set(2,2));
            offset(curr_ref_label, position_offset, e1, e3, velocity_offset, angular_velocity_offset);
        }

        // PitchBlade3Bottom
        if(i==11) {
            node_Downwind_distance = Bld_downwind_distance3;
            node_height = Bld_height3;
            node_vertical_distance = Bld_y3;
            position_offset = Vec3d(node_Downwind_distance, node_vertical_distance, node_height);
            Vec3d e1(Rshft_rotorfurl.set(0,0) + R3.set(0,0), Rshft_rotorfurl.set(1,0) + R3.set(1,0), Rshft_rotorfurl.set(2,0) + R3.set(2,0));
            Vec3d e2(Rshft_rotorfurl.set(0,2) + R3.set(0,2), Rshft_rotorfurl.set(1,2) + R3.set(1,2), Rshft_rotorfurl.set(2,2) + R3.set(2,2));
            offset(curr_ref_label, position_offset, e1, e2, velocity_offset, angular_velocity_offset);
        }

        // Nacelle_base_referenceから　offsetだけ変化した座標系をReferenceFrameクラスで作成し、vector配列のメンバ変数に保存
        references[i] = ReferenceFrame(curr_ref_label, nacelle_base_reference, offset);
    }
    //上記の分でnodeの定義に使うrefereceが出力される。仮に追加で必要になったら、以下の方法で追加できる。
    // reference.push_back( ReferenceFrame(********) )
}
void
RNA::set_nodes(){
    for(int i=0; i<num_nodes; i++) {
        int curr_node_label = nacelle_label + i + 1;
        // set_reference()にて、nodeの個数分座標系が定義してあるので、それらを用いてnode定義を行う。
        // reference[i]からの相対座標系も入力可能となっているが、基本的にreferenceの位置とnodeの位置は一致するようにするので、追加のoffsetは単位座標系となる。
        // 最後の引数はMBDynソルバーが計算結果を出力するか制御するフラグ、現時点では全てのnodeの解析結果を出力するので1、今後出力を制御するためにこの機能をつけている。
        // offset_nullはグローバル変数として定義してある。右クリックで定義を参照可能。
        // node classのvector配列のメンバ変数に保存
        nodes[i] = Node(curr_node_label, references[i], offset_null, 1);
    }
}
void
RNA::set_rigidbodies() {
    for(int i=0; i<num_rigidbodies; i++) {
  
        // labelはnodeと同じものを使用
        int curr_body_label = nacelle_label + i + 1;
        int curr_node_label = nodes[i].get_label();

        // massを取得
        double mass;
        double inerFA;
        double inerSS;
        double inerYaw;

        // 慣性モーメントを計算
        if(i==0) {
            mass = inputdata->get_value("NacMass") + inputdata->get_value("SmllNmbr");
            inerFA = inputdata->get_value("NacYIner") - inputdata->get_value("NacMass") * (pow(inputdata->get_value("NacCMxn"),2) + pow(inputdata->get_value("NacCMyn"),2)) + inputdata->get_value("SmllNmbr");
            inerSS = inputdata->get_value("SmllNmbr");
            inerYaw = inputdata->get_value("SmllNmbr");
        }

        if(i>0 && i<6) {
            mass = inputdata->get_value("SmllNmbr");
            inerFA = inputdata->get_value("SmllNmbr");
            inerSS = inputdata->get_value("SmllNmbr");
            inerYaw = inputdata->get_value("SmllNmbr");
        }

        if(i==6) {
            mass = inputdata->get_value("SmllNmbr");
            inerFA = inputdata->get_value("SmllNmbr");
            inerSS = inputdata->get_value("SmllNmbr");
            inerYaw = inputdata->get_value("GenIner") * pow(inputdata->get_value("GBRatio"),2) + inputdata->get_value("SmllNmbr");
        }
        // TeeterPin
        if(i==7) {
            mass = inputdata->get_value("SmllNmbr");
            inerFA = inputdata->get_value("SmllNmbr");
            inerSS = inputdata->get_value("SmllNmbr");
            inerYaw = inputdata->get_value("SmllNmbr");
        }
        //Hub
        if(i==8) {
            mass = inputdata->get_value("HubIner") + inputdata->get_value("SmllNmbr");
            inerFA = inputdata->get_value("SmllNmbr");
            inerSS = inputdata->get_value("SmllNmbr");
            inerYaw = inputdata->get_value("hubIner") + inputdata->get_value("SmllNmbr");
        }

        if(i>8 && i<num_rigidbodies) {
            mass = inputdata->get_value("SmllNmbr");
            inerFA = inputdata->get_value("SmllNmbr");
            inerSS = inputdata->get_value("SmllNmbr");
            inerYaw = inputdata->get_value("SmllNmbr"); 
        }
        
        // 重心位置のnodeからの相対位置を設定、platformはnodeの位置と重心位置は等しい
        Vec3d cm = zero3;
        Vec3d inertia_moment(inerYaw, inerSS, inerFA); // nodeの1軸が浮体軸

        //最後の引数はMBDynの出力制御用、現在rigidbodyの出力は特に見ていない
        rigidbodies[i] = RigidBody(curr_body_label, curr_node_label, mass, cm, inertia_moment, 0);
    }

}

//void
//RNA::set_deformableHinge() {

    //int curr_node1 = nodes[4].get_label();
    //int curr_node2 = nodes[5].get_label();
    //int joint_label = nacelle_label + 1 + 200; // jointのラベルがdeformable jointと一致しないようにする
    // jointの位置を指定する際の座標系、node1の座標系を使用
    //Frame base_frame = nodes[4].get_frame();
    // その座標系からのoffset拘束点はnode1の場所とするので、offsetはなし
    //Frame offset = offset_null;
    //ReferenceFrame joint_position = ReferenceFrame(joint_label, base_frame, offset);

    //Vec3d offset_e1 = relative_frame.get_vector_e1();
    //Vec3d offset_e3 = relative_frame.get_vector_e3();
    //total_joints[i] = TotalJoint(joint_label, curr_node1, curr_node2, joint_position, K, C, 0);

//}
    
void
RNA::set_total_joint() {
    for(int i=0; i<num_total_joints; i++) {
        //連続するnodeをtotal jointで拘束、解析初期時間が終わると拘束を開放する。

        int curr_node1;
        int curr_node2;
        Frame base_frame;

        if(i<2) {
            curr_node1 = nodes[i].get_label();
            curr_node2 = nodes[i+1].get_label();
            base_frame = nodes[i].get_frame();
        }

        if(i==2) {
            curr_node1 = nodes[0].get_label();
            curr_node2 = nodes[i+1].get_label();
            base_frame = nodes[0].get_frame();
        }
        
        if(i==3) {
            curr_node1 = nodes[5].get_label();
            curr_node2 = nodes[6].get_label();
            base_frame = nodes[5].get_frame();
        }

        if(i==4) {
            curr_node1 = nodes[i].get_label();
            curr_node2 = nodes[7].get_label();
            base_frame = nodes[i].get_frame();
        }

        if(i==5) {
            curr_node1 = nodes[7].get_label();
            curr_node2 = nodes[8].get_label();
            base_frame = nodes[7].get_frame();
        }

        if(i>5 && i<num_total_joints) {
            curr_node1 = nodes[8].get_label();
            curr_node2 = nodes[i+3].get_label();
            base_frame = nodes[8].get_frame();
        }

        int joint_label = nacelle_label + i + 1 + 100; // jointのラベルがdeformable jointと一致しないようにする

        // jointの位置を指定する際の座標系、node1の座標系を使用
        
        // その座標系からのoffset拘束点はnode1の場所とするので、offsetはなし
        Frame offset = offset_null;

        ReferenceFrame joint_position = ReferenceFrame(joint_label, base_frame, offset);
        total_joints[i] = TotalJoint(joint_label, curr_node1, curr_node2, joint_position, "Total", 0);
    }
}

void
RNA::write_reference_in(std::ofstream &output_file) const {
    //output_file<<"#-----Tower Top Reference------"<<std::endl;
    //tower_top_reference.write_reference(output_file);

    output_file<<"#----RNA node reference----"<<std::endl;

    for(const ReferenceFrame &flm : references) {
        flm.write_reference(output_file);
    }
}

void
RNA::write_nodes_in(std::ofstream &output_file) const {
    output_file << "#----RNA node-----" << std::endl;
    for(const Node &nod : nodes) {
        nod.write_node(output_file);
    }

    //output_file << "#----Tower top node----" << std::endl;
    //tower_top_node.write_node(output_file);
}

void
RNA::write_elements_in(std::ofstream &output_file) const {

    output_file << "#----RNA Rigid Bodies-----" <<std::endl;
    for(const RigidBody &rbd : rigidbodies){
        rbd.write_in_file(output_file);
    }
    //output_file << "#----Tower defomable joint-----" <<std::endl;
    //for(const DeformableJoint &dfj : deformable_joints) {
        //dfj.write_in_file(output_file);
    //}
    output_file << "#----RNA total joint-----" <<std::endl;
    for(const TotalJoint &ttj : total_joints ) {
        output_file << "driven :"<<ttj.get_label()<<", " <<"string, \"Time <= 0.0125\""  <<", " << std::endl;
        ttj.write_in_file(output_file);
    }

    //output_file << "#----Tower top initial total joint-----" <<std::endl;
    //output_file << "driven :"<<tower_label + 500 + 1<<", " <<"string, \"Time <= 0.0125\""  <<", " << std::endl;
    //tower_top_joint.write_in_file(output_file);
}

int 
RNA::get_num_nodes() const {
    // nacelle nodes 
    return num_nodes ;
}

int 
RNA::get_num_rigid_bodies() const {
    return num_rigidbodies;
}

int
RNA::get_num_joints() const {
    // RNA joints 
    return num_total_joints ;
}