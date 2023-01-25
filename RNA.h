#ifndef __RNA_H_INCLUDE__
#define __RNA_H_INCLUDE__

#include "Vec3d.h"
#include "Mat3x3d.h"

#include "InputData.h"
#include "Node.h"
#include "RigidBody.h"
#include "Joint.h"


class  RNA {
    private:
    const int nacelle_label; 
    ReferenceFrame nacelle_base_reference;
    //pointer of inputdata object
    InputData *inputdata;

    int num_reference;
    int num_nodes;
    int num_rigidbodies;
    int num_deformable_hinge;
    int num_revolute_hinges;
    int num_total_joints;
    int num_joints;
    int num_blds;

    double Hub_CM_x;
    double Hub_CM_z;
    double Bld_downwind_distance1;
    double Bld_height1;
    double Bld_y1;
    double Bld_downwind_distance2;
    double Bld_height2;
    double Bld_y2;
    double Bld_downwind_distance3;
    double Bld_height3;
    double Bld_y3;

    Mat3x3d Rgprime_Shft;
    Mat3x3d Ri_gprime; 
    Mat3x3d Rshft_rotorfurl;
    Mat3x3d R1;
    Mat3x3d R2;
    Mat3x3d R3;
    Vec3d Xhub;
    Vec3d PitchPlate1;
    Vec3d PitchPlate2;
    Vec3d PitchPlate3;


    Frame RNA_reference;
    Frame Bld_base_node1;
    Frame Bld_base_node2;
    Frame Bld_base_node3;
    ReferenceFrame Bld_base_reference1;
    ReferenceFrame Bld_base_reference2;
    ReferenceFrame Bld_base_reference3;
    TotalJoint RNA_top_joint;

    // Towerを構成するノードとエレメントの変数。vectorで宣言し、後ほど要素数に応じて容量を確保する。
    std::vector<ReferenceFrame> references;
    std::vector<Node>   nodes;
    std::vector<RigidBody>  rigidbodies;
    //std::vector<revoluteHinge> revolute_hinges;
    std::vector<DeformableHinge> deformable_hinge;
    //解析初期のみ、deformable jointを全てTotal jointで拘束して解析初期の安定性を確保している。
    std::vector<TotalJoint> total_joints;

    public:
    // コンストラクタ：初期化の時点で、ラベルとinputDataのポインタを要求する
    RNA(int nacelle_label, const ReferenceFrame &nacelle_base, InputData *InputData);
    // デストラクタ：動的なメンバ変数を破棄するが、今回は何もしない。
    ~RNA(){};

    public:
    void write_reference_in(std::ofstream &output_file) const;
    void write_nodes_in(std::ofstream &ofs) const;
    void write_elements_in(std::ofstream &ofs) const;

    //const ReferenceFrame get_top_reference() const {return Bld_base_reference;};
    int get_num_nodes() const;
    int get_num_rigid_bodies() const;
    int get_num_joints() const;

    private:
    void set_reference();
    void set_nodes();
    void set_rigidbodies();
    void set_deformablejoint();
    void set_total_joint();
};



#endif