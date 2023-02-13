#ifndef __BLADE_H_INCLUDE__
#define __BLADE_H_INCLUDE__

#include "Vec3d.h"
#include "Mat3x3d.h"

#include "InputData.h"
#include "Node.h"
#include "RigidBody.h"
#include "Joint.h"


class Blade {
    private:
    const int blade_label; 
    ReferenceFrame bld_base_reference1;
    //pointer of inputdata object
    InputData *inputdata;

    int num;
    int num_reference;
    int num_nodes;
    int num_rigidbodies;
    int num_deformable_joints;
    int num_total_joints;
    int num_joints;

    int num_bld;

    int own_bld_num;

    double InitConstraint = 0.0125;
    double BldFlexL;

    ReferenceFrame pitch_plate;

    // Bladeを構成するノードとエレメントの変数。vectorで宣言し、後ほど要素数に応じて容量を確保する。
    std::vector<ReferenceFrame> references;
    std::vector<ReferenceFrame> CM_references;
    std::vector<Node>   nodes;
    std::vector<RigidBody>  rigidbodies;
    std::vector<DeformableJoint> deformable_joints;

    std::vector<double> InterpBMass;
    std::vector<double> InterpEAStff;
    std::vector<double> InterpFlpStff;
    std::vector<double> InterpEdgStff;
    std::vector<double> InterpGJStff;
    std::vector<double> InterpdgIner;
    std::vector<double> InterpFlpIner;
    std::vector<double> InterpAeroCent;
    std::vector<double> InterpEdgcgOf;
    //解析初期のみ、deformable jointを全てTotal jointで拘束して解析初期の安定性を確保している。
    std::vector<TotalJoint> total_joints;

    public:
    // コンストラクタ：初期化の時点で、ラベルとinputDataのポインタを要求する
    Blade(int own_bld_num,int blade_label, const ReferenceFrame &bladebase, InputData *InputData);
    // デストラクタ：動的なメンバ変数を破棄するが、今回は何もしない。
    ~Blade(){};

    public:
    void write_reference_in(std::ofstream &output_file) const;
    void write_nodes_in(std::ofstream &ofs) const;

    void write_rigidbodies_in(std::ofstream &ofs) const;
    void write_joints_in(std::ofstream &ofs) const;

    //const ReferenceFrame get_top_reference() const {return tower_top_reference;};
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