#ifndef __TOWER_H_INCLUDE__
#define __TOWER_H_INCLUDE__

#include "Vec3d.h"
#include "Mat3x3d.h"

#include "InputData.h"
#include "Node.h"
#include "RigidBody.h"
#include "Joint.h"


class Tower {
    private:
    const int tower_label; 
    ReferenceFrame tower_base_reference;
    //pointer of inputdata object
    InputData *inputdata;

    int num_reference;
    int num_nodes;
    int num_rigidbodies;
    int num_deformable_joints;
    int num_total_joints;
    int num_joints;

    double TwrHt;
    double TwrDft;
    double TwrRBHt;
    double TwrFlexL;
    double DHNodes;
    double InitConstraint = 0.0125;

    Node tower_base_node;
    TotalJoint tower_top_joint;

    TotalJoint ptfmtop_to_twrbase;

    // Towerを構成するノードとエレメントの変数。vectorで宣言し、後ほど要素数に応じて容量を確保する。
    std::vector<ReferenceFrame> references;
    std::vector<Node>   nodes;
    std::vector<RigidBody>  rigidbodies;
    std::vector<DeformableJoint> deformable_joints;
    //解析初期のみ、deformable jointを全てTotal jointで拘束して解析初期の安定性を確保している。
    std::vector<TotalJoint> total_joints;

    Frame Tower_reference;
    ReferenceFrame tower_top_reference;
    ReferenceFrame tower_top_ref;
    Node tower_top_node;

    public:
    // コンストラクタ：初期化の時点で、ラベルとinputDataのポインタを要求する
    Tower(int tower_label, const ReferenceFrame &towerbase, InputData *InputData);
    // デストラクタ：動的なメンバ変数を破棄するが、今回は何もしない。
    ~Tower(){};

    public:
    void write_reference_in(std::ofstream &output_file) const;
    void write_nodes_in(std::ofstream &ofs) const;
    void write_elements_in(std::ofstream &ofs) const;
    void write_rigidbodies_in(std::ofstream &ofs) const;
    void write_joints_in(std::ofstream &ofs) const;

    const ReferenceFrame get_top_reference() const {return tower_top_reference;};
    const Node get_top_node() const {return tower_top_node;};
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