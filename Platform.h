#ifndef __PLATFORM_H_INCLUDE__
#define __PLATFORM_H_INCLUDE__

#include "Vec3d.h"
#include "Mat3x3d.h"

#include "InputData.h"
#include "Node.h"
#include "RigidBody.h"
#include "Joint.h"


class Platform {
    private:
    // platfrom全体のラベル、node 等にplatform_label + 1 .... でラベルを与える
    const int platform_label;

    //pointer of inputdata object
    InputData *inputdata;

    int num_reference;
    int num_nodes;
    int num_rigidbodies;
    int num_deformable_joints;
    int num_total_joints;
    int num_joints;

    double InitConstraint = 0.0125;

    //double TwrFlexL;
    //double TwrHt;
    //double TwrDft;
    //double TwrRBHt;
    //double TwrNodes;
    //double DHNodes;
    //double HtFract;
    //double TwFAM1Sh;
    //double Deriv;

    Vec3d init_displacement;
    Vec3d init_euler321;
    Frame ptfm_reference;
    ReferenceFrame ptfm_top_reference;
    ReferenceFrame ptfm_top_ref;

    // ground node と clamp joint
    Frame ground_reference;
    StaticNode ground_node;
    Node ptfm_top_node;
    ClampJoint ground_joint;
    TotalJoint ptfm_top_joint;

    // プラットフォームを構成するノードとエレメントの変数。vectorで宣言し、後ほど要素数に応じて容量を確保する。
    std::vector<ReferenceFrame> references;
    std::vector<Node>   nodes;
    std::vector<RigidBody>  rigidbodies;
    std::vector<DeformableJoint> deformable_joints;
    //解析初期のみ、deformable jointを全てTotal jointで拘束して解析初期の安定性を確保している。
    std::vector<TotalJoint> total_joints;

    public:
    // コンストラクタ：初期化の時点で、ラベルとinputDataのポインタを要求する
    Platform(int platform_label,  InputData *InputData);
    // デストラクタ：動的なメンバ変数を破棄するが、今回は何もしない。
    ~Platform(){};

    public:
    void write_reference_in(std::ofstream &output_file) const;
    void write_nodes_in(std::ofstream &ofs) const;
    void write_elements_in(std::ofstream &ofs) const;
    //void SHP(double HtFract,double TwrFlexL,double TwFAM1Sh,double Deriv);
    //void Interpolution();

    const ReferenceFrame get_top_reference() const {return ptfm_top_reference;};
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
