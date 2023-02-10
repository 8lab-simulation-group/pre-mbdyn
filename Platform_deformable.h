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

    // ground node と clamp joint
    Frame ground_reference;
    StaticNode ground_node;
    Node ptfm_top_node;
    ClampJoint ground_joint;
    TotalJoint ptfm_top_joint;
    TotalJoint ptfm_lock;

    struct Section {
        ReferenceFrame reference;
        Node           Node;
        RigidBody      rigidbody;
    };

    struct Connector {
        DeformableJoint defomable_joint;
        TotalJoint      total_joint;
    };

    std::vector<Section> m_section;
    std::vector<Connector> m_connector;

    public:
    // コンストラクタ：初期化の時点で、ラベルとinputDataのポインタを要求する
    Platform(int platform_label,  InputData *InputData);
    // デストラクタ：動的なメンバ変数を破棄するが、今回は何もしない。
    ~Platform(){};

    public:
    void write_reference_in(std::ofstream &output_file) const;
    void write_nodes_in(std::ofstream &ofs) const;
    void write_rigidbodies_in(std::ofstream &ofs) const;
    void write_joints_in(std::ofstream &ofs) const;
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
