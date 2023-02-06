#ifndef __Nacelle_H_INCLUDE__
#define __Nacelle_H_INCLUDE__

#include "Vec3d.h"
#include "Mat3x3d.h"

#include "InputData.h"
#include "Node.h"
#include "RigidBody.h"
#include "Joint.h"


class  Nacelle {
    private:
    const int nacelle_label; 
    ReferenceFrame nacelle_base_reference;
    //pointer of inputdata object
    InputData *inputdata;

    int num_reference;
    int num_nodes;
    int num_rigidbodies;
    int num_joints;
    int num_blds;
    double small_number;

    // tower top node
    Node tower_top_node;

    //------------------------------------------------
    // nacelle
    ReferenceFrame nacelle_reference;
    ReferenceFrame nacelle_CM_reference;
    Node           nacelle_node;
    RigidBody      nacelle_rigidbody;
    double         nacelle_CMx;
    double         nacelle_CMz;
    double         nacelle_mass;
    Vec3d          nacelle_inertia;

    // HSS 
    int            HSS_label;
    ReferenceFrame HSS_reference;
    ReferenceFrame HSS_CM_reference;
    Node           HSS_node;
    RigidBody      HSS_rigidbody;
    double         HSS_length;
    double         shaft_tilt_angle;
    double         tower_to_shaft;
    double         HSS_mass;
    Vec3d          HSS_inertia;

    // LSS 
    int            LSS_label;
    ReferenceFrame LSS_reference;
    ReferenceFrame LSS_CM_reference;
    Node           LSS_node;
    RigidBody      LSS_rigidbody;
    double         LSS_length;
    double         LSS_mass;
    Vec3d          LSS_inertia;

    // Generator
    int            Generator_label;
    ReferenceFrame Generator_reference;
    ReferenceFrame Generator_CM_reference;
    Node           Generator_node;
    RigidBody      Generator_rigidbody;
    double         Generator_length;
    double         Generator_mass;
    Vec3d          Generator_inertia;

    // Hub
    int            Hub_label;
    ReferenceFrame Hub_reference;
    ReferenceFrame Hub_CM_reference;
    Node           Hub_node;
    RigidBody      Hub_rigidbody;
    double         Overhang;
    double         Hub_mass;
    Vec3d          Hub_inertia;

    DummyNode      ShaftCS;
    DummyNode      NacelleHubRef;
    DummyNode      RotAzim;
    DummyNode      GenAzim;

    // Pitch Plate
    struct PitchPlate {
        ReferenceFrame inline_shaft_reference;
        ReferenceFrame preconed_reference;
        int            label;
        ReferenceFrame reference;
        ReferenceFrame CM_reference;
        Node           node;
        RigidBody      rigidbody;    
        double         mass;
        Vec3d          inertia;    
    };

    std::vector<PitchPlate> Pitch_Bottom;
    double          preconed_angle;

    //---------joint----------------------------------
    // yaw bearing
    int            yaw_bearing_label;
    ReferenceFrame yaw_bearing_reference;
    RevoluteJoint  yaw_bearing_revolute;
    TotalJoint     yaw_bearing_total;

    // HSS_bearing
    int            HSS_bearing_label;
    ReferenceFrame HSS_bearing_reference;
    RevoluteJoint  HSS_bearing_revolute;
    double         init_rot_speed;
    TotalJoint     HSS_bearing_lock;

    // HSS_Generator
    int            HSS_Generator_label;
    ReferenceFrame HSS_Generator_reference;
    TotalJoint     HSS_Generator_total;

    // HSS-LSS bearing
    int            HSS_LSS_bearing_label;
    ReferenceFrame HSS_LSS_bearing_reference;
    RevoluteJoint  HSS_LSS_bearing_revolute;
    TotalJoint     HSS_LSS_bearing_lock;

    DeformableHinge drivetrain_defomablehinge;
    double         dtrain_spring_constant;
    double         dtrain_dumping_constant;

    // LSS-Hub
    int            LSS_Hub_label;
    ReferenceFrame LSS_Hub_reference;
    TotalJoint     LSS_Hub_total;

    // pitchbottom
    struct  PitchBottom
    {
        int            label;
        ReferenceFrame reference;
        TotalJoint     total;
    };

    std::vector<PitchBottom> Pitch_bottom_total;
    double                   Hub_radius;


    public:
    // コンストラクタ：初期化の時点で、ラベルとinputDataのポインタを要求する
    Nacelle(int nacelle_label, const Node &tower_top, InputData *InputData);
    // デストラクタ：動的なメンバ変数を破棄するが、今回は何もしない。
    ~Nacelle(){};

    public:
    void write_reference_in(std::ofstream &output_file) const;
    void write_nodes_in(std::ofstream &ofs) const;
    void write_rigidbodies_in(std::ofstream &ofs) const;
    void write_joints_in(std::ofstream &ofs) const;

    //const ReferenceFrame get_top_reference() const {return Bld_base_reference1;};

    const ReferenceFrame get_top_reference(int i) const ;


    int get_num_nodes() const;
    int get_num_rigid_bodies() const;
    int get_num_joints() const;

    private:
    void set_reference();
    void set_nodes();
    void set_rigidbodies();
    void set_joints();
};



#endif