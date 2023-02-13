#ifndef __JOINT_H_INCLODE__
#define __JOINT_H_INCLODE__

#include "Element.h"
#include "Vec3d.h"
#include "Mat3x3d.h"
#include "Mat6x6d.h"


class Joint : public Element {
    protected:
    ReferenceFrame constraint_position;

    public:
    Joint(){};
    Joint(int label, int nodelabel1,  int nodelabel2, const ReferenceFrame &position, int outflg );
    Joint(const Joint &jnt);
    ~Joint(){};

    Joint &operator=(const Joint &jnt);
    virtual void write_in_file(std::ofstream &ofs) const override{};
    virtual void print_element() const override{};
    int get_label() const {return elem_label;};

};

class TotalJoint : public Joint {
    private:
    std::string constraint_condition;
    Vec3d init_angular_velocity;
    public:

    TotalJoint(){};
    TotalJoint(int label, int nodelabel1,  int nodelabel2, const ReferenceFrame &position, const std::string &condition ,int outflg, const Vec3d &angular_velocit = zero3);
    TotalJoint(int label, Node  &node1,  Node &node2, const ReferenceFrame &position, const std::string &condition ,int outflg, const Vec3d &angular_velocit = zero3);
    TotalJoint(const TotalJoint &totaljnt);
    ~TotalJoint(){};

    TotalJoint &operator=(const TotalJoint &totaljnt);
    void write_in_file(std::ofstream &ofs) const override;
    void print_element() const override;    
};

class RevoluteJoint : public Joint {
    private:
    double init_theta;
    public:
    RevoluteJoint(){};
    RevoluteJoint(int label, int nodelabel1,  int nodelabel2, const ReferenceFrame &position,const double init_theta, int outflg );
    RevoluteJoint(int label, Node &node1,     Node &node2,    const ReferenceFrame &position,const double init_theta, int outflg );
    RevoluteJoint(const RevoluteJoint &revolutejnt);
    ~RevoluteJoint(){};

    RevoluteJoint& operator=(const RevoluteJoint &revolutejnt);
    void write_in_file(std::ofstream &ofs) const override;
    void print_element() const override;     
};

class DeformableJoint : public Joint {
    private:
    Mat6x6d Kmatrix;
    Mat6x6d Cmatrix;
    public:
    DeformableJoint(){};
    DeformableJoint(int label, int nodelabel1,  int nodelabel2, const ReferenceFrame &position, const Mat6x6d &K, const Mat6x6d &C, int outflg );
    DeformableJoint(const DeformableJoint &defomablejnt);
    ~DeformableJoint(){};

    DeformableJoint &operator=(const DeformableJoint &deformalejnt);
    void write_in_file(std::ofstream &ofs) const override;
    void print_element() const override;    
};

class DeformableHinge : public Joint {
    private:
    double spring_coefficient;
    double dumper_coefficient;

    public:
    DeformableHinge(){};
    DeformableHinge(int label, int nodelabel1,  int nodelabel2, const ReferenceFrame &position, const double K, const double C, int outflg );
    DeformableHinge(int label, Node &node1, Node &node2, const ReferenceFrame &position, const double K, const double C, int outflg );
    DeformableHinge(const DeformableHinge &deformablehinge);
    ~DeformableHinge(){};

    DeformableHinge &operator=(const DeformableHinge &deformablehinge);
    void write_in_file(std::ofstream &ofs) const override;
    void print_element() const override;  
};

class ClampJoint {
    private:
    int elem_label;
    int node_label;
    int output_flag;
    public:
    ClampJoint(){};
    ClampJoint(int elemlabel, int node_abel, int output_flag);
    ClampJoint(const ClampJoint &cjt);

    ClampJoint &operator=(const ClampJoint &cjt);
    void write_in_file(std::ofstream &ofs) const;
};

#endif