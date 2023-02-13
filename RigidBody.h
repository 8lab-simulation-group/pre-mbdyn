#ifndef __RIGIDBODY_H_INCLUDE__
#define __RIGIDBODY_H_INCLUDE__

#include "Element.h"

class RigidBody : public Element 
{
    private:
    double mass;
    Vec3d cm_offset;
    ReferenceFrame cm_offset_frame;
    Vec3d inertia_moment;

    std::string cm_style;

    public:
    RigidBody(){};
    RigidBody(int label, int node_label, double m, const Vec3d          &cm, const Vec3d &inertia ,int outflag);
    RigidBody(int label, int node_label, double m, const ReferenceFrame &cm, const Vec3d &inertia ,int outflag);
    RigidBody(int label, Node &node,     double m, const Vec3d          &cm, const Vec3d &inertia ,int outflag);
    RigidBody(int label, Node &node,     double m, const ReferenceFrame &cm, const Vec3d &inertia ,int outflag);
    RigidBody(const RigidBody &rb);
    ~RigidBody(){};

    RigidBody &operator=(const RigidBody &rb);
    void write_in_file(std::ofstream &ofs) const override; 
    void print_element() const override ;

};



#endif