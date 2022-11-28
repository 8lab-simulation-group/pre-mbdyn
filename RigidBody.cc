#include "RigidBody.h"

RigidBody::RigidBody(int label, int nodelabel, double m ,const Vec3d &cm, const Vec3d &iner, int outflg) 
: Element(1) // rigid bodyのノード数は１ 
{
    elem_label = label;
    node_label[0] = nodelabel;
    mass = m;
    cm_offset = cm;
    inertia_moment = iner;
    out_flg = outflg;
}

RigidBody::RigidBody(const RigidBody &rb) 
: Element(rb) {
    mass = rb.mass;
    cm_offset = rb.cm_offset;
    inertia_moment = rb.inertia_moment;
}

RigidBody &
RigidBody::operator=(const RigidBody &rb) {
    Element::operator=(rb);
    mass = rb.mass;
    cm_offset = rb.cm_offset;
    inertia_moment = rb.inertia_moment;   
    return *this;
}

void 
RigidBody::print_element() const {
    std::cout<<"rigid body    :"<<elem_label<<std::endl
             <<"node          :"<<node_label[0]<<std::endl
             <<"mass          :"<<mass<<std::endl
             <<"cm            :"<<cm_offset<<std::endl
             <<"inertia_moment:"<<inertia_moment<<std::endl
             <<"output        :"<<out_flg<<std::endl<<std::endl;
}

void 
RigidBody::write_in_file(std::ofstream &ofs) const  {

    ofs << std::scientific<<std::setprecision(5)
        << "body :" << elem_label << "," << std::endl
        << "    " << node_label[0] << ", " <<std::endl
        << "    " << mass << ", " << std::endl
        << "    " << cm_offset << ", "<<std::endl
        << "    diag, " << inertia_moment << ", "<<std::endl
        << "    output, " << out_flg << ";"<<std::endl<<std::endl;
}