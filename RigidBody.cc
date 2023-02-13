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

    cm_style ="vector";
}

RigidBody::RigidBody(int label, int nodelabel, double m ,const ReferenceFrame &cm, const Vec3d &iner, int outflg) 
: Element(1) // rigid bodyのノード数は１ 
{
    elem_label = label;
    node_label[0] = nodelabel;
    mass = m;
    cm_offset_frame = cm;
    inertia_moment = iner;
    out_flg = outflg;

    cm_style = "reference";
}

RigidBody::RigidBody(int label, Node &node, double m ,const Vec3d &cm, const Vec3d &iner, int outflg) 
: Element(1) // rigid bodyのノード数は１ 
{
    elem_label = label;
    node_label[0] = node.get_label();
    mass = m;
    cm_offset = cm;
    inertia_moment = iner;
    out_flg = outflg;

    cm_style = "vector";
}

RigidBody::RigidBody(int label, Node &node, double m ,const ReferenceFrame &cm, const Vec3d &iner, int outflg) 
: Element(1) // rigid bodyのノード数は１ 
{
    elem_label = label;
    node_label[0] = node.get_label();
    mass = m;
    cm_offset_frame = cm;
    inertia_moment = iner;
    out_flg = outflg;

    cm_style ="reference";
}

RigidBody::RigidBody(const RigidBody &rb) 
: Element(rb) {
    mass = rb.mass;
    cm_offset = rb.cm_offset;
    cm_offset_frame = rb.cm_offset_frame;
    inertia_moment = rb.inertia_moment;
    cm_style = rb.cm_style;
}

RigidBody &
RigidBody::operator=(const RigidBody &rb) {
    Element::operator=(rb);
    mass = rb.mass;
    cm_offset = rb.cm_offset;
    cm_offset_frame = rb.cm_offset_frame;
    inertia_moment = rb.inertia_moment;   
    cm_style = rb.cm_style;
    return *this;
}

void 
RigidBody::print_element() const {
    if(cm_style=="vector") {
        std::cout << std::scientific<<std::setprecision(5)
            << "body :" << elem_label << "," << std::endl
            << "    " << node_label[0] << ", " <<std::endl
            << "    " << mass << ", " << std::endl
            << "    " << cm_offset << ", "<<std::endl
            << "    diag, " << inertia_moment << ", "<<std::endl
            << "    output, " << out_flg << ";"<<std::endl<<std::endl;
    }
    else if(cm_style=="reference") {
        std::cout << std::scientific<<std::setprecision(5)
            << "body :" << elem_label << "," << std::endl
            << "    " << node_label[0] << ", " <<std::endl
            << "    " << mass << ", " << std::endl
            << "    reference," << cm_offset_frame.get_label() << ", null,"<<std::endl
            << "    diag, " << inertia_moment << ", "<<std::endl
            << "    output, " << out_flg << ";"<<std::endl<<std::endl;
    }
}

void 
RigidBody::write_in_file(std::ofstream &ofs) const  {
    if(cm_style=="vector") {
        ofs << std::scientific<<std::setprecision(5)
            << "body :" << elem_label << "," << std::endl
            << "    " << node_label[0] << ", " <<std::endl
            << "    " << mass << ", " << std::endl
            << "    " << cm_offset << ", "<<std::endl
            << "    diag, " << inertia_moment << ", "<<std::endl
            << "    output, " << out_flg << ";"<<std::endl<<std::endl;
    }
    else if(cm_style=="reference") {
        ofs << std::scientific<<std::setprecision(5)
            << "body :" << elem_label << "," << std::endl
            << "    " << node_label[0] << ", " <<std::endl
            << "    " << mass << ", " << std::endl
            << "    reference," << cm_offset_frame.get_label() << ", null,"<<std::endl
            << "    diag, " << inertia_moment << ", "<<std::endl
            << "    output, " << out_flg << ";"<<std::endl<<std::endl;
    }
}