#include "Joint.h"

Joint::Joint(int label, int nodelabel1,  int nodelabel2, const ReferenceFrame &position, int outflg) 
: Element(2) //nodeを2個領域を確保
{
    elem_label = label;
    node_label.resize(2);
    node_label[0] = nodelabel1;
    node_label[1] = nodelabel2;
    out_flg = outflg;

    constraint_position = position;
}

Joint::Joint(const Joint &jnt) 
: Element(jnt)
{
    constraint_position = jnt.constraint_position;
}

Joint&
Joint::operator=(const Joint &jnt) {
    Element::operator=(jnt);
    constraint_position = jnt.constraint_position;
    return *this;
}

// -------------------definition of Total joint-----------------------
TotalJoint::TotalJoint(int label, int nodelabel1,  int nodelabel2, const ReferenceFrame &position,const std::string &condition ,int outflg,const Vec3d &angular_v)
: Joint(label, nodelabel1, nodelabel2, position, outflg)
{
    constraint_condition = condition;
    init_angular_velocity = angular_v;
}

TotalJoint::TotalJoint(const TotalJoint &totaljnt)
: Joint(totaljnt)
{
    constraint_condition = totaljnt.constraint_condition;
    init_angular_velocity = totaljnt.init_angular_velocity;    
}

TotalJoint &
TotalJoint::operator=(const TotalJoint &totaljnt){
    Joint::operator=(totaljnt);
    constraint_condition = totaljnt.constraint_condition;
    init_angular_velocity = totaljnt.init_angular_velocity;   
    return *this;    
}
void 
TotalJoint::write_in_file(std::ofstream &ofs) const {
    int base_label = constraint_position.get_base_frame().get_label();
    Frame relative_frame = constraint_position.get_relative_frame();  
    Vec3d offset_position = relative_frame.get_position();
    Vec3d offset_e1 = relative_frame.get_vector_e1();
    Vec3d offset_e3 = relative_frame.get_vector_e3();

    if(constraint_condition == "Total") {
        ofs << std::scientific<<std::setprecision(5) << std::endl
            << "joint :" << elem_label << "," <<std::endl
            << "    total joint," << std::endl
            << "    " << node_label[0] << "," <<std::endl
            << "    position,             reference, " << base_label << ", "<<offset_position << ","<< std::endl
            << "    position orientation, reference, " << base_label << "," << std::endl
            << "                            1, " << offset_e1 << "," <<std::endl
            << "                            3, " << offset_e3 << "," <<std::endl
            << "    rotation orientation, reference, " << base_label << "," << std::endl
            << "                            1, " << offset_e1 << "," <<std::endl
            << "                            3, " << offset_e3 << "," <<std::endl
            << "    " << node_label[1] << "," <<std::endl
            << "    position,             reference, " << base_label << ", "<<offset_position << ","<< std::endl
            << "    position orientation, reference, " << base_label << "," << std::endl
            << "                            1, " << offset_e1 << "," <<std::endl
            << "                            3, " << offset_e3 << "," <<std::endl
            << "    rotation orientation, reference, " << base_label << "," << std::endl
            << "                            1, " << offset_e1 << "," <<std::endl
            << "                            3, " << offset_e3 << "," <<std::endl
            << "    position constraint,    active, active, active, null," <<std::endl
            << "    orientation constraint, active, active, active, null," <<std::endl
            << "    output, " << out_flg << ";"<<std::endl<<std::endl;

    } else if (constraint_condition == "Angular Velocity") {
        ofs << std::scientific<<std::setprecision(5) << std::endl
            << "joint :" << elem_label << "," <<std::endl
            << "    total joint," << std::endl
            << "    " << node_label[0] << "," <<std::endl
            << "    position,             reference, " << base_label << ", "<<offset_position << ","<< std::endl
            << "    position orientation, reference, " << base_label << "," << std::endl
            << "                            1, " << offset_e1 << "," <<std::endl
            << "                            3, " << offset_e3 << "," <<std::endl
            << "    rotation orientation, reference, " << base_label << "," << std::endl
            << "                            1, " << offset_e1 << "," <<std::endl
            << "                            3, " << offset_e3 << "," <<std::endl
            << "    " << node_label[1] << "," <<std::endl
            << "    position,             reference, " << base_label << ", "<<offset_position << ","<< std::endl
            << "    position orientation, reference, " << base_label << "," << std::endl
            << "                            1, " << offset_e1 << "," <<std::endl
            << "                            3, " << offset_e3 << "," <<std::endl
            << "    rotation orientation, reference, " << base_label << "," << std::endl
            << "                            1, " << offset_e1 << "," <<std::endl
            << "                            3, " << offset_e3 << "," <<std::endl
            << "    position constraint,    inactive, inactive, inactive, null," <<std::endl
            << "    orientation constraint, inactive, inactive, active," <<std::endl
            << "    component, const, "<<init_angular_velocity[0]<<", const, "<<init_angular_velocity<<", const, "<<init_angular_velocity[2]<<","<<std::endl
            << "    output, " << out_flg << ";"<<std::endl<<std::endl;        
    }
}   
    
void 
TotalJoint::print_element() const{
    int base_label = constraint_position.get_base_frame().get_label();
    Frame relativfe_frame = constraint_position.get_relative_frame();
    
    if(constraint_condition=="Total") {

        std::cout<<"joint label :"<<elem_label<<std::endl
                 <<"total joint"<<std::endl
                 <<"node 1      :"<<node_label[0]<<std::endl
                 <<"position             :reference :"<<base_label<<": "<<relativfe_frame.get_position()<<std::endl
                 <<"position orientation :reference :"<<base_label<<": "<<relativfe_frame.get_rotation_global_to_local()<<std::endl
                 <<"rotation orientation :reference :"<<base_label<<": "<<relativfe_frame.get_rotation_global_to_local()<<std::endl
                 <<"node 2      :"<<node_label[1]<<std::endl
                 <<"position             :reference :"<<base_label<<": "<<relativfe_frame.get_position()<<std::endl
                 <<"position orientation :reference :"<<base_label<<": "<<relativfe_frame.get_rotation_global_to_local()<<std::endl
                 <<"rotation orientation :reference :"<<base_label<<": "<<relativfe_frame.get_rotation_global_to_local()<<std::endl
                 <<"positon constraint, active, active, active, null"<<std::endl
                 <<"orientation constraint, active, active, active, null"<<std::endl<<std::endl;

    }else if(constraint_condition=="Anglar Velocity") {

        std::cout<<"joint label :"<<elem_label<<std::endl
                 <<"total joint"<<std::endl
                 <<"node 1      :"<<node_label[0]<<std::endl
                 <<"position             :reference :"<<base_label<<": "<<relativfe_frame.get_position()<<std::endl
                 <<"position orientation :reference :"<<base_label<<": "<<relativfe_frame.get_rotation_global_to_local()<<std::endl
                 <<"rotation orientation :reference :"<<base_label<<": "<<relativfe_frame.get_rotation_global_to_local()<<std::endl
                 <<"node 2      :"<<node_label[1]<<std::endl
                 <<"position             :reference :"<<base_label<<": "<<relativfe_frame.get_position()<<std::endl
                 <<"position orientation :reference :"<<base_label<<": "<<relativfe_frame.get_rotation_global_to_local()<<std::endl
                 <<"rotation orientation :reference :"<<base_label<<": "<<relativfe_frame.get_rotation_global_to_local()<<std::endl
                 <<"positon constraint, inctive, inactive, inactive, null"<<std::endl
                 <<"orientation constraint, inactive, inactive, active,"<<std::endl
                 <<"component, const,"<<init_angular_velocity[0]<<"const,"<<init_angular_velocity[1]<<"const,"<<init_angular_velocity[2]<<std::endl<<std::endl;

    }
} 

// -----------------------definition of Revolutehinge------------------------------
RevoluteJoint::RevoluteJoint(int label, int nodelabel1,  int nodelabel2, const ReferenceFrame &position, const double inittheta,int outflg )
: Joint(label, nodelabel1, nodelabel2, position, outflg) {
    init_theta = inittheta;
};

RevoluteJoint::RevoluteJoint(const RevoluteJoint &revolutejnt)
: Joint(revolutejnt){
    init_theta = revolutejnt.init_theta;
};

RevoluteJoint &
RevoluteJoint::operator=(const RevoluteJoint &revolutejnt) {
    Joint::operator=(revolutejnt);
    init_theta = revolutejnt.init_theta;
    return *this;
}
    
void 
RevoluteJoint::write_in_file(std::ofstream &ofs) const {   
    int base_label = constraint_position.get_base_frame().get_label();
    Frame relative_frame = constraint_position.get_relative_frame();  
    Vec3d offset_position = relative_frame.get_position();
    Vec3d offset_e1 = relative_frame.get_vector_e1();
    Vec3d offset_e3 = relative_frame.get_vector_e3();

    ofs << std::scientific<<std::setprecision(5)
        << "joint :" << elem_label << "," <<std::endl
        << "    revolute hinge," << std::endl
        << "    " << node_label[0] << "," <<std::endl
        << "    position,    reference, " << base_label << ", "<<offset_position << ","<< std::endl
        << "    orientation, reference, " << base_label << ","   << std::endl
        << "                            1, " << offset_e1 << "," <<std::endl
        << "                            3, " << offset_e3 << "," <<std::endl
        << "    " << node_label[1] << "," <<std::endl
        << "    position,    reference, " << base_label << ", "<<offset_position << ","<< std::endl
        << "    orientation, reference, " << base_label << "," << std::endl
        << "                            1, " << offset_e1 << "," <<std::endl
        << "                            3, " << offset_e3 << "," <<std::endl
        << "    output, " << out_flg << "," <<std::endl
        << "    initial theta, " << init_theta << ";" << std::endl << std::endl;
}

void 
RevoluteJoint::print_element() const {
    int base_label = constraint_position.get_base_frame().get_label();
    Frame relativfe_frame = constraint_position.get_relative_frame();
    
    std::cout<<"joint label :"<<elem_label<<std::endl
                 <<"Revolute hinge"<<std::endl
                 <<"node 1      :"<<node_label[0]<<std::endl
                 <<"position    :reference :"<<base_label<<": "<<relativfe_frame.get_position()<<std::endl
                 <<"orientation :reference :"<<base_label<<": "<<relativfe_frame.get_rotation_global_to_local()<<std::endl
                 <<"node 2      :"<<node_label[1]<<std::endl
                 <<"position    :reference :"<<base_label<<": "<<relativfe_frame.get_position()<<std::endl
                 <<"orientation :reference :"<<base_label<<": "<<relativfe_frame.get_rotation_global_to_local()<<std::endl<<std::endl;
}   

//----------------definition of deformble joint --------------------------------
DeformableJoint::DeformableJoint(int label, int nodelabel1,  int nodelabel2, const ReferenceFrame &position, const Mat6x6d &K, const Mat6x6d &C, int outflg )
: Joint(label, nodelabel1, nodelabel2, position, outflg) {
    Kmatrix = K;
    Cmatrix = C;
}

DeformableJoint::DeformableJoint(const DeformableJoint &deformablejnt) 
: Joint(deformablejnt)
{
    Kmatrix = deformablejnt.Kmatrix;
    Cmatrix = deformablejnt.Cmatrix;
}

DeformableJoint &
DeformableJoint::operator=(const DeformableJoint &deformablejnt){
    Joint::operator=(deformablejnt);
    Kmatrix = deformablejnt.Kmatrix;
    Cmatrix = deformablejnt.Cmatrix;    
    return *this;
}

void 
DeformableJoint::write_in_file(std::ofstream &ofs) const {
    int base_label = constraint_position.get_base_frame().get_label();
    Frame relative_frame = constraint_position.get_relative_frame();  
    Vec3d offset_position = relative_frame.get_position();
    Vec3d offset_e1 = relative_frame.get_vector_e1();
    Vec3d offset_e3 = relative_frame.get_vector_e3();

    ofs << std::scientific<<std::setprecision(5)
        << "joint :" << elem_label << "," <<std::endl
        << "    deformable joint," << std::endl
        << "    " << node_label[0] << "," <<std::endl
        << "    position,    reference, " << base_label << ", "<<offset_position << ","<< std::endl
        << "    orientation, reference, " << base_label << "," << std::endl
        << "                         1, " << offset_e1 << "," <<std::endl
        << "                         3, " << offset_e3 << "," <<std::endl
        << "    " << node_label[1] << "," <<std::endl
        << "    position,    reference, " << base_label << ", "<<offset_position << ","<< std::endl
        << "    orientation, reference, " << base_label << "," << std::endl
        << "                         1, " << offset_e1 << "," <<std::endl
        << "                         3, " << offset_e3 << "," <<std::endl
        << "    linear viscoelastic generic," << std::endl
        << "    "<<Kmatrix.get(0,0) <<", "<<Kmatrix.get(0,1) <<", "<<Kmatrix.get(0,2) <<", "<<Kmatrix.get(0,3) <<", "<<Kmatrix.get(0,4) <<", "<<Kmatrix.get(0,5) <<", "<< std::endl
        << "    "<<Kmatrix.get(1,0) <<", "<<Kmatrix.get(1,1) <<", "<<Kmatrix.get(1,2) <<", "<<Kmatrix.get(1,3) <<", "<<Kmatrix.get(1,4) <<", "<<Kmatrix.get(1,5) <<", "<< std::endl
        << "    "<<Kmatrix.get(2,0) <<", "<<Kmatrix.get(2,1) <<", "<<Kmatrix.get(2,2) <<", "<<Kmatrix.get(2,3) <<", "<<Kmatrix.get(2,4) <<", "<<Kmatrix.get(2,5) <<", "<< std::endl
        << "    "<<Kmatrix.get(3,0) <<", "<<Kmatrix.get(3,1) <<", "<<Kmatrix.get(3,2) <<", "<<Kmatrix.get(3,3) <<", "<<Kmatrix.get(3,4) <<", "<<Kmatrix.get(3,5) <<", "<< std::endl
        << "    "<<Kmatrix.get(4,0) <<", "<<Kmatrix.get(4,1) <<", "<<Kmatrix.get(4,2) <<", "<<Kmatrix.get(4,3) <<", "<<Kmatrix.get(4,4) <<", "<<Kmatrix.get(4,5) <<", "<< std::endl
        << "    "<<Kmatrix.get(5,0) <<", "<<Kmatrix.get(5,1) <<", "<<Kmatrix.get(5,2) <<", "<<Kmatrix.get(5,3) <<", "<<Kmatrix.get(5,4) <<", "<<Kmatrix.get(5,5) <<", "<< std::endl<<std::endl
        << "    "<<Cmatrix.get(0,0) <<", "<<Cmatrix.get(0,1) <<", "<<Cmatrix.get(0,2) <<", "<<Cmatrix.get(0,3) <<", "<<Cmatrix.get(0,4) <<", "<<Cmatrix.get(0,5) <<", "<< std::endl
        << "    "<<Cmatrix.get(1,0) <<", "<<Cmatrix.get(1,1) <<", "<<Cmatrix.get(1,2) <<", "<<Cmatrix.get(1,3) <<", "<<Cmatrix.get(1,4) <<", "<<Cmatrix.get(1,5) <<", "<< std::endl
        << "    "<<Cmatrix.get(2,0) <<", "<<Cmatrix.get(2,1) <<", "<<Cmatrix.get(2,2) <<", "<<Cmatrix.get(2,3) <<", "<<Cmatrix.get(2,4) <<", "<<Cmatrix.get(2,5) <<", "<< std::endl
        << "    "<<Cmatrix.get(3,0) <<", "<<Cmatrix.get(3,1) <<", "<<Cmatrix.get(3,2) <<", "<<Cmatrix.get(3,3) <<", "<<Cmatrix.get(3,4) <<", "<<Cmatrix.get(3,5) <<", "<< std::endl
        << "    "<<Cmatrix.get(4,0) <<", "<<Cmatrix.get(4,1) <<", "<<Cmatrix.get(4,2) <<", "<<Cmatrix.get(4,3) <<", "<<Cmatrix.get(4,4) <<", "<<Cmatrix.get(4,5) <<", "<< std::endl
        << "    "<<Cmatrix.get(5,0) <<", "<<Cmatrix.get(5,1) <<", "<<Cmatrix.get(5,2) <<", "<<Cmatrix.get(5,3) <<", "<<Cmatrix.get(5,4) <<", "<<Cmatrix.get(5,5) <<", "<< std::endl
        << "    output, " << out_flg << ";"<<std::endl<<std::endl;
}
void
DeformableJoint::print_element() const {
    int base_label = constraint_position.get_base_frame().get_label();
    Frame relativfe_frame = constraint_position.get_relative_frame();
    
    std::cout<<"joint label :"<<elem_label<<std::endl
                 <<"deformable joint"<<std::endl
                 <<"node 1      :"<<node_label[0]<<std::endl
                 <<"position    :reference :"<<base_label<<": "<<relativfe_frame.get_position()<<std::endl
                 <<"orientation :reference :"<<base_label<<": "<<relativfe_frame.get_rotation_global_to_local()<<std::endl
                 <<"node 2      :"<<node_label[1]<<std::endl
                 <<"position    :reference :"<<base_label<<": "<<relativfe_frame.get_position()<<std::endl
                 <<"orientation :reference :"<<base_label<<": "<<relativfe_frame.get_rotation_global_to_local()<<std::endl
                 <<"kMatrix"<<std::endl<<Kmatrix
                 <<"CMatrix"<<std::endl<<Cmatrix<<std::endl<<std::endl;
} 

// ------------------------- definition of deformablehinge---------------------------------
DeformableHinge::DeformableHinge(int label, int nodelabel1,  int nodelabel2, const ReferenceFrame &position, const double K, const double C, int outflg ) 
: Joint(label, nodelabel1, nodelabel2, position, outflg) 
{
    spring_coefficient = K;
    dumper_coefficient = C;
}

DeformableHinge::DeformableHinge(const DeformableHinge &deformablehinge) 
: Joint(deformablehinge)
{
    spring_coefficient = deformablehinge.spring_coefficient;
    dumper_coefficient = deformablehinge.dumper_coefficient;
}

DeformableHinge &
DeformableHinge::operator=(const DeformableHinge &deformablehinge) {
    Joint::operator=(deformablehinge);
    spring_coefficient = deformablehinge.spring_coefficient;
    dumper_coefficient = deformablehinge.dumper_coefficient;
    return *this;
}

void 
DeformableHinge::write_in_file(std::ofstream &ofs) const {   
    int base_label = constraint_position.get_base_frame().get_label();
    Frame relative_frame = constraint_position.get_relative_frame();  
    Vec3d offset_position = relative_frame.get_position();
    Vec3d offset_e1 = relative_frame.get_vector_e1();
    Vec3d offset_e3 = relative_frame.get_vector_e3();

    ofs << std::scientific<<std::setprecision(5)
        << "joint :" << elem_label << "," <<std::endl
        << "    revolute hinge," << std::endl
        << "    " << node_label[0] << "," <<std::endl
        << "    position,    reference, " << base_label << ", "<<offset_position << ","<< std::endl
        << "    orientation, reference, " << base_label << ","   << std::endl
        << "                            1, " << offset_e1 << "," <<std::endl
        << "                            3, " << offset_e3 << "," <<std::endl
        << "    " << node_label[1] << "," <<std::endl
        << "    position,    reference, " << base_label << ", "<<offset_position << ","<< std::endl
        << "    orientation, reference, " << base_label << "," << std::endl
        << "                            1, " << offset_e1 << "," <<std::endl
        << "                            3, " << offset_e3 << "," <<std::endl
        << "    linear viscoelastic, " << spring_coefficient << ", " << dumper_coefficient << "," <<std::endl
        << "    output, " << out_flg << ";" <<std::endl << std::endl;
}
    
void 
DeformableHinge::print_element() const {
    int base_label = constraint_position.get_base_frame().get_label();
    Frame relativfe_frame = constraint_position.get_relative_frame();
    std::cout<<"joint label :"<<elem_label<<std::endl
                 <<"deformable hinge"<<std::endl
                 <<"node 1      :"<<node_label[0]<<std::endl
                 <<"position    :reference :"<<base_label<<": "<<relativfe_frame.get_position()<<std::endl
                 <<"orientation :reference :"<<base_label<<": "<<relativfe_frame.get_rotation_global_to_local()<<std::endl
                 <<"node 2      :"<<node_label[1]<<std::endl
                 <<"position    :reference :"<<base_label<<": "<<relativfe_frame.get_position()<<std::endl
                 <<"orientation :reference :"<<base_label<<": "<<relativfe_frame.get_rotation_global_to_local()<<std::endl
                 <<"spring coeff:"<<spring_coefficient<<std::endl
                 <<"dumper coeff:"<<dumper_coefficient<<std::endl<<std::endl<<std::endl; 
}

ClampJoint::ClampJoint(int elemlabel, int nodelabel, int outflag) 
: elem_label(elemlabel), node_label(nodelabel), output_flag(outflag) {};

ClampJoint::ClampJoint(const ClampJoint &cjt) 
: elem_label(cjt.elem_label), node_label(cjt.node_label), output_flag(cjt.output_flag) {};

ClampJoint &
ClampJoint::operator=(const ClampJoint &cjt) {
    elem_label = cjt.elem_label;
    node_label = cjt.node_label;
    output_flag = cjt.output_flag;
    return *this;
}

void
ClampJoint::write_in_file(std::ofstream &ofs) const {
    ofs << std::scientific<<std::setprecision(5)
        << "joint :" << elem_label << "," <<std::endl
        << "    clamp," << std::endl
        << "    " << node_label <<", node, node,"<<std::endl
        << "    output, "<<output_flag<<";"<<std::endl<<std::endl;   
}