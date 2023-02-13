#include "ReferenceFrame.h"

ReferenceFrame::ReferenceFrame(int label, const Frame &bsflm, const Frame &relflm)
:Frame(make_reference_frame(label, bsflm, relflm))
{
    m_base_frame = bsflm;
    m_relative_frame = relflm;   
}

ReferenceFrame::ReferenceFrame(int label, const ReferenceFrame &bsflm, const Frame &relflm) 
:Frame(make_reference_frame(label, bsflm.get_frame(), relflm))
{
    m_base_frame = bsflm.get_frame();
    m_relative_frame = relflm;    
}

Frame
ReferenceFrame::make_reference_frame(int label,const Frame &baseflm, const Frame &relativeflm) {

    Vec3d position = baseflm.get_rotation_local_to_global() * relativeflm.get_position() + baseflm.get_position();
    Mat3x3d orientation = relativeflm.get_rotation_global_to_local() * baseflm.get_rotation_global_to_local(); // A_bo = A_bc * A_co
    Vec3d velocity =  baseflm.get_rotation_local_to_global() * relativeflm.get_velocity() + baseflm.get_velocity();
    Vec3d anguler_v =  baseflm.get_rotation_local_to_global() * relativeflm.get_anglar_velocity() + baseflm.get_anglar_velocity();

    return Frame(label, position, orientation, velocity, anguler_v);
}

ReferenceFrame::ReferenceFrame(const ReferenceFrame &refflm)  
: Frame(refflm)
{
    m_base_frame = refflm.m_base_frame;
    m_relative_frame = refflm.m_relative_frame;
}

ReferenceFrame&
ReferenceFrame::operator=(const ReferenceFrame &refflm) {
    Frame::operator=(refflm);
    m_base_frame = refflm.m_base_frame;
    m_relative_frame = refflm.m_relative_frame;
    return *this;  
}

void 
ReferenceFrame::write_reference(std::ofstream &ofs) const{
    std::string orientation_expression = m_relative_frame.get_orientation_express();

    int base_label = m_base_frame.get_label();
    Vec3d offset_position = m_relative_frame.get_position();
    Vec3d offset_e1 = m_relative_frame.get_vector_e1();
    Vec3d offset_e3 = m_relative_frame.get_vector_e3();
    Vec3d offset_velocity = m_relative_frame.get_velocity();
    Vec3d offset_angular_velocity = m_relative_frame.get_anglar_velocity();

    ofs<<"reference      :"<<m_frame_label <<","<<std::endl
       <<std::scientific<<std::setprecision(5)
       <<"    reference, " <<base_label<<", "<< offset_position <<","<<std::endl
       <<"    reference, " <<base_label<<", "<< std::endl
       <<"                 1, "<< offset_e1 << "," <<std::endl
       <<"                 3, "<< offset_e3 << "," <<std::endl
       <<"    reference, " <<base_label<<", "<< offset_velocity <<","<<std::endl
       <<"    reference, " <<base_label<<", "<< offset_angular_velocity <<";"<<std::endl<<std::endl;
}

void 
ReferenceFrame::print_reference() {
    std::string orientation_expression = m_relative_frame.get_orientation_express();

    int base_label = m_base_frame.get_label();
    Vec3d offset_position = m_relative_frame.get_position();
    Vec3d offset_e1 = m_relative_frame.get_vector_e1();
    Vec3d offset_e3 = m_relative_frame.get_vector_e3();
    Vec3d offset_velocity = m_relative_frame.get_velocity();
    Vec3d offset_angular_velocity = m_relative_frame.get_anglar_velocity();

    std::cout<<"reference      :"<<m_frame_label <<","<<std::endl
       <<std::scientific<<std::setprecision(5)
       <<"    reference, " <<base_label<<", "<< offset_position <<","<<std::endl
       <<"    reference, " <<base_label<<", "<< std::endl
       <<"                 1, "<< offset_e1 << "," <<std::endl
       <<"                 3, "<< offset_e3 << "," <<std::endl
       <<"    reference, " <<base_label<<", "<< offset_velocity <<","<<std::endl
       <<"    reference, " <<base_label<<", "<< offset_angular_velocity <<";"<<std::endl<<std::endl;
}

void 
ReferenceFrame::print_reference_global() {
    Frame::print_reference();
}
