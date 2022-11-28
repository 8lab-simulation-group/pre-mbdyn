#include "ReferenceFrame.h"

ReferenceFrame::ReferenceFrame(int label, const Frame &bsflm, const Frame &relflm){
    base_frame = bsflm;
    relative_frame = relflm;
    reference_frame = make_reference_frame(label, bsflm, relflm);   
}

ReferenceFrame::ReferenceFrame(int label, const ReferenceFrame &bsflm, const Frame &relflm) {
    base_frame = bsflm.get_frame();
    relative_frame = relflm;
    reference_frame = make_reference_frame(label, base_frame, relflm);    
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
{
    base_frame = refflm.base_frame;
    relative_frame = refflm.relative_frame;
    reference_frame = refflm.reference_frame;
}

ReferenceFrame&
ReferenceFrame::operator=(const ReferenceFrame &refflm) {
    base_frame = refflm.base_frame;
    relative_frame = refflm.relative_frame;
    reference_frame = refflm.reference_frame;  
    return *this;  
}

void 
ReferenceFrame::write_reference(std::ofstream &ofs) const{
    std::string orientation_expression = relative_frame.get_orientation_express();
    int frame_label = reference_frame.get_label();
    int base_label = base_frame.get_label();
    Vec3d offset_position = relative_frame.get_position();
    Vec3d offset_e1 = relative_frame.get_vector_e1();
    Vec3d offset_e3 = relative_frame.get_vector_e3();
    Vec3d offset_velocity = relative_frame.get_velocity();
    Vec3d offset_angular_velocity = relative_frame.get_anglar_velocity();

    ofs<<"reference      :"<<frame_label <<","<<std::endl
       <<std::scientific<<std::setprecision(5)
       <<"    reference, " <<base_label<<", "<< offset_position <<","<<std::endl
       <<"    reference, " <<base_label<<", "<< std::endl
       <<"                 1, "<< offset_e1 << "," <<std::endl
       <<"                 3, "<< offset_e3 << "," <<std::endl
       <<"    reference, " <<base_label<<", "<< offset_velocity <<","<<std::endl
       <<"    reference, " <<base_label<<", "<< offset_angular_velocity <<";"<<std::endl<<std::endl;
}

void 
ReferenceFrame::print_reference(const int mode) {
    if(mode == 0) {
        if(relative_frame.get_orientation_express() == "Eular321") {
            std::cout<<"label          :"<<relative_frame.get_label() <<std::endl
                     <<"positon        :"<<" reference :"<<base_frame.get_label()<<" :"<<relative_frame.get_position() <<std::endl
                     <<"eular32        :"<<" reference :"<<base_frame.get_label()<<" :"<<relative_frame.get_eular() <<std::endl
                     <<"velocity       :"<<" reference :"<<base_frame.get_label()<<" :"<<relative_frame.get_velocity() <<std::endl
                     <<"anglar velosity:"<<" reference :"<<base_frame.get_label()<<" :"<<relative_frame.get_anglar_velocity() <<std::endl
                     <<"Rotation matrix:"<<" reference :"<<base_frame.get_label()<<" :"<<relative_frame.get_rotation_global_to_local()<<std::endl<<std::endl;
        
        }else if(relative_frame.get_orientation_express() == "SubVector") {
            std::cout<<"label          :"<<relative_frame.get_label() <<std::endl
                     <<"positon        :"<<" reference :"<<base_frame.get_label()<<" :"<<relative_frame.get_position() <<std::endl
                     <<"e1             :"<<" reference :"<<base_frame.get_label()<<" :"<<relative_frame.get_vector_e1() <<std::endl
                     <<"e3             :"<<" reference :"<<base_frame.get_label()<<" :"<<relative_frame.get_vector_e3() <<std::endl
                     <<"velocity       :"<<" reference :"<<base_frame.get_label()<<" :"<<relative_frame.get_velocity() <<std::endl
                     <<"anglar velosity:"<<" reference :"<<base_frame.get_label()<<" :"<<relative_frame.get_anglar_velocity() <<std::endl
                     <<"Rotation matrix:"<<" reference :"<<base_frame.get_label()<<" :"<<relative_frame.get_rotation_global_to_local()<<std::endl<<std::endl;
        }

    } else {
        reference_frame.print_reference();
    }
}

