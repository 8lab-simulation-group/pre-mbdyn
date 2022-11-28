#include "Frame.h"

Frame::Frame(const int label, const Vec3d &x, const Vec3d &euler_angle, const std::string &euler_name, const Vec3d &v, const Vec3d &w) 
: b_printed(false), 
  m_frame_label(label),
  m_position(x),
  m_orientation_expression(euler_name),
  m_trans_velocity(v),
  m_angular_velocity(w)
{
    m_euler_angle = euler_angle *( M_1_PI / 180.0);
    m_rotation_global_to_local = make_orientation_from_euler(m_euler_angle, euler_name);
    m_rotation_local_to_global = m_rotation_global_to_local.T();
}

Frame::Frame(const int label, const Vec3d &x, const Vec3d &e1, const Vec3d &e3, const Vec3d &v, const Vec3d &w) 
: b_printed(false), 
  m_frame_label(label),
  m_position(x),
  m_trans_velocity(v),
  m_angular_velocity(w)
{
    m_e1_o = e1/e1.norm();
    m_e3_o = e3/e3.norm();
    m_orientation_expression = "SubVector";

    m_rotation_global_to_local = make_orientation_from_subvector(e1, e3);
    m_rotation_local_to_global = m_rotation_global_to_local.T();
}

Frame::Frame(const int label, const Vec3d &x, const Mat3x3d &A_bo, const Vec3d &v, const Vec3d &w)
: b_printed(false), 
  m_frame_label(label),
  m_position(x),
  m_rotation_global_to_local(A_bo),
  m_trans_velocity(v),
  m_angular_velocity(w)
{
    m_e1_o = m_rotation_global_to_local.get_row(0);
    m_e3_o = m_rotation_global_to_local.get_row(2);

    m_orientation_expression = "SubVector";
    m_rotation_local_to_global = m_rotation_global_to_local.T();
}

Mat3x3d 
Frame::make_orientation_from_euler(const Vec3d &euler, const std::string &name) {

    if(name == "Euler123") {

        Mat3x3d R_1 = make_single_axis_rotation(euler[0], 1);  // 1軸周りに基本回転
        Mat3x3d R_2 = make_single_axis_rotation(euler[1], 2);  // 2軸周りに基本回転
        Mat3x3d R_3 = make_single_axis_rotation(euler[2], 3);  // 3軸周りに基本回転

        return R_3 * (R_2 * R_1);

    }else if(name == "Euler321") {

        Mat3x3d R_1 = make_single_axis_rotation(euler[0], 3); // 3軸周りに基本回転
        Mat3x3d R_2 = make_single_axis_rotation(euler[1], 2); // 2軸周りに基本回転
        Mat3x3d R_3 = make_single_axis_rotation(euler[2], 1); // 1軸周りに基本回転

        return R_3 * (R_2 * R_1);

    }else if(name == "Euler313") {

        Mat3x3d R_1 = make_single_axis_rotation(euler[0], 3); // 3軸周りに基本回転
        Mat3x3d R_2 = make_single_axis_rotation(euler[1], 1); // 1軸周りに基本回転
        Mat3x3d R_3 = make_single_axis_rotation(euler[2], 3); // 3軸周りに基本回転

        return R_3 * (R_2 * R_1);

    }else {
        std::cerr<<"The euler angle name of Class Frame is invalid "<<std::endl;
        return zero3x3;
    }
}
Mat3x3d
Frame::make_single_axis_rotation(const double thita, const int axis) {
    if(axis == 1){

        return Mat3x3d(  1,        0.,                0.,
                         0.,  std::cos(thita), std::sin(thita),
                         0., -std::sin(thita), std::cos(thita)   );
    }else if(axis == 2){

        return Mat3x3d(  std::cos(thita), 0.,  -std::sin(thita),
                              0.,         1.,      0., 
                         std::sin(thita), 0.,   std::cos(thita)  );
    }else if(axis == 3){

        return Mat3x3d( std::cos(thita), std::sin(thita), 0.,
                       -std::sin(thita), std::cos(thita), 0.,
                             0.,            0.,           1.     );
    } else {
        std::cerr<<"the axis of function make_single_axix_rotation is invalid"<<std::endl;
        return zero3x3;
    }
}
Mat3x3d
Frame::make_orientation_from_subvector(const Vec3d &e1, const Vec3d &e3) {
    const Vec3d e2 = e3.cross(e1);
    return Mat3x3d(e1[0], e1[1], e1[2], e2[0], e2[1], e2[2], e3[0], e3[1], e3[2]);
}

Frame::Frame(const Frame &flm) {
    m_frame_label = flm.m_frame_label;
    m_position =  flm.m_position;
    m_euler_angle = flm.m_euler_angle;
    m_e1_o = flm.m_e1_o;
    m_e3_o = flm.m_e3_o;
    m_orientation_expression = flm.m_orientation_expression;

    m_trans_velocity = flm.m_trans_velocity;
    m_angular_velocity = flm.m_angular_velocity;
    m_rotation_global_to_local = flm.m_rotation_global_to_local; 
}

Frame&
Frame::operator=(const Frame &flm) {
    m_frame_label = flm.m_frame_label;
    m_position =  flm.m_position;
    m_euler_angle = flm.m_euler_angle;
    m_e1_o = flm.m_e1_o;
    m_e3_o = flm.m_e3_o;
    m_orientation_expression = flm.m_orientation_expression;

    m_trans_velocity = flm.m_trans_velocity;
    m_angular_velocity = flm.m_angular_velocity;
    m_rotation_global_to_local = flm.m_rotation_global_to_local; 

    return *this;
}
bool
Frame::is_printed() {
    if(b_printed){
        return true;
    } else {
        return false;
    }
}
void 
Frame::write_reference(std::ofstream &ofs) const {
    if(m_orientation_expression=="Euler321") {
        ofs<<"reference      :"<<m_frame_label <<","<<std::endl
           <<std::scientific<<std::setprecision(5)
           <<"    reference, global, "<< m_position <<","<<std::endl
           <<"    reference, global, euler321,"<<std::endl
           <<"                       "<< m_euler_angle << "," <<std::endl
           <<"    reference, global, "<< m_trans_velocity  << ","<<std::endl
           <<"    reference, global, "<< m_angular_velocity <<";"<<std::endl<<std::endl;

    }else if(m_orientation_expression=="SubVector") {
        ofs<<"reference      :"<<m_frame_label <<","<<std::endl
           <<std::scientific<<std::setprecision(5)
           <<"    reference, global, "<< m_position <<","<<std::endl
           <<"    reference, global, "<< std::endl
           <<"                    1, "<< m_e1_o << "," <<std::endl
           <<"                    3, "<< m_e3_o << "," <<std::endl
           <<"    reference, global, "<< m_trans_velocity <<","<<std::endl
           <<"    reference, global, "<< m_angular_velocity <<";"<<std::endl<<std::endl;
    }
}

void 
Frame::print_reference() {
    if(m_orientation_expression=="Euler321") {
        std::cout<<"reference      :"<<m_frame_label <<","<<std::endl
           <<std::scientific<<std::setprecision(5)
           <<"    reference, global, "<< m_position <<","<<std::endl
           <<"    reference, global, euler321, degrees,"<<std::endl
           <<"                       "<< m_euler_angle << "," <<std::endl
           <<"    reference, global, "<< m_trans_velocity  << ","<<std::endl
           <<"    reference, global, "<< m_angular_velocity <<";"<<std::endl<<std::endl;

    }else if(m_orientation_expression=="SubVector") {
        std::cout<<"reference      :"<<m_frame_label <<","<<std::endl
           <<std::scientific<<std::setprecision(5)
           <<"    reference, global, "<< m_position <<","<<std::endl
           <<"    reference, global, "<< std::endl
           <<"                    1, "<< m_e1_o << "," <<std::endl
           <<"                    3, "<< m_e3_o << "," <<std::endl
           <<"    reference, global, "<< m_trans_velocity <<","<<std::endl
           <<"    reference, global, "<< m_angular_velocity <<";"<<std::endl<<std::endl;
    }
}