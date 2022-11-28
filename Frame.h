#ifndef __FRAME_H_INCLUDE_
#define __FRAME_H_INCLUDE_

#include "Mat3x3d.h"
#include <iomanip>

class Frame {

    protected:
    int m_frame_label;
    Vec3d m_position;
    Vec3d m_euler_angle;
    Vec3d m_e1_o;
    Vec3d m_e3_o;

    std::string m_orientation_expression;

    Vec3d m_trans_velocity;
    Vec3d m_angular_velocity;

    /* rotation matrix R , vector expressed in global fram V_O, vector expressed in local frame V_B
     * V_B = R * V_O,   V_O = R^T * V_B
    */ 
    Mat3x3d m_rotation_global_to_local; 
    Mat3x3d m_rotation_local_to_global;

    bool b_printed;

public:
    Frame(){};
    Frame(const int label, const Vec3d &x, const Vec3d &euler_angle, const std::string &euler_name, const Vec3d &v, const Vec3d &w);
    Frame(const int label, const Vec3d &x, const Vec3d &e1, const Vec3d &e3, const Vec3d &v, const Vec3d &w);
    Frame(const int label, const Vec3d &x, const Mat3x3d &A_bo, const Vec3d &v, const Vec3d &w);
    Frame(const Frame &flm);
    ~Frame(){};
    Frame& operator=(const Frame &flm);
    
    void write_reference(std::ofstream &ofs) const;
    void print_reference();

private:
    Mat3x3d make_orientation_from_euler(const Vec3d& euler_angle, const std::string &euler_name);
    Mat3x3d make_orientation_from_subvector(const Vec3d &e1, const Vec3d &e3);
    Mat3x3d make_single_axis_rotation(const double thita, const int axis);

public:
    bool is_printed();
    double get_label() const {return m_frame_label;};
    Vec3d get_position() const {return m_position;};
    Vec3d get_eular() const {return m_euler_angle;};
    Vec3d get_vector_e1() const {return m_e1_o;};
    Vec3d get_vector_e3() const {return m_e3_o;};
    std::string get_orientation_express() const {return m_orientation_expression;};
    Vec3d get_velocity() const {return m_trans_velocity;};
    Vec3d get_anglar_velocity() const {return m_angular_velocity;};
    Mat3x3d get_rotation_global_to_local() const {return m_rotation_global_to_local;};
    Mat3x3d get_rotation_local_to_global() const {return m_rotation_local_to_global;};

};

const Frame global_frame(0, zero3, eye3x3, zero3, zero3 );
const Frame offset_null(0 ,zero3, eye3x3, zero3, zero3);
#endif