#include "Platform_deformable.h"
#include <cmath>
#include <math.h>

Platform::Platform(int label, InputData *ID)
: platform_label(label), inputdata(ID) // const メンバ変数を引数で初期化
{
    // inputdata クラスより，各種変数を取得
    int num_section = inputdata->get_value("NPtfmDivH");

    m_section.resize(num_section);
    m_connector.resize(num_section - 1);

    // set platform section
    for(auto section : m_section) {
      
    }

    set_reference();
    set_nodes();
    set_rigidbodies();
    set_deformablejoint();
    set_total_joint();
}

void
Platform::set_reference() {
    for(int i=0; i<num_nodes; i++) {

}

void
Platform::set_nodes(){

}

void
Platform::set_rigidbodies() {

}

void
Platform::set_deformablejoint() {

}

void
Platform::set_total_joint() {


}

void
Platform::write_reference_in(std::ofstream &output_file) const {

}

void
Platform::write_nodes_in(std::ofstream &output_file) const {

}


void
Platform::write_rigidbodies_in(std::ofstream &output_file) const {

}

void
Platform::write_joints_in(std::ofstream &output_file) const {

}

int
Platform::get_num_nodes() const {

}

int
Platform::get_num_rigid_bodies() const {

}

int
Platform::get_num_joints() const {

}