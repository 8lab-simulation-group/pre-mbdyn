#include "InputData.h"
#include "Platform.h"
#include "Tower.h"
#include "Nacelle.h"
#include "Blade.h"

//void output(Platform &ptfm, Tower &Tower, RNA &RNA, Blade &Blade1, Blade &Blade2, Blade &Blade3);
void output(Platform &ptfm, Tower &Tower, Nacelle &nacelle);

int main(int argc, char *argv[]){

    //std::string input_file = std::string(argv[1]);
    std::string input_file = "pre-MBDyn.ipt";
    // object of Class InputData
    InputData inputdata(input_file);

    const int platform_label = 1000;
    const int tower_label    = 2000;
    const int nacelle_label  = 3000;
    const int blade_label    =10000;

    // コンストラクタ内でデータを加工し、mbdynのフォームに会わせてアウトプット可能な形式でデータを保管する
    Platform platform(platform_label, &inputdata);

    Tower Tower(tower_label, platform.get_top_reference(), &inputdata);

    Nacelle Nacelle(nacelle_label, Tower.get_top_node(), &inputdata);
    /*
    Blade Blade1(1, blade_label, RNA.get_top_reference(1), &inputdata);
    Blade Blade2(2, blade_label + 10000, RNA.get_top_reference(2), &inputdata);
    Blade Blade3(3, blade_label + 20000, RNA.get_top_reference(3), &inputdata);
    */
    output(platform , Tower, Nacelle);

    return 0;
}

void output(Platform &platform, Tower &Tower, Nacelle &Nacelle) {
    const std::string out_file_name_base = "NREL5MW_OC3Hywind_MBDyn";

    //--------------output Reference----------------------------
    const std::string out_file_name_ref = out_file_name_base + std::string(".ref");
    // object of ofstream 
    std::ofstream writing_file_reference;
    // file open
    writing_file_reference.open(out_file_name_ref, std::ios::out);

    // writing reference in the file
    writing_file_reference<<"#----Platform--------------"<<std::endl;
    platform.write_reference_in(writing_file_reference);

    writing_file_reference<<"#----Tower--------------"<<std::endl;
    Tower.write_reference_in(writing_file_reference);

    writing_file_reference<<"#----Nacelle Hub--------------"<<std::endl;
    Nacelle.write_reference_in(writing_file_reference);

    /*
    writing_file_reference<<"#----Blade 1--------------"<<std::endl;
    Blade1.write_reference_in(writing_file_reference);
    writing_file_reference<<"#----Blade 2--------------"<<std::endl;
    Blade2.write_reference_in(writing_file_reference);
    writing_file_reference<<"#----Blade 3--------------"<<std::endl;
    Blade3.write_reference_in(writing_file_reference);   
    */
    //------------output nodes -----------------------------------
    const std::string out_file_name_node = out_file_name_base + std::string(".nod");
    std::ofstream writing_file_node;

    writing_file_node.open(out_file_name_node, std::ios::out);

    writing_file_node<<"# number of nodes platform :"<<platform.get_num_nodes()<<std::endl;
    writing_file_node<<"# number of nodes Tower    :"<<Tower.get_num_nodes()<<std::endl;
    writing_file_node<<"# number of nodes Nacelle  :"<<Nacelle.get_num_nodes()<<std::endl;

    writing_file_node<<"#----Platform--------------"<<std::endl;
    platform.write_nodes_in(writing_file_node);

    writing_file_node<<"#----Tower--------------"<<std::endl;
    Tower.write_nodes_in(writing_file_node);

    writing_file_node<<"#----Nacelle Hub--------------"<<std::endl;
    Nacelle.write_nodes_in(writing_file_node);

    /*
    writing_file_node<<"#----Blade 1--------------"<<std::endl;
    Blade1.write_nodes_in(writing_file_node);
    writing_file_node<<"#----Blade 2--------------"<<std::endl;
    Blade2.write_nodes_in(writing_file_node);
    writing_file_node<<"#----Blade 3--------------"<<std::endl;
    Blade3.write_nodes_in(writing_file_node);
    */

    //-----------output element -------------------------------------
    //-----------output rigidbody -------------------------------------
    const std::string out_file_name_rbd = out_file_name_base + std::string(".rbd");
    std::ofstream writing_file_rbd;

    writing_file_rbd.open(out_file_name_rbd, std::ios::out);

    writing_file_rbd<<"# number of bodies platform :"<<platform.get_num_rigid_bodies()<<std::endl;
    writing_file_rbd<<"# number of bodies tower    :"<<Tower.get_num_rigid_bodies()<<std::endl;
    writing_file_rbd<<"# number of bodies nacelle  :"<<Nacelle.get_num_rigid_bodies()<<std::endl;
    platform.write_rigidbodies_in(writing_file_rbd);
    Tower.write_rigidbodies_in(writing_file_rbd);
    Nacelle.write_rigidbodies_in(writing_file_rbd);



    const std::string out_file_name_jnt = out_file_name_base + std::string(".jnt");
    std::ofstream writing_file_jnt;

    writing_file_jnt.open(out_file_name_jnt, std::ios::out);

    writing_file_jnt<<"# number of bodies platform :"<<platform.get_num_joints()<<std::endl;
    writing_file_jnt<<"# number of bodies tower    :"<<Tower.get_num_joints()<<std::endl;
    writing_file_jnt<<"# number of bodies nacelle  :"<<Nacelle.get_num_joints()<<std::endl;

    platform.write_joints_in(writing_file_jnt);
    Tower.write_joints_in(writing_file_jnt);
    Nacelle.write_joints_in(writing_file_jnt);

    /*
    writing_file_elem<<"#----Blade 1--------------"<<std::endl;
    Blade1.write_elements_in(writing_file_elem);
    writing_file_elem<<"#----Blade 2--------------"<<std::endl;
    Blade2.write_elements_in(writing_file_elem);
    writing_file_elem<<"#----Blade 3--------------"<<std::endl;
    Blade3.write_elements_in(writing_file_elem);
    */
}