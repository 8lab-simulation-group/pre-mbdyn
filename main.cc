#include "InputData.h"
#include "Platform.h"
#include "Tower.h"
#include "RNA.h"

void output(Platform &ptfm, Tower &Tower, RNA &RNA);

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
    RNA RNA(nacelle_label, Tower.get_top_reference(), &inputdata);
    output(platform , Tower, RNA);

    return 0;
}

void output(Platform &platform, Tower &Tower, RNA &RNA) {
    const std::string out_file_name_base = "NREL5MW_OC3Hywind_MBDyn";

    //--------------output Reference----------------------------
    const std::string out_file_name_ref = out_file_name_base + std::string(".ref");
    // object of ofstream 
    std::ofstream writing_file_reference;
    // file open
    writing_file_reference.open(out_file_name_ref, std::ios::out);

    // writing reference in the file
    platform.write_reference_in(writing_file_reference);
    Tower.write_reference_in(writing_file_reference);
    RNA.write_reference_in(writing_file_reference);

    //------------output nodes -----------------------------------
    const std::string out_file_name_node = out_file_name_base + std::string(".nod");
    std::ofstream writing_file_node;

    writing_file_node.open(out_file_name_node, std::ios::out);

    platform.write_nodes_in(writing_file_node);
    Tower.write_nodes_in(writing_file_node);
    RNA.write_nodes_in(writing_file_node);

    //-----------output element -------------------------------------
    const std::string out_file_name_elem = out_file_name_base + std::string(".elm");
    std::ofstream writing_file_elem;

    writing_file_elem.open(out_file_name_elem, std::ios::out);

    platform.write_elements_in(writing_file_elem);
    Tower.write_elements_in(writing_file_elem);
    RNA.write_elements_in(writing_file_elem);

}