#include "Node.h"


Node::Node(int label, const Frame &baseframe, const Frame &offset, int outflag) : ReferenceFrame(label, baseframe, offset){
    node_label = label;
    output_flg = outflag;
}

Node::Node(int label, const ReferenceFrame &baseframe, const Frame &offset, int outflag) : ReferenceFrame(label, baseframe.get_frame(), offset) {
    node_label = label;
    output_flg = outflag;
}

Node::Node(int label, const ReferenceFrame &frame, int outflag) : ReferenceFrame(frame){
    node_label = label;
    output_flg = outflag;
}

Node::Node(const Node &node) : ReferenceFrame(node) {
    node_label = node.node_label;
    output_flg = node.output_flg;    
}

Node &
Node::operator=(const Node &node) {
    node_label = node.node_label;
    output_flg = node.output_flg;
    ReferenceFrame::operator=(node);

    return *this;
}

void 
Node::write_node(std::ofstream &ofs) const {
    int reference_label = m_base_frame.get_label();
    Vec3d offset_position = m_relative_frame.get_position();
    Vec3d offset_e1 = m_relative_frame.get_vector_e1();
    Vec3d offset_e3 = m_relative_frame.get_vector_e3();
    Vec3d offset_velocity = m_relative_frame.get_velocity();
    Vec3d offset_angular_velocity = m_relative_frame.get_anglar_velocity();

    ofs << std::scientific<<std::setprecision(5)
        << "structural :" << node_label << "," << std::endl
        << "    dynamic," << std::endl
        << "    reference, " << reference_label << ", " << offset_position << ", " << std::endl
        << "    reference, " << reference_label << ", " << std::endl
        << "            1, " << offset_e1 << ", "<<std::endl
        << "            3, " << offset_e3 << ", "<<std::endl
        << "    reference, " << reference_label << ", " << offset_velocity << ", " << std::endl
        << "    reference, " << reference_label << ", " << offset_angular_velocity << ", " << std::endl
        << "    output, " << output_flg << ";"<<std::endl<<std::endl;
}
    
void 
Node::print_node() {
    int reference_label = m_base_frame.get_label();
    Vec3d offset_position = m_relative_frame.get_position();
    Vec3d offset_e1 = m_relative_frame.get_vector_e1();
    Vec3d offset_e3 = m_relative_frame.get_vector_e3();
    Vec3d offset_velocity = m_relative_frame.get_velocity();
    Vec3d offset_angular_velocity = m_relative_frame.get_anglar_velocity();

    std::cout << std::scientific<<std::setprecision(5)
        << "structural :" << node_label << "," << std::endl
        << "    dynamic," << std::endl
        << "    reference, " << reference_label << ", " << offset_position << ", " << std::endl
        << "    refefence, " << reference_label << ", " << std::endl
        << "            1, " << offset_e1 << ", "<<std::endl
        << "            3, " << offset_e3 << ", "<<std::endl
        << "    reference, " << reference_label << ", " << offset_velocity << ", " << std::endl
        << "    reference, " << reference_label << ", " << offset_angular_velocity << ", " << std::endl
        << "    output, " << output_flg << ";"<<std::endl<<std::endl;
}

StaticNode::StaticNode(int label, int reflabel, int outflg) 
: node_label(label), reference_label(reflabel), output_flag(outflg) {};

StaticNode::StaticNode(const StaticNode &snd)
: node_label(snd.node_label), reference_label(snd.reference_label), output_flag(snd.output_flag) {};

StaticNode &
StaticNode::operator=(const StaticNode &snd) {
    node_label = snd.node_label;
    reference_label = snd.reference_label;
    output_flag = snd.output_flag;
    return *this;
}

void 
StaticNode::write_node(std::ofstream &ofs) const {

    ofs << std::scientific<<std::setprecision(5)
        << "structural :" << node_label << "," << std::endl
        << "    static," << std::endl
        << "    reference, " << reference_label << ", null," <<std::endl
        << "    reference, " << reference_label << ", eye, " << std::endl
        << "    reference, " << reference_label << ", null," << std::endl
        << "    reference, " << reference_label << ", null," << std::endl
        << "    output, " << output_flag << ";"<<std::endl<<std::endl;
}
