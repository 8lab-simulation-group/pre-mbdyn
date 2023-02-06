#include "Node.h"

DummyNode::DummyNode(int label_, const Node &node_, const Frame &offset_ ,int outflag_) {
    dummynode_label = label_;
    node = node_;
    reference_label = node_.get_label();
    offset = offset_;
    outflag = outflag_;
}

DummyNode::DummyNode(int label_, const Node &node_, const ReferenceFrame &frame_ ,int outflag_) {
    dummynode_label = label_;
    node = node_;
    reference_label = frame_.get_base_frame().get_label();
    offset = frame_.get_relative_frame();
    outflag = outflag_;
}


DummyNode::DummyNode(const DummyNode &dummy) {
    dummynode_label = dummy.dummynode_label;
    node = dummy.node;
    reference_label = dummy.reference_label;
    offset = dummy.offset;
    outflag = dummy.outflag;
}

DummyNode &
DummyNode:: operator = (const DummyNode &dummy) {
    dummynode_label = dummy.dummynode_label;
    node = dummy.node;
    reference_label = dummy.reference_label;
    offset = dummy.offset;
    outflag = dummy.outflag;

    return *this;    
}

void
DummyNode::write_node(std::ofstream &ofs) const {

    Vec3d offset_position = offset.get_position();
    Vec3d offset_e1 = offset.get_vector_e1();
    Vec3d offset_e3 = offset.get_vector_e3();

    ofs << std::scientific << std::setprecision(5)
        << "structural :" << dummynode_label << "," <<std::endl
        << "    dummy, " << node.get_label() << "," << std::endl
        << "    offset," << std::endl
        << "    reference, " << reference_label << "," << offset_position << ","<<std::endl
        << "    reference, " << reference_label << "," << std::endl
        << "               1, " << offset_e1 << "," << std::endl
        << "               3, " << offset_e3 << "," << std::endl
        << "    output, " << outflag << ";" << std::endl << std::endl;

}

void
DummyNode::print_node() const {

    Vec3d offset_position = offset.get_position();
    Vec3d offset_e1 = offset.get_vector_e1();
    Vec3d offset_e3 = offset.get_vector_e3();

    std::cout << std::scientific << std::setprecision(5)
        << "structural :" << dummynode_label << "," <<std::endl
        << "    dummy, " << node.get_label() << "," << std::endl
        << "    offset," << std::endl
        << "    reference, " << reference_label << "," << offset_position << ","<<std::endl
        << "    reference, " << reference_label << "," << std::endl
        << "               1, " << offset_e1 << "," << std::endl
        << "               3, " << offset_e3 << "," << std::endl
        << "    output, " << outflag << ";" << std::endl << std::endl;

}